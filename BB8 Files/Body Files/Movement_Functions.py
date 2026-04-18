import time
import os
import busio
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'
# from gpiozero import PWMOutputDevice, OutputDevice
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import PhaseEnableMotor as Motor
from gpiozero import OutputDevice
from PID import PIDController
from IMU import IMUSensor
from Telemetry import TelemetryLogger


class BB8Movement:
    def __init__(self):
        print("Initializing BB-8 Hardware...")
        
        # 1. INITIALIZE SENSORS & PID
        self.imu = IMUSensor()
        
        # Initialize PID with the starter values from your dummy test
        self.balance_pid = PIDController(kp=0.4, ki=0.0, kd=0.5) 
        self.target_pitch = 0.0 # 0 degrees = perfectly upright
        
        # 1. INITIALIZE I2C & PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50 

        # 2. DEFINE MOTORS (Main Drive & Turn)
        # Phase = Direction pin, enable = PWM speed pin
        self.DC_motor1 = Motor(phase=5, enable=13, pwm=True)
        self.DC_motor2 = Motor(phase=6, enable=18, pwm=True)
        self.Turn_motor = Motor(phase=26, enable=19, pwm=True)
        
        # 3. DEFINE SERVOS
        self.swing_servo = servo.Servo(self.pca.channels[0])
        self.head_fb = servo.Servo(self.pca.channels[1])       # Forward/Backward
        self.head_sts = servo.Servo(self.pca.channels[2])      # Side-to-Side
        self.head_rotate = servo.ContinuousServo(self.pca.channels[3], min_pulse=810, max_pulse=2300)

        
        self.swing_servo2 = servo.Servo(self.pca.channels[4])

        # 4. DEFINE RELAYS
        self.relay1 = OutputDevice(20, initial_value=False)
        self.relay2 = OutputDevice(21, initial_value=False)

        # --- STATE VARIABLES ---
        self.current_head_fb = 90
        self.current_head_sts = 90
        self.current_swing = 90
        
        # --- INITIAL VALUES ---
        self.swing_offset = -1.0
        self.head_fb_offset = 2.5
        self.head_sts_offset = -2

        # --- EMA FILTER VARIABLES (HEAD ONLY) ---
        self.smoothed_head_fb = 90.0
        self.smoothed_head_sts = 90.0
        
        # Alpha controls the smoothing speed (0.01 = extremely slow, 1.0 = instant)
        # 0.15 is a great starting point for organic, droid-like head movement
        self.alpha_head = 0.15
        
        # --- Telemetry Logger ---
        self.logger = TelemetryLogger()
        
        # --- SLEW RATE VARIABLES ---
        self.current_drive_val = 0.0
        self.drive_slew_rate = 0.005  # Adjust this: Lower = slower acceleration, Higher = punchier
        
        # --- TURN PULSE VARIABLES (SQUARE WAVE) ---
        self.turn_pulse_period = 0.00001  # Length of one full cycle in seconds (100ms = 10Hz)
        self.turn_duty_cycle = 1.0   # 90% ON, 10% OFF
        
    # --- POWER MANAGEMENT ---
    def enable_system(self):
        self.relay1.on()
        self.relay2.on()
        self.balance_pid.reset() # Prevent any integral windup from before the system was enabled
        pass
    
    def disable_system(self):
        self.relay1.off()
        self.relay2.off()
        self.stop_all()
        self.rest_all_servos()
        
        self.logger.save_to_csv() # Dump telemetry data to disk when the system is disabled

    # --- MAIN DRIVE & STEERING ---
    def drive(self, target_speed):
        """
        Drive the main motors with an acceleration/deceleration ramp.
        'target_speed' is a float from -1.0 (Backward) to 1.0 (Forward).
        """
        max_speed_factor = 1.0
        
        # Calculate the final target value the joystick is asking for
        target_val = max(-1.0, min(1.0, target_speed)) * max_speed_factor
        
        # --- SLEW RATE LIMITER (THE RAMP) ---
        # If target is greater than current, ramp up
        if self.current_drive_val < target_val:
            self.current_drive_val += self.drive_slew_rate
            # Clamp it so we don't overshoot the target
            if self.current_drive_val > target_val:
                self.current_drive_val = target_val
                
        # If target is less than current, ramp down
        elif self.current_drive_val > target_val:
            self.current_drive_val -= self.drive_slew_rate
            # Clamp it so we don't overshoot
            if self.current_drive_val < target_val:
                self.current_drive_val = target_val

        # Apply the smoothly changing value to both hardware motors
        self.DC_motor1.value = self.current_drive_val
        self.DC_motor2.value = self.current_drive_val

    def steer(self, direction):
        """
        Steer the internal pendulum drive using a Square Wave (Stick-Slip).
        'direction' is a float from -1.0 (Full Left) to 1.0 (Full Right).
        """
        max_turn_speed = 0.30  
        target_val = max(-1.0, min(1.0, direction)) * max_turn_speed
        
        # 1. If joystick is centered, stop immediately (no pulsing)
        if abs(target_val) < 0.01:
            self.Turn_motor.value = 0
            return

        # 2. Square Wave Logic
        current_time = time.time()
        # Modulo the current time by the period to find our position in the current wave
        time_in_cycle = current_time % self.turn_pulse_period
        
        # 3. Check if we are in the "ON" phase of the duty cycle
        if time_in_cycle < (self.turn_pulse_period * self.turn_duty_cycle):
            self.Turn_motor.value = target_val  # Pulse ON
        else:
            self.Turn_motor.value = 0           # Pulse OFF
  
        
    def set_swing(self, degrees):
        # --- DEBUG MODE: SWING SERVOS DISABLED & RESTED ---
        # self.pca.channels[0].duty_cycle = 0 # Kill signal to swing_servo
        # self.pca.channels[4].duty_cycle = 0 # Kill signal to swing_servo2
        # return # Exit the function immediately
        # --------------------------------------------------
        """Sets internal pendulum swing. Safe range 70 to 117"""
        self.current_swing = max(70, min(117, degrees))
        difference = self.current_swing - 90 + self.swing_offset
        self.swing_servo.angle = 90 + difference
        self.swing_servo2.angle = 90 - difference

    def update_balance(self):
        """
        THE CORE PID LOOP. First part balances the robot.
        """
        # 1. UPDATE ALL IMU SENSOR DATA AT ONCE
        self.imu.update()

        # 2. Get current lean angle from the stored IMU attributes
        current_pitch = self.imu.pitch

        # Compute the PID correction
        pid_correction = self.balance_pid.compute(self.target_pitch, current_pitch)

        # Log the data to RAM
        self.logger.log_step(current_pitch, self.target_pitch, self.balance_pid)

        # 3. Apply the hardware mapping (90 - correction)
        target_servo_degrees = 90 - pid_correction
        
        # Command the swing servos INSTANTLY (No smoothing for stability)
        self.set_swing(target_servo_degrees)
        
        # --- HEAD LEVELING (WITH EMA SMOOTHING) ---
        # Calculates the roll of the robot for x-axis tilt
        current_roll = self.imu.roll
        
        # Calculate raw TARGET servo angles
        target_head_fb = 90 + current_pitch
        target_head_sts = 90 + current_roll
        
        # Apply the EMA filter (Shock Absorber)
        self.smoothed_head_fb = (self.alpha_head * target_head_fb) + ((1 - self.alpha_head) * self.smoothed_head_fb)
        self.smoothed_head_sts = (self.alpha_head * target_head_sts) + ((1 - self.alpha_head) * self.smoothed_head_sts)
        
        # Send the smoothed data to the servos (rounded to 1 decimal for clean PWM)
        self.set_head_fb(round(self.smoothed_head_fb, 1))
        self.set_head_sts(round(self.smoothed_head_sts, 1))

    # --- HEAD CONTROLS ---
    def set_head_fb(self, degrees):
        """Tilts head forward/backward. Safe range 60 to 120"""        
        self.current_head_fb = max(55, min(125, degrees))
        self.head_fb.angle = self.current_head_fb + self.head_fb_offset

    def set_head_sts(self, degrees):
        """Tilts head side-to-side. Safe range 60 to 120"""
        self.current_head_sts = max(55, min(125, degrees))
        self.head_sts.angle = self.current_head_sts + self.head_sts_offset

    def spin_head(self, throttle):
        """
        Spins the head continuously.
        throttle: -1.0 (Spin Left) to 1.0 (Spin Right). 0.0 is Stop.
        """
        self.head_rotate.throttle = max(-1.0, min(1.0, throttle))

    def execute_step_response_test(self):
        """Automated test to gather Transfer Function data"""
        print("Starting Step Response Test in 3 seconds...")
        time.sleep(3)
        
        # 1. Settle the robot at 0 degrees
        self.target_pitch = 0.0
        self.enable_system()
        
        start_time = time.time()
        while time.time() - start_time < 3.0: # Let it balance for 3 seconds
            self.update_balance()
            # self.logger.log_step(..., ...) # Ensure your logger is running here
            time.sleep(0.01) 
            
        # 2. THE STEP COMMAND
        print("EXECUTING 5-DEGREE STEP!")
        self.target_pitch = 10.0
        
        # 3. Record the dynamic response
        step_time = time.time()
        while time.time() - step_time < 3.0: # Record for 3 seconds
            self.update_balance()
            # self.logger.log_step(..., ...)
            time.sleep(0.01)
            
        # 4. Safely shut down and dump the CSV
        self.disable_system()
        print("Test Complete. Check pid_telemetry.csv")
    
    # --- UTILITY & SAFETY ---
    def stop_all(self):
        """Instantly stops all motors and centers servos"""
        self.DC_motor1.value = 0
        self.DC_motor2.value = 0
        self.Turn_motor.value = 0
        self.head_rotate.throttle = 0
        self.set_swing(90)
        self.set_head_fb(90)
        self.set_head_sts(90)

    def rest_all_servos(self):
        """Kills power to servos to prevent overheating"""
        for i in range(5):
            self.pca.channels[i].duty_cycle = 0