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


class BB8Movement:
    def __init__(self):
        print("Initializing BB-8 Hardware...")
        
        # 1. INITIALIZE SENSORS & PID
        self.imu = IMUSensor()
        
        # Initialize PID with the starter values from your dummy test
        self.balance_pid = PIDController(kp=1.0, ki=0.0, kd=0.1) 
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

        # --- EMA FILTER VARIABLES (HEAD ONLY) ---
        self.smoothed_head_fb = 90.0
        self.smoothed_head_sts = 90.0
        
        # Alpha controls the smoothing speed (0.01 = extremely slow, 1.0 = instant)
        # 0.15 is a great starting point for organic, droid-like head movement
        self.alpha_head = 0.15
        
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

    # --- MAIN DRIVE & STEERING ---
    def drive(self, speed):
        """
        Drive the main motors.    
        """
        max_speed_factor = 0.30
        speed_val = max(-1.0, min(1.0, speed)) * max_speed_factor
        
        self.DC_motor1.value = speed_val
        self.DC_motor2.value = speed_val

    def steer(self, direction):
        """
        Steer the internal pendulum drive.
        'direction' is a float from -1.0 (Full Left) to 1.0 (Full Right).
        """
        max_turn_speed = 0.30  # Cap turning speed to 30% for stability
        
        # Clamp input and apply the speed limit
        turn_val = max(-1.0, min(1.0, direction)) * max_turn_speed
        
        # gpiozero handles the signed magnitude (Phase/PWM) automatically
        self.Turn_motor.value = turn_val

    def set_swing(self, degrees):
        """Sets internal pendulum swing. Safe range 70 to 117"""
        self.current_swing = max(70, min(117, degrees))
        difference = self.current_swing - 90
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

        # 3. Apply the hardware mapping (90 - correction)
        target_servo_degrees = 90 - pid_correction
        
        # Command the swing servos INSTANTLY (No smoothing for stability)
        self.set_swing(target_servo_degrees)
        
        # --- HEAD LEVELING (WITH EMA SMOOTHING) ---
        # Calculates the roll of the robot for x-axis tilt
        current_roll = self.imu.roll
        
        # Calculate raw TARGET servo angles
        target_head_fb = 90 - current_pitch
        target_head_sts = 90 - current_roll
        
        # Apply the EMA filter (Shock Absorber)
        self.smoothed_head_fb = (self.alpha_head * target_head_fb) + ((1 - self.alpha_head) * self.smoothed_head_fb)
        self.smoothed_head_sts = (self.alpha_head * target_head_sts) + ((1 - self.alpha_head) * self.smoothed_head_sts)
        
        # Send the smoothed data to the servos (rounded to 1 decimal for clean PWM)
        self.set_head_fb(round(self.smoothed_head_fb, 1))
        self.set_head_sts(round(self.smoothed_head_sts, 1))

    # --- HEAD CONTROLS ---
    def set_head_fb(self, degrees):
        """Tilts head forward/backward. Safe range 55 to 125"""        
        self.current_head_fb = max(55, min(125, degrees))
        self.head_fb.angle = self.current_head_fb + 15

    def set_head_sts(self, degrees):
        """Tilts head side-to-side. Safe range 40 to 140"""
        self.current_head_sts = max(40, min(140, degrees))
        self.head_sts.angle = self.current_head_sts + 5

    def spin_head(self, throttle):
        """
        Spins the head continuously.
        throttle: -1.0 (Spin Left) to 1.0 (Spin Right). 0.0 is Stop.
        """
        self.head_rotate.throttle = max(-1.0, min(1.0, throttle))

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