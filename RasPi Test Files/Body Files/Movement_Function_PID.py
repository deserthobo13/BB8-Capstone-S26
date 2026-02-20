import time
import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import PWMOutputDevice, OutputDevice

# Import your newly structured classes!
from IMU import IMUSensor
from PID import PIDController

class BB8Movement:
    def __init__(self):
        print("Initializing BB-8 Hardware...")
        
        # 1. INITIALIZE SENSORS & PID
        self.imu = IMUSensor()
        
        # TODO: You will need to tune these Kp, Ki, Kd values for your specific robot weight/motors
        self.balance_pid = PIDController(kp=0.05, ki=0.001, kd=0.01) 
        self.target_pitch = 0.0 # 0 degrees is perfectly upright
        
        # 2. INITIALIZE I2C & PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50 

        # 3. DEFINE MOTORS
        self.DC_motor1 = PWMOutputDevice(13, initial_value=0.5, frequency=10000)
        self.DC_motor2 = PWMOutputDevice(18, initial_value=0.5, frequency=10000)
        self.Turn_motor = PWMOutputDevice(19, initial_value=0.5, frequency=10000)
        
        self.motor_driver_relay = OutputDevice(17, active_high=False)
        self.turn_driver_relay = OutputDevice(27, active_high=False)

        # 4. DEFINE SERVOS
        self.swing_servo = servo.Servo(self.pca.channels[0])
        self.head_fb = servo.Servo(self.pca.channels[2])
        self.head_sts = servo.Servo(self.pca.channels[1])
        self.head_rotate = servo.ContinuousServo(self.pca.channels[3], min_pulse=810, max_pulse=2300)
        self.swing_servo2 = servo.Servo(self.pca.channels[4])

    # --- POWER & BALANCE MANAGEMENT ---
    def enable_system(self):
        """Turns on relays and resets the PID loop so it doesn't jerk on startup"""
        self.motor_driver_relay.on()
        self.turn_driver_relay.on()
        self.balance_pid.reset() # Clears stale time/error data!
        self.target_pitch = 0.0

    def disable_system(self):
        """Safely stops all movement and cuts power"""
        self.target_pitch = 0.0
        self.DC_motor1.value = 0.5
        self.DC_motor2.value = 0.5
        self.Turn_motor.value = 0.5
        self.head_rotate.throttle = 0
        self.motor_driver_relay.off()
        self.turn_driver_relay.off()

    # --- MAIN DRIVE (SETPOINT CONTROL) ---
    def drive(self, speed):
        """
        Speed (-1.0 to 1.0). 
        Maps speed to a target lean angle for the PID to maintain.
        """
        MAX_LEAN_ANGLE = 15.0 # Max degrees the chassis is allowed to lean
        self.target_pitch = speed * MAX_LEAN_ANGLE

    def update_balance(self):
        """
        THE CORE PID LOOP. Must be called as fast as possible in the main script.
        """
        # 1. Get current lean angle
        current_pitch = self.imu.get_pitch()

        # 2. Compute motor adjustment needed to reach target_pitch
        motor_power = self.balance_pid.compute(self.target_pitch, current_pitch)

        # 3. Apply limits and output to motors (0.5 is stop)
        # Note: You may need to invert motor_power (change + to -) depending on motor wiring polarity
        pwm_val = 0.5 + motor_power
        pwm_val = max(0.0, min(1.0, pwm_val)) # Strictly clamp between 0 and 1
        
        self.DC_motor1.value = pwm_val
        self.DC_motor2.value = pwm_val

    # --- STEERING & HEAD (Direct Control) ---
    # (These remain the same as before because they don't dictate forward/backward balance)
    def steer(self, direction):
        if direction < -0.5: self.Turn_motor.value = 0
        elif direction > 0.5: self.Turn_motor.value = 1
        else: self.Turn_motor.value = 0.5

    def spin_head(self, throttle):
        self.head_rotate.throttle = max(-1.0, min(1.0, throttle))
        
    def stop_all(self):
        self.drive(0.0) # Tell PID to balance upright
        self.Turn_motor.value = 0.5
        self.head_rotate.throttle = 0