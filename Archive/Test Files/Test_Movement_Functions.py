import time
import os
import busio
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import PhaseEnableMotor as Motor
from gpiozero import OutputDevice
from PID import PIDController
from IMU import IMUSensor

class TestBB8Movement:
    def __init__(self):
        print("Initializing Test BB-8 Hardware...")
        
        # 1. INITIALIZE SENSORS & PID
        self.imu = IMUSensor()
        self.balance_pid = PIDController(kp=1.0, ki=0.005, kd=0.15) 
        self.target_pitch = 0.0 
        
        # 2. INITIALIZE I2C & PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50 

        # 3. DEFINE MOTORS
        self.DC_motor1 = Motor(phase=5, enable=13, pwm=True)
        self.DC_motor2 = Motor(phase=6, enable=18, pwm=True)
        self.Turn_motor = Motor(phase=26, enable=19, pwm=True)
        
        # 4. DEFINE SERVOS
        self.swing_servo = servo.Servo(self.pca.channels[0])
        self.head_fb = servo.Servo(self.pca.channels[1])       
        self.head_sts = servo.Servo(self.pca.channels[2])      
        self.head_rotate = servo.ContinuousServo(self.pca.channels[3], min_pulse=810, max_pulse=2300)
        self.swing_servo2 = servo.Servo(self.pca.channels[4])

        # 5. DEFINE RELAYS
        self.relay1 = OutputDevice(20, initial_value=False)
        self.relay2 = OutputDevice(21, initial_value=False)

        # --- STATE VARIABLES ---
        self.current_head_fb = 90
        self.current_head_sts = 90
        self.current_swing = 90

        # --- HARDWARE OFFSETS ---
        self.swing_offset = -1.0   
        self.head_fb_offset = 15.0 
        self.head_sts_offset = 45

        # --- EMA FILTER VARIABLES (HEAD ONLY) ---
        self.smoothed_head_fb = 90.0
        self.smoothed_head_sts = 90.0
        self.alpha_head = 0.15
        
    def enable_system(self):
        self.relay1.on()
        self.relay2.on()
        self.balance_pid.reset()
    
    def disable_system(self):
        self.relay1.off()
        self.relay2.off()
        self.stop_all()
        self.rest_all_servos()

    def drive(self, speed):
        max_speed_factor = 0.30
        speed_val = max(-1.0, min(1.0, speed)) * max_speed_factor
        self.DC_motor1.value = speed_val
        self.DC_motor2.value = speed_val

    def steer(self, direction):
        max_turn_speed = 0.30  
        turn_val = max(-1.0, min(1.0, direction)) * max_turn_speed
        self.Turn_motor.value = turn_val

    def set_swing(self, degrees):
        self.current_swing = max(70, min(117, degrees))
        difference = (self.current_swing - 90) + self.swing_offset
        self.swing_servo.angle = 90 + difference
        self.swing_servo2.angle = 90 - difference

    def update_balance(self):
        """
        THE CORE PID LOOP. 
        Head leveling is disabled for manual puppeteering.
        """
        self.imu.update()
        current_pitch = self.imu.pitch
        pid_correction = self.balance_pid.compute(self.target_pitch, current_pitch)
        target_servo_degrees = 90 - pid_correction
        self.set_swing(target_servo_degrees)
        
        # HEAD LEVELING REMOVED HERE

    def set_head_fb(self, degrees):       
        self.current_head_fb = max(55, min(125, degrees))
        self.head_fb.angle = self.current_head_fb + self.head_fb_offset

    def set_head_sts(self, degrees):
        self.current_head_sts = max(40, min(140, degrees))
        self.head_sts.angle = self.current_head_sts + self.head_sts_offset

    def spin_head(self, throttle):
        self.head_rotate.throttle = max(-1.0, min(1.0, throttle))

    def stop_all(self):
        self.DC_motor1.value = 0
        self.DC_motor2.value = 0
        self.Turn_motor.value = 0
        self.head_rotate.throttle = 0
        self.set_swing(90)
        self.set_head_fb(90)
        self.set_head_sts(90)

    def rest_all_servos(self):
        for i in range(5):
            self.pca.channels[i].duty_cycle = 0