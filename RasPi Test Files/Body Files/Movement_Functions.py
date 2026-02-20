import time
import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import PWMOutputDevice, OutputDevice
from PID import PID
from IMU import get_stabilization_data


class BB8Movement:
    def __init__(self):
        print("Initializing BB-8 Hardware...")
        
        # 1. INITIALIZE I2C & PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50 

        # 2. DEFINE MOTORS (Main Drive & Turn)
        # Note: 0.5 is STOP for these PWM motor controllers
        self.DC_motor1 = PWMOutputDevice(13, initial_value=0.5, frequency=10000)
        self.DC_motor2 = PWMOutputDevice(18, initial_value=0.5, frequency=10000)
        self.Turn_motor = PWMOutputDevice(19, initial_value=0.5, frequency=10000)
        
        self.motor_driver_relay = OutputDevice(17, active_high=False)
        self.turn_driver_relay = OutputDevice(27, active_high=False)

        # 3. DEFINE SERVOS
        self.swing_servo = servo.Servo(self.pca.channels[0])
        self.head_fb = servo.Servo(self.pca.channels[2])       # Forward/Backward
        self.head_sts = servo.Servo(self.pca.channels[1])      # Side-to-Side
        self.head_rotate = servo.ContinuousServo(self.pca.channels[3], min_pulse=810, max_pulse=2300)
        self.swing_servo2 = servo.Servo(self.pca.channels[4])

        # --- STATE VARIABLES ---
        self.current_head_fb = 90
        self.current_head_sts = 90
        self.current_swing = 90

    # --- POWER MANAGEMENT ---
    def enable_system(self):
        """Turns on the motor driver relays"""
        self.motor_driver_relay.on()
        self.turn_driver_relay.on()

    def disable_system(self):
        """Turns off relays and safely stops all movement"""
        self.stop_all()
        self.motor_driver_relay.off()
        self.turn_driver_relay.off()
        self.rest_all_servos()

    # --- MAIN DRIVE & STEERING ---
    def drive(self, speed):
        """
        Drive the main motors. 
        speed: -1.0 (Full Reverse) to 1.0 (Full Forward). 0.0 is Stop.
        """
        # Clamping input and applying the 0.15 scale factor from your original code
        speed_val = max(-1.0, min(1.0, speed)) 
        pwm_val = 0.5 + (speed_val * 0.15)
        
        self.DC_motor1.value = pwm_val
        self.DC_motor2.value = pwm_val

    def steer(self, direction):
        """
        Steer the internal pendulum.
        direction: -1 (Left), 1 (Right), 0 (Center)
        """
        if direction < -0.5:
            self.Turn_motor.value = 0 # Turn Left
        elif direction > 0.5:
            self.Turn_motor.value = 1 # Turn Right
        else:
            self.Turn_motor.value = 0.5 # Center

    # --- SWING & BALANCE ---
    def set_swing(self, degrees):
        """Sets internal pendulum swing. Safe range 70 to 117"""
        self.current_swing = max(70, min(117, degrees))
        difference = self.current_swing - 90
        self.swing_servo.angle = 90 + difference
        self.swing_servo2.angle = 90 - difference

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
        self.DC_motor1.value = 0.5
        self.DC_motor2.value = 0.5
        self.Turn_motor.value = 0.5
        self.head_rotate.throttle = 0
        self.set_swing(90)
        self.set_head_fb(90)
        self.set_head_sts(90)

    def rest_all_servos(self):
        """Kills power to servos to prevent overheating"""
        for i in range(5):
            self.pca.channels[i].duty_cycle = 0