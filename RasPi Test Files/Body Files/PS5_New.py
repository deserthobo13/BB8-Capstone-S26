import os
# --- 1. HARDWARE PWM FIX (Must be before gpiozero import) ---
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

import time
import sys
import busio
import board
import pygame

# Adafruit & GPIOZero imports
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import PhaseEnableMotor, OutputDevice

# Imports for stability
from PID import PIDController
from IMU import IMUSensor

# INITIALIZE HARDWARE
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50 

# Initialize IMU sensors and PID
imu = IMUSensor()
balance_pid = PIDController(kp=1.0, ki=0.0, kd=0.1)
target_pitch = 0.0  # Target angle for self-balancing (upright)

# INITIALIZE PYGAME FOR CONTROLLER
pygame.init()
pygame.joystick.init()

try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller detected: {joystick.get_name()}")
except pygame.error:
    print("No controller found. Please connect your DualSense.")
    sys.exit()

# DEFINE MOTORS, SERVOS & RELAYS
# Phase = Direction, Enable = PWM Speed
DC_motor1 = PhaseEnableMotor(phase=5, enable=13, pwm=True)
DC_motor2 = PhaseEnableMotor(phase=6, enable=18, pwm=True)
Turn_motor = PhaseEnableMotor(phase=26, enable=19, pwm=True)

swing_servo = servo.Servo(pca.channels[0])
head_fb = servo.Servo(pca.channels[2])
head_sts = servo.Servo(pca.channels[1])
head_rotate = servo.ContinuousServo(pca.channels[3], min_pulse=810, max_pulse=2300)
swing_servo2 = servo.Servo(pca.channels[4])

# Initialize Relays (Defaults to outputting 0V / Low)
relay1 = OutputDevice(20, initial_value=False)
relay2 = OutputDevice(21, initial_value=False)

# 5. HELPER FUNCTIONS
def set_swing(degrees):
    # Adjust the min and max limits if your offset pushes the pendulum too far
    degrees = max(70, min(117, degrees))
    
    # --- PENDULUM HARDWARE OFFSET ---
    # Change this number to physically center the pendulum. 
    # Try +5 or -5 to see which way it physically shifts the weight.
    offset = -1
    
    # Calculate how far from center we need to be, including our physical offset
    difference = (degrees - 90) + offset
    
    # Apply to both servos (one adds, one subtracts because they are mirrored)
    swing_servo.angle = 90 + difference
    swing_servo2.angle = 90 - difference

def set_head_fb(degrees):
    # Safe range 55 to 125
    degrees = max(55, min(125, degrees))
    head_fb.angle = degrees -5 

def set_head_sts(degrees):
    # Safe range 40 to 140
    degrees = max(40, min(140, degrees))
    head_sts.angle = degrees + 5

def rest_all_servos():
    for i in range(5):
        pca.channels[i].duty_cycle = 0

# 6. MENU SYSTEM
PS5_control_mode = False
test_choice = input("Select Mode:\n1: DC Motor Test\n2: Swing Test\n3: Head Test\n4: PS5 Controller Mode\nChoice: ")

if test_choice == "4": PS5_control_mode = True

# 7. MAIN EXECUTION LOOP
print("\n--- SYSTEM ACTIVE ---")
print("PS BUTTON: Emergency Stop | X BUTTON: Reset All")
print("L1/R1: Head Spin | D-PAD L/R: Steering | L-STICK Y: Drive")

# Turn on relays if entering PS5 mode (Outputs 3.3V)
if PS5_control_mode:
    relay1.on()
    relay2.on()
    print("Relays Powered ON (3.3V Active)")

try:
    while True:
        if PS5_control_mode:
            pygame.event.pump()
            
            # --- A. STABILITY CONTROL (AUTOMATED) ---
            # 1. Get IMU data
            current_pitch = imu.get_pitch()
            current_roll = imu.get_roll()
            
            # 2. Compute PID for pendulum swing
            pid_correction = balance_pid.compute(target_pitch, current_pitch)
            target_swing = 90 - pid_correction
            set_swing(target_swing)
            
            # 3. Automatic head levelling (WITH DEADZONE)
            head_deadzone = 5 # Degrees of tilt to ignore (adjust if it still jitters)
            
            # Apply the deadzone: if the tilt is too small, treat it as 0
            if abs(current_roll) < head_deadzone:
                active_roll = 0.0
            else:
                # Optional: Smooth the transition so it doesn't "snap" when crossing the deadzone
                active_roll = current_roll - head_deadzone if current_roll > 0 else current_roll + head_deadzone

            if abs(current_pitch) < head_deadzone:
                active_pitch = 0.0
            else:
                active_pitch = current_pitch - head_deadzone if current_pitch > 0 else current_pitch + head_deadzone

            # Calculate servo positions using the deadzoned values
            level_head_fb = 90 + active_roll   
            level_head_sts = 90 + active_pitch 
            
            set_head_fb(level_head_fb)
            set_head_sts(level_head_sts)

            # --- B. SYSTEM BUTTONS ---
            if joystick.get_button(10): # PS Button (Kill)
                raise KeyboardInterrupt
            
            if joystick.get_button(0):  # X Button (Reset Drive & Turn)
                DC_motor1.value = 0 # True Zero
                DC_motor2.value = 0
                Turn_motor.value = 0
                head_rotate.throttle = 0
                print("Motors Reset to Zero...")

            # --- C. LEFT STICK: DRIVE (Y-Axis Only) ---
            ly_axis = -joystick.get_axis(1) # Drive Y
            
            if abs(ly_axis) > 0.1:
                speed = ly_axis * 0.30 # Adjust this multiplier for max speed
                DC_motor1.value = speed
                DC_motor2.value = speed
            else:
                DC_motor1.value = 0
                DC_motor2.value = 0

            # --- D. D-PAD: STEERING (TURN MOTOR) ---
            hat = joystick.get_hat(0)
            if hat[0] == -1:
                Turn_motor.value = 1 # Turn Left (Swap to 1 if it steers backwards)
            elif hat[0] == 1:
                Turn_motor.value = -1  # Turn Right
            else:
                Turn_motor.value = 0  # True Zero stop

            # --- E. BUMPERS: HEAD ROTATION (SPIN) ---
            # L1 is Button 4, R1 is Button 5
            if joystick.get_button(4):
                head_rotate.throttle = -1.0 # Spin Left
            elif joystick.get_button(5):
                head_rotate.throttle = 1.0  # Spin Right
            else:
                head_rotate.throttle = -0.05     # Stop Spin

            time.sleep(0.01)

        # --- ONE TIME TESTS ---
        elif test_choice == "1":
            DC_motor1.value = 0.5; time.sleep(1); DC_motor1.value = 0
            sys.exit()
        elif test_choice == "2":
            set_swing(75); time.sleep(1); set_swing(105); time.sleep(1); set_swing(90)
            sys.exit()

except (KeyboardInterrupt, ValueError):
    print("\nShutting Down Safely...")
finally:
    # Cleanup with True Zero
    DC_motor1.value = 0
    DC_motor2.value = 0
    Turn_motor.value = 0
    head_rotate.throttle = 0
    rest_all_servos()
    
    # Power down relays (Outputs 0V)
    relay1.off()
    relay2.off()
    print("Relays Powered OFF")
    
    pygame.quit()
    sys.exit()