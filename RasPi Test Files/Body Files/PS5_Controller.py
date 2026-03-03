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

# 2. INITIALIZE HARDWARE
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50 

# 3. INITIALIZE PYGAME FOR CONTROLLER
pygame.init()
pygame.joystick.init()

try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller detected: {joystick.get_name()}")
except pygame.error:
    print("No controller found. Please connect your DualSense.")
    sys.exit()

# 4. DEFINE MOTORS & SERVOS (Sign-Magnitude Mode)
# Phase = Direction, Enable = PWM Speed
DC_motor1 = PhaseEnableMotor(phase=5, enable=13, pwm=True)
DC_motor2 = PhaseEnableMotor(phase=6, enable=18, pwm=True)
Turn_motor = PhaseEnableMotor(phase=26, enable=19, pwm=True)

swing_servo = servo.Servo(pca.channels[0])
head_fb = servo.Servo(pca.channels[2])
head_sts = servo.Servo(pca.channels[1])
head_rotate = servo.ContinuousServo(pca.channels[3], min_pulse=810, max_pulse=2300)
swing_servo2 = servo.Servo(pca.channels[4])

# --- STATE VARIABLES ---
current_head_fb = 90
current_head_sts = 90
current_swing = 90

# 5. HELPER FUNCTIONS
def set_swing(degrees):
    # Safe range 70 to 117
    degrees = max(70, min(117, degrees))
    difference = degrees - 90
    swing_servo.angle = 90 + difference
    swing_servo2.angle = 90 - difference

def set_head_fb(degrees):
    # Safe range 55 to 125
    degrees = max(55, min(125, degrees))
    head_fb.angle = degrees + 15

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
print("L1/R1: Head Spin | D-PAD L/R: Steering | L-STICK: Drive/Swing | R-STICK: Head Tilt")

try:
    while True:
        if PS5_control_mode:
            pygame.event.pump()

            # --- A. SYSTEM BUTTONS ---
            if joystick.get_button(10): # PS Button (Kill)
                raise KeyboardInterrupt
            
            if joystick.get_button(0):  # X Button (Reset)
                DC_motor1.value = 0 # True Zero
                DC_motor2.value = 0
                Turn_motor.value = 0
                head_rotate.throttle = 0
                current_head_fb = 90
                current_head_sts = 90
                current_swing = 90
                set_swing(90)
                set_head_fb(90)
                set_head_sts(90)
                print("Resetting Positions...")

            # --- B. LEFT STICK: DRIVE (Y) & SWING (X) ---
            ly_axis = -joystick.get_axis(1) # Drive Y
            lx_axis = joystick.get_axis(0)  # Swing X
            
            # Drive Logic (Natively handles -1.0 to 1.0 in PhaseEnable mode)
            if abs(ly_axis) > 0.1:
                speed = ly_axis * 0.30 # Adjust this multiplier for max speed
                DC_motor1.value = speed
                DC_motor2.value = speed
            else:
                DC_motor1.value = 0
                DC_motor2.value = 0

            # Swing Logic with Auto-Center
            if abs(lx_axis) > 0.1:
                current_swing += (lx_axis * 2.5)
            else:
                if current_swing > 91:
                    current_swing -= 1.5
                elif current_swing < 89:
                    current_swing += 1.5
                else:
                    current_swing = 90
            set_swing(current_swing)

            # --- C. RIGHT STICK: SLOW HEAD TILT ---
            rx_axis = joystick.get_axis(3) # Side-to-Side
            ry_axis = joystick.get_axis(4) # Forward-Backward
            
            # Smooth Head Side-to-Side
            if abs(rx_axis) > 0.1:
                current_head_sts -= (rx_axis * 1.0) # Speed set to 1.0 (Cinematic)
                current_head_sts = max(40, min(140, current_head_sts))
                set_head_sts(current_head_sts)

            # Smooth Head Forward-Backward
            if abs(ry_axis) > 0.1:
                current_head_fb += (ry_axis * 1.0) 
                current_head_fb = max(55, min(125, current_head_fb))
                set_head_fb(current_head_fb)

            # --- D. D-PAD: STEERING (TURN MOTOR) ---
            hat = joystick.get_hat(0)
            if hat[0] == -1:
                Turn_motor.value = -1 # Turn Left (Swap to 1 if it steers backwards)
            elif hat[0] == 1:
                Turn_motor.value = 1  # Turn Right
            else:
                Turn_motor.value = 0  # True Zero stop

            # --- E. BUMPERS: HEAD ROTATION (SPIN) ---
            # L1 is Button 4, R1 is Button 5
            if joystick.get_button(4):
                head_rotate.throttle = -1.0 # Spin Left
            elif joystick.get_button(5):
                head_rotate.throttle = 1.0  # Spin Right
            else:
                head_rotate.throttle = 0     # Stop Spin

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
    pygame.quit()
    sys.exit()