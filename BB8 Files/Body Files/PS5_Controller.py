import os
# --- 1. Using pigpio for hardware PWM ---
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

import time
import sys
import pygame

# Import the unified hardware manager
from Movement_Functions import BB8Movement

# Import the dead reckoning module
# from Dead_Reckoning import DeadReckoning

# Initialize hardware via class
bb8 = BB8Movement()
# dr = DeadReckoning()

# Initialize pygame for the controller
pygame.init()
pygame.joystick.init()

try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller detected: {joystick.get_name()}")
except pygame.error:
    print("No controller found. Please connect your DualSense.")
    sys.exit()

# 6. MENU SYSTEM
PS5_control_mode = False
auto_balance = False
test_choice = input("Select Mode:\n1: DC Motor Test\n2: Swing Test\n3: Head Test\n4: Step Response Test\n5: PS5 Controller Mode Automated Stability\n6: PS5 Controller Mode\nChoice: ")

if test_choice == "6": 
    auto_balance = False
    PS5_control_mode = True
    bb8.enable_system()

elif test_choice == "5": 
    auto_balance = True
    PS5_control_mode = True
    bb8.enable_system()
    
elif test_choice == "4":
    print("\n[INITIATING AUTOMATED STEP RESPONSE TEST]")
    # 1. Call the self-contained function you added to Movement_Functions.py
    bb8.execute_step_response_test()
    
    # 2. Raise a KeyboardInterrupt to intentionally break the script.
    # This acts as a safe "Go to Finally" command, ensuring your script
    # cleanly shuts down the motors, kills the relays, and exits Pygame.
    raise KeyboardInterrupt

# 7. MAIN EXECUTION LOOP
print("\n--- SYSTEM ACTIVE ---")
print("PS BUTTON: Emergency Stop | X BUTTON: Reset All")
print("L1/R1: Head Spin | D-PAD L/R: Steering | L-STICK Y: Drive")

# --- DELTA TIME SETUP ---
TARGET_LOOP_TIME = 0.02 # 50Hz

# --- INITIALIZE PREVIOUS TIME ---
last_time = time.time()

try:
    while True:
        loop_start = time.time()
        
        # --- CALCULATE DT (Time since last loop) ---
        dt = loop_start - last_time
        last_time = loop_start
        
        if PS5_control_mode:
            pygame.event.pump()
            if auto_balance:
                # --- A. STABILITY CONTROL (AUTOMATED) ---
                bb8.update_balance()
            
                # --- B. STABILITY CONTROL (AUTOMATED) ---
                z_lin_accel = bb8.imu.accel_z
                # velocity, distance = dr.update(z_lin_accel, dt)
            else: 
                # MANUAL SWING OVERRIDE (Right Stick Y-Axis)
                # Note: Pygame usually maps Right Stick Y to axis 3 or 4 depending on the OS.
                # If axis 3 acts weird (like a trigger), change it to axis 4.
                ry_axis = -joystick.get_axis(3) 
                
                # Map joystick (-1.0 to 1.0) to safe servo degrees (70 to 110)
                # 90 is perfectly centered. Max tilt is +/- 20 degrees.
                target_swing = 90 + (ry_axis * 20) 
                bb8.set_swing(target_swing)
                
                # Note: We skip bb8.update_balance(), so the head servos won't auto-level
                # based on IMU roll/pitch in this mode, which is expected for manual testing!
                
            # --- C. SYSTEM BUTTONS ---
            if joystick.get_button(10): # PS Button (Kill)
                raise KeyboardInterrupt
            
            if joystick.get_button(0):  # X Button (Reset Drive & Turn)
                bb8.stop_all()
                print("Motors Reset to Zero...")

            # --- D. LEFT STICK: DRIVE (Y-Axis Only) ---
            ly_axis = -joystick.get_axis(1) # Drive Y
            if abs(ly_axis) > 0.1:
                bb8.drive(ly_axis) # BB8Movement handles the 0.30 multiplier now
            else:
                bb8.drive(0)

            # --- E. D-PAD: STEERING (TURN MOTOR) ---
            hat = joystick.get_hat(0)
            if hat[0] == -1:
                bb8.steer(1) # Turn Left 
            elif hat[0] == 1:
                bb8.steer(-1)  # Turn Right
            else:
                bb8.steer(0)  # True Zero stop

            # --- F. BUMPERS: HEAD ROTATION (SPIN) ---
            if joystick.get_button(4):
                bb8.spin_head(-1.0) # Spin Left
            elif joystick.get_button(5):
                bb8.spin_head(1.0)  # Spin Right
            else:
                bb8.spin_head(-0.05) # Stop Spin

            # --- G. DYNAMIC LOOP TIMING ---
            elapsed = time.time() - loop_start
            if elapsed < TARGET_LOOP_TIME:
                time.sleep(TARGET_LOOP_TIME - elapsed)

        # --- ONE TIME TESTS ---
        elif test_choice == "1":
            bb8.drive(0.5); time.sleep(1); bb8.drive(0)
            sys.exit()
        elif test_choice == "2":
            bb8.set_swing(75); time.sleep(1); bb8.set_swing(105); time.sleep(1); bb8.set_swing(90)
            sys.exit()

except (KeyboardInterrupt, ValueError):
    print("\nShutting Down Safely...")
finally:
    bb8.stop_all()
    bb8.rest_all_servos()
    bb8.disable_system() # Ensure relays are off
    print("Relays Powered OFF")
    
    pygame.quit()
    sys.exit()