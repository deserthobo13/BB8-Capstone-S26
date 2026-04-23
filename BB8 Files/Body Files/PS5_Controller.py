import os
# --- 1. Using pigpio for hardware PWM ---
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

import time
import sys
import pygame

# Import the unified hardware manager
from Movement_Functions import BB8Movement

# Initialize hardware via class
bb8 = BB8Movement()

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
    bb8.execute_step_response_test()
    raise KeyboardInterrupt

# 7. MAIN EXECUTION LOOP
print("\n--- SYSTEM ACTIVE ---")
print("PS BUTTON: Emergency Stop | X BUTTON: Reset All")
print("L1/R1: Head Spin | D-PAD L/R: Steering | L-STICK Y: Drive")
print("R-STICK: Head Pitch/Roll | L2/R2: Manual Swing (L/R)")

# --- DELTA TIME SETUP ---
TARGET_LOOP_TIME = 0.02 # 50Hz
last_time = time.time()

try:
    while True:
        loop_start = time.time()
        dt = loop_start - last_time
        last_time = loop_start
        
        if PS5_control_mode:
            # Safely clear the Pygame event queue
            for event in pygame.event.get():
                pass 
            
            if auto_balance:
                # --- OPTION 5: STABILITY CONTROL (AUTOMATED) ---
                bb8.update_balance()
            
            else: 
                # --- OPTION 6: FULL MANUAL CONTROL ---
                
                # 1. MANUAL PENDULUM SWING (L2 and R2 Triggers)
                # DualSense Linux Mapping: Axis 4 (L2), Axis 5 (R2)
                l2_trigger = joystick.get_axis(4) 
                r2_trigger = joystick.get_axis(5)
                
                # Convert from (-1 to 1) resting scale to a (0 to 1) active scale
                l2_amount = (l2_trigger + 1.0) / 2.0
                r2_amount = (r2_trigger + 1.0) / 2.0
                
                # Calculate target swing: 
                # Base is 90. 
                # R2 adds up to +20 (swings right toward 110). 
                # L2 subtracts up to -20 (swings left toward 70).
                # If both are pulled equally, they cancel out.
                target_swing = 90 + (r2_amount * 20.0) - (l2_amount * 20.0)
                bb8.set_swing(target_swing)
                
                # 2. MANUAL HEAD MOVEMENT (Right Stick)
                # DualSense Linux Mapping: Axis 2 (RX - Roll), Axis 3 (RY - Pitch)
                rx_axis = joystick.get_axis(2) 
                ry_axis = -joystick.get_axis(3) # Inverted so pushing up tilts head forward
                
                # Map joystick (-1.0 to 1.0) to raw target degrees
                # Using a +/- 25 degree range for head movement
                target_head_fb = 90 + (ry_axis * 25)
                target_head_sts = 90 + (rx_axis * 25)
                
                # Apply the EMA filter (Shock Absorber) to the manual inputs
                bb8.smoothed_head_fb = (bb8.alpha_head * target_head_fb) + ((1 - bb8.alpha_head) * bb8.smoothed_head_fb)
                bb8.smoothed_head_sts = (bb8.alpha_head * target_head_sts) + ((1 - bb8.alpha_head) * bb8.smoothed_head_sts)
                
                # Send the smoothed data to the servos
                bb8.set_head_fb(round(bb8.smoothed_head_fb, 1))
                bb8.set_head_sts(round(bb8.smoothed_head_sts, 1))

            # --- C. SYSTEM BUTTONS ---
            if joystick.get_button(10): # PS Button (Kill)
                raise KeyboardInterrupt
            
            if joystick.get_button(0):  # X Button (Reset Drive & Turn)
                bb8.stop_all()
                print("Motors Reset to Zero...")

            # --- D. LEFT STICK: DRIVE (Y-Axis Only) ---
            ly_axis = -joystick.get_axis(1) # Drive Y
            if abs(ly_axis) > 0.1:
                bb8.drive(ly_axis)
            else:
                bb8.drive(0)

            # --- E. D-PAD: STEERING (TURN MOTOR) ---
            hat = joystick.get_hat(0)
            if hat[0] == -1:
                bb8.steer(1.0) # Turn Left (Using float value now)
            elif hat[0] == 1:
                bb8.steer(-1.0)  # Turn Right
            else:
                bb8.steer(0.0)

            # --- F. BUMPERS: HEAD ROTATION (SPIN) ---
            if joystick.get_button(4): # L1
                bb8.spin_head(-1.0) # Spin Left
            elif joystick.get_button(5): # R1
                bb8.spin_head(1.0)  # Spin Right
            else:
                bb8.spin_head(-0.05) # Stop Spin (Slight offset to counter drift if needed)

            # --- G. DYNAMIC LOOP TIMING ---
            elapsed = time.time() - loop_start
            if elapsed < TARGET_LOOP_TIME:
                time.sleep(TARGET_LOOP_TIME - elapsed)

        # --- ONE TIME TESTS ---
        elif test_choice == "1":
            print("Testing Drive Motor...")
            bb8.drive(0.5); time.sleep(1); bb8.drive(0)
            time.sleep(0.5)
            print("Testing Steer Motor...")
            bb8.steer(1.0); time.sleep(1); bb8.steer(0) 
            print("Tests Complete.")
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