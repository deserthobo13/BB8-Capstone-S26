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
print("L-STICK Y: Drive (Fixed 40%) | D-PAD L/R: Pirouette Steer")
print("L1/R1: Head Spin | R-STICK: Head Pitch & Roll | L2/R2: Pendulum Swing")

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
            
            # --- SYSTEM BUTTONS (Always active) ---
            if joystick.get_button(10): # PS Button (Kill)
                raise KeyboardInterrupt
            
            if joystick.get_button(0):  # X Button (Reset Drive & Turn)
                bb8.stop_all()
                print("Motors Reset to Zero...")

            # ==========================================
            # CONDITIONAL CONTROLS (Pendulum & Head Tilt)
            # ==========================================
            if auto_balance:
                # --- OPTION 5: STABILITY CONTROL ---
                # Completely relies on IMU and update_balance(), leaves joystick alone
                bb8.update_balance()
            
            else: 
                # --- OPTION 6: MANUAL STABILITY CONTROL ---
                
                # 1. MANUAL PENDULUM SWING (L2 & R2 Triggers)
                # Left Trigger = Axis 2 | Right Trigger = Axis 5
                l2_raw = joystick.get_axis(2) 
                r2_raw = joystick.get_axis(5)
                
                # Zero-state bug fix for Pygame triggers
                if l2_raw == 0.0: l2_raw = -1.0
                if r2_raw == 0.0: r2_raw = -1.0
                
                l2_amount = (l2_raw + 1.0) / 2.0
                r2_amount = (r2_raw + 1.0) / 2.0
                
                # Math Swapped: L2 is now (+), R2 is now (-)
                target_pendulum = 90 + (l2_amount * 20.0) - (r2_amount * 20.0)
                bb8.set_swing(target_pendulum) 
                
                # 2. MANUAL HEAD MOVEMENT (Right Stick)
                # Right Stick X = Axis 3 (STS) | Right Stick Y = Axis 4 (FB)
                # BOTH AXES NEGATED AS REQUESTED
                stick_sts = -joystick.get_axis(3) 
                stick_fb = joystick.get_axis(4) 
                
                target_from_stick_sts = 90 + (stick_sts * 25)
                target_from_stick_fb = 90 + (stick_fb * 25)
                
                # Wired as requested: Stick STS -> head_fb | Stick FB -> head_sts
                bb8.smoothed_head_fb = (bb8.alpha_head * target_from_stick_sts) + ((1 - bb8.alpha_head) * bb8.smoothed_head_fb)
                bb8.smoothed_head_sts = (bb8.alpha_head * target_from_stick_fb) + ((1 - bb8.alpha_head) * bb8.smoothed_head_sts)
                
                bb8.set_head_fb(round(bb8.smoothed_head_fb, 1))
                bb8.set_head_sts(round(bb8.smoothed_head_sts, 1))

            # ==========================================
            # UNIVERSAL CONTROLS (Active in Mode 5 and 6)
            # ==========================================
            
            # 3. MANUAL DRIVE (Left Stick Y-Axis)
            # Left Stick Y = Axis 1
            ly_axis = -joystick.get_axis(1) 
            
            if ly_axis > 0.15: 
                bb8.drive(0.3)  # Hardcoded 40% forward
            elif ly_axis < -0.15:
                bb8.drive(-0.3) # Hardcoded 40% backward
            else:
                bb8.drive(0.0)  # Hardcoded 0% stop

            # 4. PIROUETTE STEERING (Turn Motor on D-Pad)
            hat = joystick.get_hat(0)
            if hat[0] == -1:
                bb8.steer(-1.0) # FLIPPED! 
            elif hat[0] == 1:
                bb8.steer(1.0)  # FLIPPED! 
            else:
                bb8.steer(0.0)

            # 5. HEAD ROTATION (SPIN on Bumpers)
            if joystick.get_button(4): # L1
                bb8.spin_head(-1.0) # Spin Left
            elif joystick.get_button(5): # R1
                bb8.spin_head(1.0)  # Spin Right
            else:
                bb8.spin_head(-0.05) # Stop Spin

            # --- DYNAMIC LOOP TIMING ---
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

except KeyboardInterrupt:
    print("\nShutting Down Safely (User Initiated)...")
except ValueError as e:
    print(f"\n[SENSOR ERROR] ValueError caught: {e}")
    print("This is likely bad I2C data from the IMU. Check wiring or retry.")
except Exception as e:
    print(f"\n[CRITICAL ERROR] {e}")
finally:
    bb8.stop_all()
    bb8.rest_all_servos()
    bb8.disable_system() 
    print("Relays Powered OFF")
    
    pygame.quit()
    sys.exit()