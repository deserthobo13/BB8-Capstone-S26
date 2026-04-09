import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

import time
import sys
import pygame

# Import the new Test Hardware Manager
from Test_Movement_Functions import TestBB8Movement
from Dead_Reckoning import DeadReckoning

bb8 = TestBB8Movement()
dr = DeadReckoning()

pygame.init()
pygame.joystick.init()

try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller detected: {joystick.get_name()}")
except pygame.error:
    print("No controller found. Please connect your DualSense.")
    sys.exit()

PS5_control_mode = False
test_choice = input("Select Mode:\n1: DC Motor Test\n2: Swing Test\n3: Head Test\n4: PS5 Controller Mode\nChoice: ")

if test_choice == "4": 
    PS5_control_mode = True
    bb8.enable_system()

print("\n--- TEST SYSTEM ACTIVE ---")
print("MANUAL HEAD MODE: Right Stick controls head pitch and roll.")

TARGET_LOOP_TIME = 0.02 # 50Hz
last_time = time.time()

try:
    while True:
        loop_start = time.time()
        dt = loop_start - last_time
        last_time = loop_start
        
        if PS5_control_mode:
            pygame.event.pump()
            
            # --- A. STABILITY CONTROL (AUTOMATED SWING ONLY) ---
            bb8.update_balance()
            z_lin_accel = bb8.imu.accel_z
            velocity, distance = dr.update(z_lin_accel, dt)
            
            # --- B. SYSTEM BUTTONS ---
            if joystick.get_button(10): raise KeyboardInterrupt
            if joystick.get_button(0):  
                bb8.stop_all()
                print("Motors Reset to Zero...")

            # --- C. LEFT STICK: DRIVE ---
            ly_axis = -joystick.get_axis(1) 
            if abs(ly_axis) > 0.1:
                bb8.drive(ly_axis) 
            else:
                bb8.drive(0)

            # --- D. D-PAD: STEERING ---
            hat = joystick.get_hat(0)
            if hat[0] == -1: bb8.steer(1) 
            elif hat[0] == 1: bb8.steer(-1)  
            else: bb8.steer(0)  

            # --- E. BUMPERS: HEAD SPIN ---
            if joystick.get_button(4): bb8.spin_head(-1.0) 
            elif joystick.get_button(5): bb8.spin_head(1.0)  
            else: bb8.spin_head(-0.05) 

            # --- F. RIGHT STICK: MANUAL HEAD PUPPETEERING ---
            # Map joystick (-1.0 to 1.0) to your safe servo ranges
            rx_axis = joystick.get_axis(2) # Side-to-Side (Roll)
            ry_axis = -joystick.get_axis(3) # Forward/Backward (Pitch) - Inverted for intuitive flight-stick control

            # If joystick is pushed full right (+1.0), it adds 50 degrees to the 90 center (140 max)
            target_head_sts = 90 + (rx_axis * 50)
            
            # If joystick is pushed full up (+1.0), it adds 35 degrees to the 90 center (125 max)
            target_head_fb = 90 + (ry_axis * 35) 
            
            # Apply EMA filter for organic movement
            bb8.smoothed_head_fb = (bb8.alpha_head * target_head_fb) + ((1 - bb8.alpha_head) * bb8.smoothed_head_fb)
            bb8.smoothed_head_sts = (bb8.alpha_head * target_head_sts) + ((1 - bb8.alpha_head) * bb8.smoothed_head_sts)
            
            # Send to hardware
            bb8.set_head_fb(round(bb8.smoothed_head_fb, 1))
            bb8.set_head_sts(round(bb8.smoothed_head_sts, 1))

            # --- G. DYNAMIC LOOP TIMING ---
            elapsed = time.time() - loop_start
            if elapsed < TARGET_LOOP_TIME:
                time.sleep(TARGET_LOOP_TIME - elapsed)

        # --- ONE TIME TESTS ---
        elif test_choice == "1":
            bb8.drive(0.5); time.sleep(1); bb8.drive(0); sys.exit()
        elif test_choice == "2":
            bb8.set_swing(75); time.sleep(1); bb8.set_swing(105); time.sleep(1); bb8.set_swing(90); sys.exit()
        elif test_choice == "3":
            bb8.set_head_fb(125); time.sleep(1); bb8.set_head_fb(55); time.sleep(1); bb8.set_head_fb(90); sys.exit()

except (KeyboardInterrupt, ValueError):
    print("\nShutting Down Safely...")
finally:
    bb8.stop_all()
    bb8.rest_all_servos()
    bb8.disable_system() 
    print("Relays Powered OFF")
    pygame.quit()
    sys.exit()