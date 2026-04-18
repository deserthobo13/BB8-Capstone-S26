import os
import time

# --- 1. HARDWARE PWM FIX ---
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

from Movement_Functions import BB8Movement

print("Initializing Calibration Mode...")
bb8 = BB8Movement()
bb8.enable_system()

# Start at perfectly centered
current_angle = 90
bb8.set_swing(current_angle)
print("\n--- PENDULUM CALIBRATION TOOL ---")
print("WARNING: KEEP EYES ON THE HARDWARE. BE READY TO KILL POWER.")
print("Type a number to move the servo. Type 'q' to quit.")

try:
    while True:
        user_input = input(f"\nCurrent Angle is {current_angle}. Enter new angle: ")
        
        if user_input.lower() == 'q':
            break
            
        try:
            target_angle = int(user_input)
            
            # Remove the safe clamps for this test so we can actually push it!
            # We are directly commanding the servo hardware here.
            difference = target_angle - 90 + bb8.swing_offset
            bb8.swing_servo.angle = 90 + difference
            bb8.swing_servo2.angle = 90 - difference
            
            current_angle = target_angle
            print(f">>> Servos moved to {current_angle}°")
            
        except ValueError:
            print("Please enter a valid whole number.")
            
except KeyboardInterrupt:
    pass
finally:
    print("\nShutting down safely...")
    bb8.set_swing(90)
    time.sleep(0.5)
    bb8.stop_all()
    bb8.rest_all_servos()
    bb8.disable_system()