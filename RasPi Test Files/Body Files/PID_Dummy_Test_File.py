import time
import sys
import os
from Movement_Functions import BB8Movement

def run_balance_test():
    print("--- Starting BB-8 Real Hardware Balance Test ---")
    print("WARNING: Make sure BB-8 is safely secured on the motion stand!")
    print("The pendulum may swing rapidly during tuning.\n")
    
    # 1. Initialize the movement class (This also initializes IMU, PID, I2C, etc.)
    bb8 = BB8Movement()
    
    print("\nInitialization complete. Starting balance loop in 3 seconds...")
    time.sleep(3)
    
    # 2. Reset the PID right before starting to clear any startup time delays
    bb8.balance_pid.reset()
    
    print("Balance loop running! Press Ctrl+C to stop.")
    
    try:
        while True:
            # Continuously update the PID math and command the servos
            bb8.update_balance()
            
            # (Optional) Uncomment the lines below to see live data in your terminal.
            # Warning: Printing to the terminal slows down the loop slightly!
            # current_pitch = bb8.imu.get_pitch()
            # current_swing = bb8.current_swing
            # print(f"Pitch: {current_pitch:>6.2f}° | Swing Servo: {current_swing:>6.2f}°", end='\r')
            
            # Run the loop rapidly. 0.02 seconds = 50Hz update rate. 
            # You want this to run fast so the D (Derivative) term works smoothly.
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\nTest manually stopped. Shutting down and centering...")
        
        # 1. Command everything to zero
        bb8.stop_all()
        bb8.rest_all_servos() 
        
        # 2. Give pigpiod time to actually receive the zero command
        time.sleep(0.1)
        
        print("BB-8 safely stopped.")
        
        # 3. HARD EXIT: Kills Python instantly before gpiozero can release the pins
        os._exit(0)

if __name__ == "__main__":
    run_balance_test()