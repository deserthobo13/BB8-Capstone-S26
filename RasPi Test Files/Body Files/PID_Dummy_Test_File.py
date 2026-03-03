import time
from IMU import IMUSensor
from PID import PIDController

def run_pendulum_pid_test():
    print("--- Starting Pendulum PID Test ---")
    imu = IMUSensor()
    
    # Kp=1.0 means 1 degree of fall = 1 degree of servo swing. 
    # You will likely need to increase this later!
    pid = PIDController(kp=1.0, ki=0.0, kd=0.1) 
    target_pitch = 0.0 # Balance upright
    
    print("\nStarting loop in 3 seconds...")
    time.sleep(3)
    pid.reset()
    
    try:
        while True:
            current_pitch = imu.get_pitch()
            
            # 1. Calculate the correction
            pid_correction = pid.compute(target_pitch, current_pitch)
            
            # 2. SUBTRACT correction based on your hardware test!
            servo_degrees = 90 - pid_correction
            
            # 3. Clamp the values to your safe physical limits
            clamped_degrees = max(70, min(117, servo_degrees))
            
            print(f"Pitch: {current_pitch:>6.2f}° | PID: {pid_correction:>6.2f} | Command to set_swing: {clamped_degrees:>6.2f}°")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nTest stopped.")

if __name__ == "__main__":
    run_pendulum_pid_test()