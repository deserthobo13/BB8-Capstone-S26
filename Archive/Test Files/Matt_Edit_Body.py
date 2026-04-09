import os
import time
import socket

# --- 1. HARDWARE PWM FIX ---
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

# --- 2. IMPORT MOTOR LIBRARIES ---
from Matt_Edit_Movement_Functions import BB8Movement

bb8 = BB8Movement()
bb8.enable_system() 

UDP_IP = "0.0.0.0" 
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) 

print(f"Motor Pi: Listening for Head Pi on port {UDP_PORT}...")

last_command_time = time.time()
TIMEOUT_SECONDS = 0.5
current_command = "STOP"
previous_command = "NONE"


def drive_with_heading(target_heading, duration, base_speed=0.5):
    print(f">>> Dead Reckoning Start: Heading {target_heading} for {duration}s")
    start_time = time.time()
    
    while time.time() < start_time + duration:
        # Get current yaw from the IMU (already initialized in bb8 object)
        current_heading = bb8.imu.get_yaw()
        
        # Calculate error (how far off 90 degrees are we?)
        error = target_heading - current_heading
        
        # Standardize error to -180 to 180
        if error > 180: error -= 360
        if error < -180: error += 360
            
        # kp: Steering aggressiveness. 
        # Start at 0.02. If it oscillates, lower it to 0.01.
        kp = 0.02 
        steer_val = error * kp
        
        # Use your new hardware function!
        bb8.drive_and_steer(base_speed, steer_val)
        
        time.sleep(0.05)
        
    bb8.stop_all()
    print(">>> Dead Reckoning Complete.")


try:
    while True:
        current_time = time.time()
        
        try:
            data, addr = sock.recvfrom(1024)
            new_command = data.decode('utf-8')
            if new_command != current_command:
                current_command = new_command
            last_command_time = current_time 
        except BlockingIOError:
            pass

        # SAFETY WATCHDOG
        if current_time - last_command_time > TIMEOUT_SECONDS:
            if current_command != "STOP":
                current_command = "STOP"
        
        # --- MOTOR EXECUTION LOGIC ---
        if current_command != previous_command:
            print(f"[{time.strftime('%H:%M:%S')}] Executing: '{current_command}'")
            
            if current_command == "FORWARD":
                bb8.drive(0.5)
            elif current_command == "BACKWARD":
                bb8.drive(-0.5)
            elif current_command == "STOP":
                bb8.stop_all()
            elif current_command == "SPIN_HEAD_LEFT":
                bb8.spin_head(-0.5)
            elif current_command == "SPIN_HEAD_RIGHT":
                bb8.spin_head(0.5)
            elif current_command == "SCAN":
                # Debug print to prove the Body Pi hears the command
                print(">>> SCAN SIGNAL RECEIVED: Attempting to spin <<<")
                # Increased throttle from 0.2 to 0.5 to overcome magnet friction
                bb8.spin_head(0.5)
            elif current_command == "MOVE_EAST":
                # This will BLOCK the loop for 4 seconds while it drives
                drive_with_heading(90.0, 4.0) 
                current_command = "STOP" # Reset after finished

            previous_command = current_command
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nShutting down Body Pi...")
finally:
    bb8.rest_all_servos() 
    bb8.stop_all() 
    sock.close()