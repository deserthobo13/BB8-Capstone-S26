import os
import time

# --- 1. HARDWARE PWM FIX ---
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

# --- 2. IMPORT MOTOR LIBRARIES ---
import socket
from Movement_Functions import BB8Movement

# Initialize the BB-8 Movement class & Functions
bb8 = BB8Movement()
bb8.enable_system() 

# Set up the UDP socket
UDP_IP = "0.0.0.0" 
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) 

print(f"Motor Pi: Listening for Head Pi on port {UDP_PORT}...")

last_command_time = time.time()
TIMEOUT_SECONDS = 0.5

current_command = "STOP"
previous_command = "NONE" # Track the last executed command

# Variables for calculating instruction rate
packet_count = 0
rate_timer = time.time()

try:
    while True:
        current_time = time.time()
        
        try:
            data, addr = sock.recvfrom(1024)
            new_command = data.decode('utf-8')
            
            packet_count += 1
            
            # Network update
            if new_command != current_command:
                current_command = new_command
            
            last_command_time = current_time # Pet the watchdog timer
            
        except BlockingIOError:
            pass

        # --- Rate Printing Logic ---
        if current_time - rate_timer >= 1.0:
            if current_command != "STOP" or packet_count > 0:
                print(f"--- Data Rate: {packet_count} packets/sec ---")
            packet_count = 0
            rate_timer = current_time

        # --- SAFETY WATCHDOG TIMEOUT ---
        if current_time - last_command_time > TIMEOUT_SECONDS:
            if current_command != "STOP":
                print("--- CONNECTION LOST! EXECUTING EMERGENCY STOP ---")
                current_command = "STOP"
        
        # --- MOTOR EXECUTION LOGIC (STATE MACHINE) ---
        # Only update the hardware if the command has ACTUALLY changed
        if current_command != previous_command:
            print(f"[{time.strftime('%H:%M:%S')}] Executing Hardware State: '{current_command}'")
            
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
            
            # Update the tracker so it doesn't execute again until necessary
            previous_command = current_command
        
        time.sleep(0.01) # 100Hz loop frequency

except KeyboardInterrupt:
    print("\nCtrl+C detected. Shutting down Body Pi...")
finally:
    print("Cleaning up hardware states...")
    bb8.rest_all_servos() 
    bb8.stop_all() 
    time.sleep(0.5) 
    sock.close() 
    print("Shutdown complete.")