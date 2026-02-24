import socket
import time
import os
from Movement_Functions import BB8Movement

# Initialize the BB-8 Movement class & Functions
bb8 = BB8Movement()
bb8.enable_system() # Power on the motors and relays

# Set up the UDP socket
UDP_IP = "0.0.0.0" # Correctly listens on all interfaces (Using 0.0.0.0 is safer)
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) # Prevents recvfrom from pausing the code

print("Motor Pi: Ready for commands...")

current_command = "STOP"

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            current_command = data.decode('utf-8')
            print(f"Received new command: {current_command}")
        except BlockingIOError:
            # No data received this loop, which is fine
            pass

        # --- SAFETY TIMEOUT REMOVED ---
        # The robot will now hold the last command forever until a new one arrives.
    
        # Motor execution logic (This now runs continuously without freezing)
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
    
        time.sleep(0.01) # Small sleep to prevent maxing out the CPU (100Hz loop)

except KeyboardInterrupt:
    print("\nCtrl+C detected. Shutting down Body Pi...")
finally:
    bb8.rest_all_servos() # Return all servos to neutral position
    bb8.stop_all() # Ensure we turn off motors and relays on exit
    time.sleep(0.5) # Sleep to ensure the GPIO pins revert to 0
    sock.close() # Clean up the socket connection
    os.system("pinctrl set 13,18,19 op dl")