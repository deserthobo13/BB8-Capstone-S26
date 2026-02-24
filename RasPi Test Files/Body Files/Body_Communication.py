import socket
import time
import os
from Movement_Functions import BB8Movement

# Initialize the BB-8 Movement class & Functions
bb8 = BB8Movement()
bb8.enable_system() # Power on the motors and relays

# Set up the UDP socket
UDP_IP = "" # Correctly listens on all interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) # Prevents recvfrom from pausing the code

print("Motor Pi: Ready for commands...")

last_command_time = time.time()
TIMEOUT_SECONDS = 0.5
current_command = "STOP"

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            current_command = data.decode('utf-8')
            last_command_time = time.time() # Reset the safety timer
        except BlockingIOError:
            # No data received this loop, which is fine
            pass

    # Safety Timeout: If no commands received recently, stop moving
        if time.time() - last_command_time > TIMEOUT_SECONDS:
            if current_command != "STOP":
                print("Connection timeout! Stopping motors for safety.")
                current_command = "STOP"
    
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
    time.sleep(0.5) #Sleep to ensure the GPIO pins revert to 0
    sock.close() # Clean up the socket connection
    os.system("pinctrl set 13,18,19 op dl")
