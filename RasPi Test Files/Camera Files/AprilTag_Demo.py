import cv2
import numpy as np
import time
import socket
from picamera2 import Picamera2
from pupil_apriltags import Detector

# --- UDP SETUP ---
MOTOR_PI_IP = "bb8pi.local" # Change to Body IP if .local doesn't resolve
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(cmd):
    """Sends the command string to the Body Pi via UDP."""
    sock.sendto(bytes(cmd, "utf-8"), (MOTOR_PI_IP, UDP_PORT))
    print(f"Network Update -> Sent: {cmd}")

# --- 1. SETTINGS (TUNE THESE) ---
# THE MASTER DISTANCE CONTROL (in inches)
TARGET_DISTANCE = 70.0 

# Distance Zones
START_MOVE_MIN = TARGET_DISTANCE - 5.0  
START_MOVE_MAX = TARGET_DISTANCE + 5.0  
STOP_FORWARD = TARGET_DISTANCE    
STOP_BACKWARD = TARGET_DISTANCE + 3.0   

# Timing
MISSING_FRAME_LIMIT = 8 
LOST_TIMEOUT = 3.0      

# --- 2. INITIALIZATION ---
last_seen_time = 0
is_tracking = False 
is_moving = False  
move_dir = None 
missing_frame_count = 0
last_sent_command = "STOP" # Prevents spamming the network

at_detector = Detector(families='tag36h11')
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print(f"--- BB-8: HEADLESS VISION TO UDP BRIDGE ACTIVE ---")
print(f"--- TARGET DISTANCE: {TARGET_DISTANCE} inches ---")
print("Press Ctrl+C in the terminal to stop.")

try:
    while True:
        # Capture the array and convert straight to Grayscale for efficiency
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        results = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[600, 600, 320, 240], tag_size=165.1)
        my_tag = next((t for t in results if t.tag_id == 0), None)
        
        desired_command = "STOP" # Default state is stop

        if my_tag:
            # --- LOCK ACQUIRED ---
            missing_frame_count = 0 
            is_tracking = True
            last_seen_time = time.time()
            
            z_inch = (my_tag.pose_t[2][0]) / 25.4

            if not is_moving:
                # Check if we need to wake up and move
                if z_inch < START_MOVE_MIN:
                    is_moving = True
                    move_dir = 'backward'
                elif z_inch > START_MOVE_MAX:
                    is_moving = True
                    move_dir = 'forward'
            
            if is_moving:
                # Set the desired command based on direction
                desired_command = "BACKWARD" if move_dir == 'backward' else "FORWARD"

                # Check if we reached our destination boundaries
                if (move_dir == 'backward' and z_inch >= STOP_BACKWARD) or \
                   (move_dir == 'forward' and z_inch <= STOP_FORWARD):
                    desired_command = "STOP"
                    is_moving = False
            else:
                desired_command = "STOP"

        else:
            # --- TAG MISSING ---
            missing_frame_count += 1
            time_since_last_seen = time.time() - last_seen_time
            
            if missing_frame_count < MISSING_FRAME_LIMIT and is_tracking:
                # Keep sending whatever we were doing during brief flickers
                desired_command = last_sent_command 
            elif time_since_last_seen < LOST_TIMEOUT:
                is_moving = False
                desired_command = "STOP"
            else:
                is_tracking = False
                is_moving = False
                desired_command = "STOP"

        # --- SMART UDP SENDER ---
        # Only send a packet if the command state has actually changed.
        if desired_command != last_sent_command:
            send_command(desired_command)
            last_sent_command = desired_command

except KeyboardInterrupt:
    print("\nCtrl+C detected. Exiting script...")

finally:
    print("Shutting down Vision node...")
    send_command("STOP") # Send a final safety stop to the body
    picam2.stop()
    sock.close()
    print("Head Pi safely shut down.")