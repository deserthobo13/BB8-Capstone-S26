import socket
import time
import threading
import cv2
from picamera2 import Picamera2
from pupil_apriltags import Detector

# --- 1. CONFIGURATION ---
MOTOR_PI_IP = "bb8pi.local"  # Update this if your Motor Pi IP changes
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# State Variables
current_command = "STOP"
robot_mode = "MANUAL"  
TAG_A = 0  
TAG_B = 1  
current_target = TAG_A
TARGET_DIST = 50.0    # Distance in inches to stop from tag
PIXEL_DEADZONE = 45   # Center screen sensitivity
pause_until = 0       # Timer for the dramatic pause

# --- 2. BACKGROUND THREADS ---
def send_udp_stream():
    """Heartbeat thread: Sends the current command to Body Pi at 10Hz"""
    while True:
        try:
            sock.sendto(bytes(current_command, "utf-8"), (MOTOR_PI_IP, UDP_PORT))
        except Exception:
            pass 
        time.sleep(0.1) 

def terminal_input_thread():
    """UI thread: Handles keyboard switching between manual and demo mode"""
    global current_command, robot_mode, current_target
    while True:
        cmd = input("").strip().lower()
        if cmd == 'demo':
            robot_mode = "PING_PONG"
            current_target = TAG_A
            print("\n>>> PING-PONG ACTIVATED: Forward to Tag 0, Backward to Tag 1 <<<")
        elif cmd in ['manual', 'stop', 'x']:
            robot_mode = "MANUAL"
            current_command = "STOP"
            print("\n>>> MANUAL MODE: Automation paused. <<<")

# Start background processes
threading.Thread(target=send_udp_stream, daemon=True).start()
threading.Thread(target=terminal_input_thread, daemon=True).start()

# --- 3. VISION SYSTEM SETUP ---
at_detector = Detector(families='tag36h11')
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("--- BB-8 PING-PONG DEMO READY ---")
print("Type 'demo' to start. Type 'manual' to stop.")

try:
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        # 1. HANDLE DRAMATIC PAUSE BETWEEN SWAPS
        if time.time() < pause_until:
            current_command = "STOP"
            continue

        # 2. DEMO LOGIC
        if robot_mode == "PING_PONG":
            results = at_detector.detect(gray, estimate_tag_pose=True, 
                                         camera_params=[600, 600, 320, 240], 
                                         tag_size=165.1) # 165.1mm = 6.5 inch tag
            
            # Filter for the specific tag we want
            my_tag = next((t for t in results if t.tag_id == current_target), None)

            if my_tag:
                error_x = my_tag.center[0] - 320
                z_inch = my_tag.pose_t[2][0] / 25.4

                # STEP A: CENTER THE HEAD
                if error_x > PIXEL_DEADZONE:
                    current_command = "SPIN_HEAD_RIGHT"
                elif error_x < -PIXEL_DEADZONE:
                    current_command = "SPIN_HEAD_LEFT"
                
                # STEP B: DRIVE (Forward for Tag 0, Backward for Tag 1)
                else:
                    if z_inch > TARGET_DIST:
                        # Logic: Tag 0 -> Forward | Tag 1 -> Backward
                        current_command = "FORWARD" if current_target == TAG_A else "BACKWARD"
                    else:
                        # STEP C: ARRIVAL & SWAP
                        current_command = "STOP"
                        print(f"Reached Tag {current_target}! Swapping...")
                        current_target = TAG_B if current_target == TAG_A else TAG_A
                        pause_until = time.time() + 2.0 
            
            else:
                # SEARCHING: If target tag is not in view, spin head infinitely
                current_command = "SPIN_HEAD_RIGHT"
                
        # 3. MANUAL COMMANDS ARE HANDLED BY THE INPUT THREAD
        
except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    current_command = "STOP"
    time.sleep(0.2)
    picam2.stop()
    sock.close()