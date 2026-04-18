import socket
import time
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector

# --- 1. CONFIGURATION ---
MOTOR_PI_IP = "bb8pi.local"  # Body Pi IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

COMMAND_MAP = {'w': "FORWARD", 's': "BACKWARD", 'x': "STOP", 'a': "SPIN_HEAD_LEFT", 'd': "SPIN_HEAD_RIGHT"}

# Global State Variables
current_command = "STOP"
robot_mode = "MANUAL"  # Starts in manual keyboard mode

# --- 2. BACKGROUND THREADS ---
def send_udp_stream():
    """Constantly sends the current_command to the Body Pi at 10Hz"""
    while True:
        try:
            sock.sendto(bytes(current_command, "utf-8"), (MOTOR_PI_IP, UDP_PORT))
        except Exception:
            pass 
        time.sleep(0.1) 

def terminal_input_thread():
    """Runs in the background waiting for your keyboard commands"""
    global current_command, robot_mode
    while True:
        cmd = input("").strip().lower()
        
        if cmd == 'search' or cmd == 'auto':
            robot_mode = "AUTO"
            print("\n>>> AUTONOMOUS MODE ACTIVATED. Eyes taking control. Type 'manual' to cancel. <<<")
        elif cmd == 'manual' or cmd == 'stop':
            robot_mode = "MANUAL"
            current_command = "STOP"
            print("\n>>> MANUAL MODE ACTIVATED. Keyboard control restored. <<<")
        elif robot_mode == "MANUAL" and cmd in COMMAND_MAP:
            current_command = COMMAND_MAP[cmd]
            print(f"Manual Command: {current_command}")

# Start Threads
threading.Thread(target=send_udp_stream, daemon=True).start()
threading.Thread(target=terminal_input_thread, daemon=True).start()

# --- 3. VISION SYSTEM MAIN LOOP ---
at_detector = Detector(families='tag36h11')
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

PIXEL_DEADZONE = 40
TARGET_DIST = 70.0 # Target distance in inches

print("--- BB-8 HYBRID CONTROL ACTIVE ---")
print("Controls: W/A/S/D/X for manual. Type 'search' for Auto-Tracking. Type 'manual' to cancel.")

# --- ADD THIS BEFORE THE WHILE LOOP ---
last_seen_time = time.time()
LOST_GRACE_PERIOD = 1.0  # 1 second buffer

try:
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        if robot_mode == "AUTO":
            # estimate_tag_pose=True is REQUIRED for distance (Z-axis) calculation
            results = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[600, 600, 320, 240], tag_size=165.1)
            my_tag = next((t for t in results if t.tag_id == 0), None)

            if my_tag:
                error_x = my_tag.center[0] - 320
                z_inch = my_tag.pose_t[2][0] / 25.4

                # Priority 1: Center the head on the tag
                if error_x > PIXEL_DEADZONE:
                    current_command = "SPIN_HEAD_RIGHT"
                elif error_x < -PIXEL_DEADZONE:
                    current_command = "SPIN_HEAD_LEFT"
                else:
                    # Priority 2: Once head is centered, move the body to the target distance
                    if z_inch > (TARGET_DIST + 5.0):
                        current_command = "FORWARD"
                    elif z_inch < (TARGET_DIST - 5.0):
                        current_command = "BACKWARD"
                    else:
                        current_command = "STOP"
                
                cv2.circle(frame, (int(my_tag.center[0]), int(my_tag.center[1])), 10, (0, 255, 0), -1)
                status_text = f"AUTO: {current_command} ({z_inch:.1f}in)"
            else:
                current_command = "SCAN"
                status_text = "AUTO: SCANNING..."
        else:
            status_text = f"MANUAL: {current_command}"

        # Display UI
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        # cv2.imshow("BB-8 Head", frame)
        
        # if cv2.waitKey(1) & 0xFF == ord('q'): 
            # break

except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    current_command = "STOP"
    time.sleep(0.2)
    picam2.stop()
    cv2.destroyAllWindows()
    sock.close()