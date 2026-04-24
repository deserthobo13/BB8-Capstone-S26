import socket
import time
import threading
import cv2
from picamera2 import Picamera2
from pupil_apriltags import Detector

# --- 1. CONFIGURATION ---
# Updated to hard-coded Body Pi IP
MOTOR_PI_IP = "10.227.99.163"  
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# State Variables
current_command = "STOP"
robot_mode = "MANUAL"  

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
    global current_command, robot_mode
    while True:
        cmd = input("").strip().lower()
        if cmd == 'demo':
            robot_mode = "REACTIVE"
            # UPDATED: Added Tag 3 to the print menu
            print("\n>>> REACTIVE MODE ACTIVATED: Tag 0=Forward, Tag 1=Spin Head, Tag 2=Backward, Tag 3=Turn Right <<<")
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

print("--- BB-8 TAG REACTION DEMO READY ---")
print("Type 'demo' to start. Type 'manual' to stop.")

try:
    while True:
        # Capture the frame. (Note: picamera2 often defaults to RGB format)
        frame = picam2.capture_array()
        
        # OpenCV usually expects BGR for display, so we flip the colors for the display frame
        display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # The detector needs a grayscale image
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # --- 4. DEMO LOGIC ---
        if robot_mode == "REACTIVE":
            results = at_detector.detect(gray) 
            
            if len(results) > 0:
                tag_id = results[0].tag_id

                if tag_id == 0:
                    current_command = "FORWARD"
                elif tag_id == 1:
                    current_command = "SPIN_HEAD_RIGHT" 
                elif tag_id == 2:
                    current_command = "BACKWARD"
                elif tag_id == 3:  # <--- ADDED LOGIC FOR TAG 3
                    current_command = "TURN_RIGHT"
                else:
                    current_command = "STOP"
            else:
                current_command = "STOP"
                
        # --- 5. VIDEO DISPLAY ---
        # Draw the current state text onto the frame
        # Syntax: cv2.putText(image, text, (x, y), font, scale, (B, G, R) color, thickness)
        text_to_show = f"Mode: {robot_mode} | Cmd: {current_command}"
        cv2.putText(display_frame, text_to_show, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Show the video window
        cv2.imshow("BB-8 Vision Feed", display_frame)
        
        # Wait 1 millisecond (Required for OpenCV to actually update the display window)
        cv2.waitKey(1)
        
except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    current_command = "STOP"
    time.sleep(0.2)
    picam2.stop()
    sock.close()
    cv2.destroyAllWindows() # Clean up the video window