import socket
import time
import threading
import cv2
from picamera2 import Picamera2
from pupil_apriltags import Detector

# --- Configuration ---
MOTOR_PI_IP = "bb8pi.local"  # Ensure this matches your Body Pi's network name or IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Global variable to hold the current state
current_command = "STOP"

def send_udp_stream():
    """
    Background thread function that continuously blasts the 
    current command to the Body Pi at 10Hz (every 0.1s).
    """
    while True:
        try:
            sock.sendto(bytes(current_command, "utf-8"), (MOTOR_PI_IP, UDP_PORT))
        except Exception as e:
            pass # Ignore temporary network blips
        time.sleep(0.1) 

print("--- BB-8 Autonomous Vision Control (Head Pi) ---")
print("Press 'q' on the video window to stop.")

# Start the continuous sending thread
stream_thread = threading.Thread(target=send_udp_stream, daemon=True)
stream_thread.start()

# --- NEW ADDITION: VISION SYSTEM INIT ---
at_detector = Detector(families='tag36h11')
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

PIXEL_DEADZONE = 40 # Deadzone to prevent jitter

try:
    # Replaced the manual 'input()' with the continuous camera capture
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        results = at_detector.detect(gray)
        my_tag = next((t for t in results if t.tag_id == 0), None)

        if my_tag:
            # --- TAG SEEN: UPDATE COMMAND ---
            error_x = my_tag.center[0] - 320
            
            if error_x > PIXEL_DEADZONE:
                current_command = "SPIN_HEAD_RIGHT"
            elif error_x < -PIXEL_DEADZONE:
                current_command = "SPIN_HEAD_LEFT"
            else:
                current_command = "STOP"
            
            # Draw a circle on the tag for debugging
            cv2.circle(frame, (int(my_tag.center[0]), int(my_tag.center[1])), 10, (0, 255, 0), -1)
        else:
            # --- TAG LOST: SCAN MODE ---
            current_command = "SCAN"

        # Show the video feed and the current command being sent via UDP
        cv2.putText(frame, f"Sending UDP: {current_command}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("BB-8 Head", frame)
        
        # Press 'q' to break the loop instead of typing 'exit'
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

except KeyboardInterrupt:
    print("\nShutting down Head Pi Controller...")
finally:
    current_command = "STOP" # Update state
    time.sleep(0.2) # Give the thread a moment to send the final STOP packet
    picam2.stop()
    cv2.destroyAllWindows()
    sock.close()