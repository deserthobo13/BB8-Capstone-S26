import cv2
import numpy as np
import time
from picamera2 import Picamera2
from picarx import Picarx
from pupil_apriltags import Detector

px = Picarx()

# --- 1. SETTINGS (TUNE THESE) ---
CRAWL_SPEED = 15       # Your new preferred speed
SEARCH_SPEED = 25      # Speed for the search rotation

# Distance Zones
START_MOVE_MIN = 47.0  
START_MOVE_MAX = 57.0  
STOP_FORWARD = 52.0    
STOP_BACKWARD = 55.0   

# Stability Thresholds (The Jitter Fixers)
PIXEL_DEADZONE = 20    # Ignore tag movements smaller than this (pixels)
FACING_THRESHOLD = 10.0 # Ignore camera angles smaller than this (degrees)
SETTLE_TIME = 0.5      

# Timing
MISSING_FRAME_LIMIT = 8 
LOST_TIMEOUT = 3.0      

# --- 2. INITIALIZATION ---
current_pan = 0
last_seen_time = 0
is_tracking = False 
is_moving = False  
move_dir = None 
missing_frame_count = 0

at_detector = Detector(families='tag36h11')
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

def update_servos(pan):
    pan = max(min(int(pan), 45), -45)
    px.set_cam_pan_angle(pan)
    px.set_cam_tilt_angle(0) 
    return pan

print(f"--- BB-8: SEARCH RESTORED (Speed 15) ---")

try:
    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        results = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[600, 600, 320, 240], tag_size=165.1)
        my_tag = next((t for t in results if t.tag_id == 0), None)

        if my_tag:
            # --- LOCK ACQUIRED ---
            missing_frame_count = 0 
            is_tracking = True
            last_seen_time = time.time()
            
            error_x = my_tag.center[0] - 320
            
            # Apply Pixel Deadzone to stop camera twitching
            if abs(error_x) > PIXEL_DEADZONE:
                current_pan += error_x * 0.05
                current_pan = update_servos(current_pan)
            
            z_inch = (my_tag.pose_t[2][0]) / 25.4
            steer_angle = int(current_pan * 1.0) 

            if not is_moving:
                # Check if we need to wake up and move
                if z_inch < START_MOVE_MIN:
                    is_moving = True
                    move_dir = 'backward'
                elif z_inch > START_MOVE_MAX:
                    is_moving = True
                    move_dir = 'forward'
                elif abs(current_pan) > FACING_THRESHOLD:
                    is_moving = True
                    move_dir = 'backward' if z_inch < 52 else 'forward'
            
            if is_moving:
                if move_dir == 'backward':
                    px.set_dir_servo_angle(-steer_angle)
                    px.backward(CRAWL_SPEED)
                else:
                    px.set_dir_servo_angle(steer_angle)
                    px.forward(CRAWL_SPEED)

                if (move_dir == 'backward' and z_inch >= STOP_BACKWARD) or \
                   (move_dir == 'forward' and z_inch <= STOP_FORWARD):
                    px.forward(0)
                    is_moving = False
                    time.sleep(SETTLE_TIME)
            else:
                px.forward(0)
                px.set_dir_servo_angle(0)
                status = "LOCKED"

        else:
            # --- TAG MISSING ---
            missing_frame_count += 1
            time_since_last_seen = time.time() - last_seen_time
            
            if missing_frame_count < MISSING_FRAME_LIMIT and is_tracking:
                # Brief flicker, keep doing whatever we were doing
                status = "FILTERING..."
            elif time_since_last_seen < LOST_TIMEOUT:
                # Tag is definitely gone, but wait before searching
                is_moving = False
                px.forward(0)
                status = "WAITING..."
            else:
                # --- SEARCH LOGIC RESTORED ---
                is_tracking = False
                is_moving = False
                px.forward(0)
                
                # Camera Sweep
                t = time.time()
                current_pan = 40 * np.sin(t * 1.5)
                update_servos(current_pan)
                
                # Optional: Make the car body rotate slightly during search
                # px.set_dir_servo_angle(30 if current_pan > 0 else -30)
                # px.forward(SEARCH_SPEED)
                
                status = "SEARCHING AREA..."

        cv2.putText(frame_bgr, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("BB-8", frame_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    px.forward(0)
    px.set_dir_servo_angle(0)
    update_servos(0)
    picam2.stop()