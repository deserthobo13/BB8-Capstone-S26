import cv2
import numpy as np
import time
from picamera2 import Picamera2
from picarx import Picarx
from pupil_apriltags import Detector

px = Picarx()

# --- 1. SETTINGS ---
CRAWL_SPEED = 15      
SEARCH_SPEED = 25     
TARGET_DISTANCE = 70.0

# Distance Zones
START_MOVE_MIN = TARGET_DISTANCE - 5.0 
START_MOVE_MAX = TARGET_DISTANCE + 5.0 
STOP_FORWARD = TARGET_DISTANCE   
STOP_BACKWARD = TARGET_DISTANCE + 3.0  

# Stability Thresholds
PIXEL_DEADZONE = 20   
FACING_THRESHOLD = 10.0
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

# --- VIDEO RECORDING SETUP ---
# Saves as 'picar_recording.avi' at 20 frames per second
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('picar_recording.avi', fourcc, 20.0, (640, 480))

def update_servos(pan):
    pan = max(min(int(pan), 45), -45)
    px.set_cam_pan_angle(pan)
    px.set_cam_tilt_angle(0)
    return pan

print(f"--- BB-8: RECORDING MODE (WHEELS DISABLED) ---")
print(f"--- TARGET DISTANCE: {TARGET_DISTANCE} inches ---")

try:
    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        results = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[600, 600, 320, 240], tag_size=165.1)
        my_tag = next((t for t in results if t.tag_id == 0), None)
        
        status = "SEARCHING AREA..."

        if my_tag:
            missing_frame_count = 0
            is_tracking = True
            last_seen_time = time.time()
            
            error_x = my_tag.center[0] - 320
            
            if abs(error_x) > PIXEL_DEADZONE:
                current_pan += error_x * 0.05
                current_pan = update_servos(current_pan)
            
            z_inch = (my_tag.pose_t[2][0]) / 25.4
            steer_angle = int(current_pan * 1.0)

            if not is_moving:
                if z_inch < START_MOVE_MIN:
                    is_moving = True
                    move_dir = 'backward'
                elif z_inch > START_MOVE_MAX:
                    is_moving = True
                    move_dir = 'forward'
                elif abs(current_pan) > FACING_THRESHOLD:
                    is_moving = True
                    move_dir = 'backward' if z_inch < TARGET_DISTANCE else 'forward'
            
            if is_moving:
                # -------------------------------------------------------------
                # ### MOVEMENT HARDWARE COMMANDS ###
                # UNCOMMENT the lines below to enable physical driving
                # -------------------------------------------------------------
                if move_dir == 'backward':
                    # px.set_dir_servo_angle(-steer_angle)
                    # px.backward(CRAWL_SPEED)
                    pass # Hardware Disabled
                else:
                    # px.set_dir_servo_angle(steer_angle)
                    # px.forward(CRAWL_SPEED)
                    pass # Hardware Disabled
                # -------------------------------------------------------------

                if (move_dir == 'backward' and z_inch >= STOP_BACKWARD) or \
                   (move_dir == 'forward' and z_inch <= STOP_FORWARD):
                    px.forward(0)
                    is_moving = False
                    time.sleep(SETTLE_TIME)
                    
                status = f"MOVING {move_dir.upper()} ({z_inch:.1f} in)"
            else:
                px.forward(0)
                # px.set_dir_servo_angle(0) # UNCOMMENT to reset steering
                status = f"LOCKED ({z_inch:.1f} in)"

            ptA, ptB, ptC, ptD = my_tag.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            cv2.rectangle(frame_bgr, ptA, ptC, (0, 255, 0), 2)

        else:
            missing_frame_count += 1
            time_since_last_seen = time.time() - last_seen_time
            
            if missing_frame_count < MISSING_FRAME_LIMIT and is_tracking:
                status = "FILTERING..."
            elif time_since_last_seen < LOST_TIMEOUT:
                is_moving = False
                px.forward(0)
                status = "WAITING..."
            else:
                is_tracking = False
                is_moving = False
                px.forward(0)
                
                t = time.time()
                current_pan = 40 * np.sin(t * 1.5)
                update_servos(current_pan)
                
                status = "SEARCHING AREA..."

        # Display and Save Status
        cv2.putText(frame_bgr, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # WRITE FRAME TO VIDEO FILE
        out.write(frame_bgr)
        
        cv2.imshow("BB-8", frame_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    px.forward(0)
    px.set_dir_servo_angle(0)
    update_servos(0)
    picam2.stop()
    out.release() # CRITICAL: Saves and closes the video file
    cv2.destroyAllWindows()