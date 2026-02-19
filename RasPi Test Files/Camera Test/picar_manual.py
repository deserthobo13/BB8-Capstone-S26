import cv2
import numpy as np
import time
from picamera2 import Picamera2
from pynput import keyboard
from picarx import Picarx
from pupil_apriltags import Detector

# 1. INITIALIZE ROBOT & SETTINGS
px = Picarx()
current_angle = 0
MAX_ANGLE = 40
TAG_SIZE = 165.1  # mm

# State tracking for smooth, non-stuttering movement
keys_pressed = {'w': False, 's': False, 'a': False, 'd': False}

# 2. LOAD CAMERA CALIBRATION
try:
    with np.load('camera_calib.npz') as data:
        mtx = data['mtx']
        cam_params = [mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2]]
except:
    print("Calibration file not found! Using default camera params.")
    cam_params = [600, 600, 320, 240]

# 3. KEYBOARD LISTENERS
def on_press(key):
    try:
        k = key.char
        if k in keys_pressed: keys_pressed[k] = True
    except AttributeError: pass

def on_release(key):
    try:
        k = key.char
        if k in keys_pressed: keys_pressed[k] = False
    except AttributeError: pass
    if key == keyboard.Key.esc: return False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# 4. SETUP VISION
at_detector = Detector(families='tag36h11')
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("--- BB-8 ONLINE: PRO DRIVE & VISION ---")
print("Drive: W/S/A/D | Exit: ESC or Q")

try:
    while True:
        # --- VISION STEP ---
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        results = at_detector.detect(gray, estimate_tag_pose=True, 
                                   camera_params=cam_params, 
                                   tag_size=TAG_SIZE)

        # Draw Tag Info
        for tag in results:
            z = tag.pose_t[2][0]  # Z distance in mm
            dist_inch = z / 25.4
            cx, cy = int(tag.center[0]), int(tag.center[1])
            
            cv2.circle(frame_bgr, (cx, cy), 8, (0, 255, 0), -1)
            cv2.putText(frame_bgr, f"ID:{tag.tag_id} DIST:{dist_inch:.1f}in", 
                        (cx - 60, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # --- MOTOR LOGIC STEP ---
        # Smooth Steering (increments by 2 degrees per frame)
        if keys_pressed['a']:
            current_angle = max(current_angle - 2, -MAX_ANGLE)
        elif keys_pressed['d']:
            current_angle = min(current_angle + 2, MAX_ANGLE)
        
        px.set_dir_servo_angle(current_angle)

        # Smooth Throttle
        if keys_pressed['w']:
            px.forward(45)
        elif keys_pressed['s']:
            px.backward(45)
        else:
            px.forward(0)

        # --- UI DISPLAY ---
        cv2.putText(frame_bgr, f"STEER: {current_angle} | TAGS: {len(results)}", 
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("BB-8 PRO HUB", frame_bgr)

        if cv2.waitKey(1) & 0xFF == ord('q') or not listener.running:
            break

finally:
    px.forward(0)
    px.set_dir_servo_angle(0)
    picam2.stop()
    cv2.destroyAllWindows()
    listener.stop()