import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector

# 1. LOAD CALIBRATION DATA
with np.load('camera_calib.npz') as data:
    camera_matrix = data['mtx']
    dist_coeffs = data['dist']

# 2. TAG SETTINGS
TAG_SIZE = 165.1  # The width of your tag in millimeters
at_detector = Detector(families='tag36h11')

# 3. START CAMERA
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("BB-8 Tracking Online. Looking for tags 0-5...")

try:
    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Detect tags
        results = at_detector.detect(gray, estimate_tag_pose=True, 
                                   camera_params=[camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2]], 
                                   tag_size=TAG_SIZE)

        for tag in results:
            # Extract Position (tvec) and Rotation (R)
            # tvec is [x, y, z]. Z is the distance forward.
            x, y, z = tag.pose_t[0][0], tag.pose_t[1][0], tag.pose_t[2][0]
            
            # Convert to inches for easy reading
            dist_inch = z / 25.4
            
            # Draw on screen
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv2.circle(frame_bgr, (cx, cy), 5, (0, 255, 0), -1)
            
            # Print distance to screen
            text = f"ID: {tag.tag_id} | Dist: {dist_inch:.1f} in"
            cv2.putText(frame_bgr, text, (cx - 50, cy - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            print(f"Tag {tag.tag_id} found at: {dist_inch:.2f} inches away")

        cv2.imshow("BB-8 Vision", frame_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()