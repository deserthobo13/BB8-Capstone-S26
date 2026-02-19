import cv2
import numpy as np
from picamera2 import Picamera2
import os

# Checkerboard settings: (internal corners)
# If your board is 7x9 squares, the internal corners are 6x8
CHECKERBOARD = (7, 7) 
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store points
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Prepare real world coordinates (0,0,0), (1,0,0) ...
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Calibration Started. Hold the board in view.")
print("Press 's' to save a frame, 'q' to finish and calculate.")

count = 0
try:
    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        # Add flags to make the search more aggressive
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK

        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, flags)

        if ret == True:
            # Draw and display the corners so you know it's working
            cv2.drawChessboardCorners(frame_bgr, CHECKERBOARD, corners, ret)
            
            key = cv2.waitKey(1)
            if key & 0xFF == ord('s'):
                objpoints.append(objp)
                imgpoints.append(corners)
                count += 1
                print(f"Saved snapshot {count}/20")

        cv2.imshow('Calibration', frame_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q') or count >= 20:
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()

if count > 10:
    print("Calculating calibration... please wait.")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    # Save the results
    np.savez("camera_calib.npz", mtx=mtx, dist=dist)
    print("Calibration saved to camera_calib.npz!")
    print("Camera Matrix:\n", mtx)
else:
    print("Not enough snapshots taken.")