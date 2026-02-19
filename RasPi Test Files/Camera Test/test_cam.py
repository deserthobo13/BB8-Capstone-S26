import cv2
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()

# The OV5647 has a native 640x480 mode that is very fast
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("BB-8 Eyes Online (Sensor: OV5647)")
print("Press 'q' in the window to exit.")

try:
    while True:
        # Capture frame as a numpy array for OpenCV
        frame = picam2.capture_array()
        
        # Convert RGB to BGR for OpenCV display
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cv2.imshow("BB-8 Vision", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()