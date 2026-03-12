import time
from IMU import IMUSensor

# Initialize the sensor
imu = IMUSensor()

# Initialize global tracking variables
velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
last_time = time.time()

def update_position(imu_device):
    
    return position