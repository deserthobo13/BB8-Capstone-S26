import time
import board
import adafruit_bno055

class IMUSensor:
    def __init__(self):
        print("Initializing BNO055 IMU...")
        self.i2c = board.I2C()  
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        time.sleep(1) # Give sensor a moment to boot
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.accel_z = 0.0
        
    def update(self):
        """Call this exactly ONCE at the start of your main loop."""
        try:
            angles = self.sensor.euler
            linear_acceleration = self.sensor.linear_acceleration
            if angles is not None:
                self.yaw = angles[0] if angles[0] is not None else self.yaw
                self.roll = angles[1] if angles[1] is not None else self.roll
                self.pitch = angles[2] if angles[2] is not None else self.pitch
            if linear_acceleration is not None and linear_acceleration[2] is not None:
                self.accel_z = linear_acceleration[2]
        except OSError:
            pass # Handle I2C stretch errors silently to keep loop alive

# --- TESTING BLOCK ---
if __name__ == "__main__":
    imu = IMUSensor()
    print("Starting IMU Test...")
    while True:
        try:
            # 1. Update all sensor data once per loop
            imu.update()
            
            # 2. Print the stored attributes
            print(f"Raw Euler: {imu.sensor.euler}")
            print(f"Pitch (Stabilization): {imu.pitch}")
            print(f"Roll: {imu.roll}")
            print(f"Yaw: {imu.yaw}")
            
        except Exception as e:
            print(f"Error reading sensor: {e}")
        
        time.sleep(0.1) # Faster test loop