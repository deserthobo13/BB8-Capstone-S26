import time
import board
import adafruit_bno055

class IMUSensor:
    def __init__(self):
        print("Initializing BNO055 IMU...")
        self.i2c = board.I2C()  
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        time.sleep(1) # Give sensor a moment to boot

    def get_pitch(self):
        """
        Main function call for the balance loop used by the PID controller.
        Returns the pitch angle. Adjust the index [2] if sensor is mounted differently.
        """
        try:
            angles = self.sensor.euler
            if angles is None or angles[2] is None:
                return 0.0
            return angles[2]
        except OSError:
            # BNO055 sometimes throws I2C stretch errors. Just return 0.0 safely.
            return 0.0

# --- TESTING BLOCK ---
# This ONLY runs if you type `python IMU.py` in the terminal.
# It is completely ignored when imported by BB8_Movement.py
if __name__ == "__main__":
    imu = IMUSensor()
    print("Starting IMU Test...")
    while True:
        try:
            print(f"Euler angle: {imu.sensor.euler}")
            print(f"Pitch (Stabilization): {imu.get_pitch()}")
        except Exception as e:
            print(f"Error reading sensor: {e}")
        
        time.sleep(0.1) # Faster test loop