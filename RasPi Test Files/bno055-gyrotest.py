import time
import board
import adafruit_bno055

i2c = board.I2C()  
sensor = adafruit_bno055.BNO055_I2C(i2c)

def get_stabilization_data():
    try:
        angles = sensor.euler
        if angles is None:
            return 0.0
        return angles[2]
    except OSError:
        return 0.0

while True:
    # Useless info that only slows down the sampling rate:
    #print("Temperature: {} degrees C".format(temperature()))
    #print(f"Accelerometer (m/s^2): {sensor.acceleration}")
    #print(f"Magnetometer (microteslas): {sensor.magnetic}")
    #print(f"Gyroscope (rad/sec): {sensor.gyro}")
    #print(f"Linear acceleration (m/s^2): {sensor.linear_acceleration}")
    #print(f"Gravity (m/s^2): {sensor.gravity}")
    #print()
    
    #Important info for PID loop; Use quaternion if euler becomes erratic.
    print(f"Euler angle: {sensor.euler}")
    #print(f"Quaternion: {sensor.quaternion}")

    # Delete this sleep function before testing PID loop.
    time.sleep(1)
