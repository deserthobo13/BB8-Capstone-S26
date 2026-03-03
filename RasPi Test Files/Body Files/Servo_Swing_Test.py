import time
import busio
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def run_servo_test():
    print("--- Starting Pendulum Servo Hardware Test ---")
    print("Initializing I2C and PCA9685...")
    
    # Initialize I2C and the PCA9685 board
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50 

    # Define the two swing servos based on your Movement_Function_PID.py channels
    swing_servo = servo.Servo(pca.channels[0])
    swing_servo2 = servo.Servo(pca.channels[4])

    # Your exact pendulum function
    def set_swing(degrees):
        """Sets internal pendulum swing. Safe range 70 to 117"""
        current_swing = max(70, min(117, degrees))
        difference = current_swing - 90
        print(f"--> Commanding servos to {current_swing} degrees.")
        swing_servo.angle = 90 + difference
        swing_servo2.angle = 90 - difference

    print("\nStand back and watch the pendulum!")
    time.sleep(2)

    try:
        # 1. Center the pendulum
        print("\nStep 1: Centering pendulum (90 degrees)")
        set_swing(90)
        time.sleep(3) # Wait 3 seconds so you can observe

        # 2. Test > 90 degrees
        print("\nStep 2: Testing 110 degrees.")
        print("QUESTION: Does the pendulum swing LEFT or RIGHT?")
        set_swing(110)
        time.sleep(4)

        # 3. Center the pendulum
        print("\nStep 3: Centering pendulum (90 degrees)")
        set_swing(90)
        time.sleep(3)

        # 4. Test < 90 degrees
        print("\nStep 4: Testing 70 degrees.")
        print("QUESTION: Does the pendulum swing LEFT or RIGHT?")
        set_swing(70)
        time.sleep(4)

        # 5. Finish and center
        print("\nTest complete. Centering and exiting.")
        set_swing(90)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nTest manually stopped. Attempting to center pendulum...")
        set_swing(90)

if __name__ == "__main__":
    run_servo_test()