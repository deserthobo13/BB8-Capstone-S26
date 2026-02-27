import os
import time
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

# --- SETUP HARDWARE PWM ---
# This tells gpiozero to use the pigpio engine for true hardware timing
print("Connecting to pigpio daemon...")
try:
    factory = PiGPIOFactory()
except Exception as e:
    print("ERROR: Could not connect to pigpio!")
    print("Did you forget to run 'sudo pigpiod' in the terminal?")
    exit()

# Speed Pin (AN2) -> GPIO 18 (Native Hardware PWM0)
# Direction Pin (IN2) -> GPIO 6
print("Initializing Motor on GPIO 18 (Speed) and GPIO 6 (Direction) at 10,000Hz...")

speed = PWMOutputDevice(18, initial_value=0.0, frequency=10000, pin_factory=factory)
direction = DigitalOutputDevice(6, pin_factory=factory)

print("\n--- CONTINUOUS TEST: HARDWARE PWM (SIGN-MAGNITUDE) ---")
print("Press Ctrl+C to safely stop.")

try:
    while True:
        # Forward
        print("Moving Forward (50%)...")
        direction.off()
        speed.value = 0.5
        time.sleep(2)
        
        # Stop
        print("Stopping...")
        speed.value = 0.0
        time.sleep(1)
        
        # Backward
        print("Moving Backward (50%)...")
        direction.on()
        speed.value = 0.5
        time.sleep(2)
        
        # Stop
        print("Stopping...")
        speed.value = 0.0
        time.sleep(1)

except KeyboardInterrupt:
    print("\nTest interrupted. Hardware PWM safely dropped to 0V.")
    speed.value = 0.0