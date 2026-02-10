import time
import math
from evdev import InputDevice, categorize, ecodes, list_devices

# Hardware libraries
from gpiozero import PWMOutputDevice
from gpiozero import OutputDevice
from adafruit_servokit import ServoKit
import adafruit_bno055 
import adafruit_motor
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import busio, board

# --- 1. SETUP CONTROLLER ---
# Automatically find the Xbox controller path
controller_path = None
print("Searching for Xbox controller...")
devices = [InputDevice(path) for path in list_devices()]
for device in devices:
    if "Xbox" in device.name:
        controller_path = device.path
        print(f"Found Controller: {device.name} at {controller_path}")
        break

if controller_path is None:
    print("No Xbox controller found! Please pair via Bluetooth first.")
    exit()

gamepad = InputDevice(controller_path)

# --- 2. SETUP HARDWARE ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50 

# DC Motors
DC_motor1 = PWMOutputDevice(13, initial_value=0.5, frequency=10000)
DC_motor2 = PWMOutputDevice(18, initial_value=0.5, frequency=10000)
Turn_motor = PWMOutputDevice(19, initial_value=0.5, frequency=10000)
motor_driver_relay = OutputDevice(17, active_high=False)
turn_driver_relay = OutputDevice(27, active_high=False)

# Activate Relays
print("Activating Motor Relays...")
motor_driver_relay.on()
turn_driver_relay.on()

# --- 3. HELPER FUNCTIONS ---

def map_motor_value(value, deadzone=2000):
    """
    Maps Xbox joystick input (approx -32768 to 32767) to Motor PWM (0 to 1).
    0.5 is stopped. 
    1.0 is full forward. 
    0.0 is full reverse.
    """
    # 1. Apply Deadzone (ignore small drift)
    if abs(value) < deadzone:
        return 0.5

    # 2. Normalize to -1.0 (Down/Back) to +1.0 (Up/Forward)
    # Note: Xbox sticks usually read Negative for Up, Positive for Down.
    # We invert 'value' here so Up becomes Positive.
    norm_value = -value / 32768.0 

    # Cap it just in case it exceeds 1.0 slightly
    norm_value = max(-1.0, min(1.0, norm_value))

    # 3. Convert -1..1 range to 0.0..1.0 range (centered at 0.5)
    # Output = (Input / 2) + 0.5
    final_motor_val = (norm_value / 2) + 0.5
    
    return final_motor_val

# --- 4. MAIN CONTROL LOOP ---
print("\nREADY TO DRIVE! (Press Ctrl+C to stop)")
print("Left Stick Y-Axis controls Forward/Backward speed.")

try:
    for event in gamepad.read_loop():
        # logic for Forward/Backwards (Left Stick Y-Axis is usually ABS_Y / Code 01)
        if event.type == ecodes.EV_ABS:
            
            # Code 1 is usually the Left Joystick Vertical (Y) axis
            if event.code == 1: 
                
                # Convert raw joystick number to motor speed (0.0 - 1.0)
                speed = map_motor_value(event.value)
                
                # Apply to motors
                DC_motor1.value = speed
                DC_motor2.value = speed
                
                # Optional: Uncomment to debug values if it's acting weird
                # print(f"Joy: {event.value} -> Motor: {speed:.2f}")

except KeyboardInterrupt:
    print("\nStopping...")

except OSError:
    print("\nController disconnected!")

finally:
    # --- SAFETY SHUTDOWN ---
    # Stop motors and turn off relays when script ends or crashes
    DC_motor1.value = 0.5
    DC_motor2.value = 0.5
    Turn_motor.value = 0.5
    motor_driver_relay.off()
    turn_driver_relay.off()
    print("Safe Shutdown Complete.")