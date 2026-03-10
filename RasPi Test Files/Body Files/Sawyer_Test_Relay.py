from gpiozero import OutputDevice
from time import sleep

# Define the relay pin using the BCM number (GPIO 21 is physical pin 40).
# active_high=False tells OutputDevice to use 0V for ON and 3.3V for OFF.
# initial_value=False forces it to start in the OFF state.
motor_relay = OutputDevice(21, active_high=False, initial_value=False)

def enable_motors():
    """Turns the relay ON, allowing power to flow to the Cytron driver."""
    print("Engaging Main Drive Power...")
    motor_relay.on()  # This safely pulls GPIO 21 down to 0V (Ground)

def disable_motors():
    """Turns the relay OFF, killing power to the Cytron driver."""
    print("Killing Main Drive Power...")
    motor_relay.off() # This safely pushes GPIO 21 back to 3.3V

# --- Quick Test Sequence ---
if __name__ == "__main__":
    print("Starting Relay Test...")
    
    # Force the relay into a safe, OFF state immediately just to be sure
    disable_motors()
    sleep(1)
    
    # Listen for the loud CLICK!
    enable_motors()
    sleep(3) # Hold it on for 3 seconds
    
    # Listen for it to click back open
    disable_motors()
    print("Test Complete. Motors are Safe.")