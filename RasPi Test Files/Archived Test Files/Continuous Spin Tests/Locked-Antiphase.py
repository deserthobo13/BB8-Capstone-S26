import pigpio
import time
import signal
import sys

# Configuration
PWM_PIN = 18          # Must be a hardware PWM capable pin (12, 13, 18, or 19 on Pi)
FREQ = 20000          # 20 kHz (Out of human hearing range)
RANGE = 1000000       # pigpio hardware PWM range (1,000,000 is default)

# Calculate duty cycles based on the 1M range
STOP_DUTY = int(RANGE * 0.50)   # 50%
FWD_DUTY = int(RANGE * 0.75)    # 75%
REV_DUTY = int(RANGE * 0.25)    # 25%

# Initialize connection to pigpiod
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpiod. Did you run 'sudo pigpiod'?")
    sys.exit(1)

def graceful_exit(signum=None, frame=None):
    """Catches kill signals to safely stop the motor before exiting."""
    print("\n[!] Script killed or interrupted. Stopping motor...")
    
    # Force the motor to 50% (Stopped in locked-antiphase)
    pi.hardware_PWM(PWM_PIN, FREQ, STOP_DUTY)
    
    # Note: We do NOT set the pin to 0 (LOW), because 0V = Full Reverse!
    
    pi.stop()
    sys.exit(0)

# Register signal handlers for Ctrl+C (SIGINT) and termination (SIGTERM)
signal.signal(signal.SIGINT, graceful_exit)
signal.signal(signal.SIGTERM, graceful_exit)

try:
    print("Starting Locked-Antiphase Test...")
    
    print("1. Neutral (50%)")
    pi.hardware_PWM(PWM_PIN, FREQ, STOP_DUTY)
    time.sleep(3)

    print("2. Moving Forward (75%)")
    pi.hardware_PWM(PWM_PIN, FREQ, FWD_DUTY)
    time.sleep(3)
    
    # Try killing the script (Ctrl+C) during this sleep!
    print("3. Moving Reverse (25%) - Try pressing Ctrl+C now!")
    pi.hardware_PWM(PWM_PIN, FREQ, REV_DUTY)
    time.sleep(5) 

    print("4. Back to Neutral (50%)")
    pi.hardware_PWM(PWM_PIN, FREQ, STOP_DUTY)
    time.sleep(2)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Ensure exit routine runs even if the try block fails naturally
    graceful_exit()