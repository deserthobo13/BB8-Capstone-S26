import socket
import time
import threading

# --- Configuration ---
MOTOR_PI_IP = "bb8pi.local"  # Ensure this matches your Body Pi's network name or IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Map user-friendly inputs to the exact strings expected by the Body Pi logic
COMMAND_MAP = {
    'w': "FORWARD",
    's': "BACKWARD",
    'x': "STOP",
    'a': "SPIN_HEAD_LEFT",
    'd': "SPIN_HEAD_RIGHT",
    'stop': "STOP",
    'forward': "FORWARD",
    'backward': "BACKWARD"
}

# Global variable to hold the current state
current_command = "STOP"

def send_udp_stream():
    """
    Background thread function that continuously blasts the 
    current command to the Body Pi at 10Hz (every 0.1s).
    This prevents the Body Pi's watchdog timer from triggering.
    """
    while True:
        try:
            sock.sendto(bytes(current_command, "utf-8"), (MOTOR_PI_IP, UDP_PORT))
        except Exception as e:
            pass # Ignore temporary network blips
        time.sleep(0.1) 

print("--- BB-8 Manual Control Test (Head Pi) ---")
print("Controls: W=Forward, S=Backward, A=Head Left, D=Head Right, X=Stop")
print("Type 'exit' to quit.")

# Start the continuous sending thread
stream_thread = threading.Thread(target=send_udp_stream, daemon=True)
stream_thread.start()

try:
    while True:
        # The input() function blocks here, but the background thread keeps sending!
        user_input = input("\nEnter Command: ").strip().lower()

        if user_input == 'exit':
            break
        
        # Update the global command variable based on input
        if user_input in COMMAND_MAP:
            current_command = COMMAND_MAP[user_input]
            print(f"State changed to: {current_command}")
        else:
            current_command = user_input.upper()
            print(f"State changed to: {current_command} (Raw)")

except KeyboardInterrupt:
    print("\nShutting down Head Pi Controller...")
finally:
    current_command = "STOP" # Update state
    time.sleep(0.2) # Give the thread a moment to send the final STOP packet
    sock.close()