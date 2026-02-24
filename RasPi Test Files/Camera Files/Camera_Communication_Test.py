import socket
import time

# Configuration
MOTOR_PI_IP = "bb8pi.local"  # Ensure this matches your Body Pi's network name or IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(cmd):
    """Sends the command string to the Body Pi via UDP."""
    sock.sendto(bytes(cmd, "utf-8"), (MOTOR_PI_IP, UDP_PORT))
    print(f"Sent: {cmd}")

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

print("--- BB-8 Manual Control Test (Head Pi) ---")
print("Controls: W=Forward, S=Backward, A=Head Left, D=Head Right, X=Stop")
print("Type 'exit' to quit.")

try:
    while True:
        user_input = input("\nEnter Command: ").strip().lower()

        if user_input == 'exit':
            break
        
        # Check if the input is in our map, otherwise send the raw input
        if user_input in COMMAND_MAP:
            cmd_to_send = COMMAND_MAP[user_input]
        else:
            # Send as-is in uppercase if not in the shortcut map
            cmd_to_send = user_input.upper()

        send_command(cmd_to_send)

except KeyboardInterrupt:
    print("\nShutting down Head Pi Controller...")
finally:
    send_command("STOP") # Safety: stop the robot on exit
    sock.close()