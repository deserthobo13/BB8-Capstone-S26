import socket
import time

MOTOR_PI_IP = "IP Address" # Change to Body IP Address
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(cmd):
    sock.sendto(bytes(cmd, "utf-8"), (MOTOR_PI_IP, UDP_PORT))

# Placeholder for your AI logic
# If distance > 100cm: send_command("FORWARD")
# If distance < 20cm: send_command("BACKWARD")