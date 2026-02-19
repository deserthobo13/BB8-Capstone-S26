import socket

# Set up the UDP socket
UDP_IP = "BB-8-Camera.local" # Listen on all interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Motor Pi: Ready for commands...")

while True:
    data, addr = sock.recvfrom(1024) # Buffer size 1024 bytes
    command = data.decode('utf-8')
    
    # Logic for Phase 1
    if command == "FORWARD":
        print("Engaging Motors: Moving Forward")
    elif command == "BACKWARD":
        print("Engaging Motors: Reversing")
    else:
        print(f"Received unknown command: {command}")