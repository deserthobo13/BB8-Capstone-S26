import socket
import time

# Set up the UDP socket
UDP_IP = "0.0.0.0" # Listens on all available network interfaces
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) # Prevents the script from freezing while waiting for data

print(f"Test Receiver: Listening for Head Pi on port {UDP_PORT}...")
print("Press Ctrl+C to stop.")

last_command_time = time.time()
TIMEOUT_SECONDS = 0.5
current_command = "NONE"

# --- Variables for calculating instruction rate ---
packet_count = 0
rate_timer = time.time()

try:
    while True:
        current_time = time.time()
        
        try:
            # Try to receive data
            data, addr = sock.recvfrom(1024)
            new_command = data.decode('utf-8')
            
            # Increment the packet counter for every single received message
            packet_count += 1
            
            # If the command changed, print it and the sender's IP
            if new_command != current_command:
                print(f"[{time.strftime('%H:%M:%S')}] Received: '{new_command}' from {addr[0]}")
                current_command = new_command
            
            last_command_time = current_time # Reset the safety timer
            
        except BlockingIOError:
            # No data received in this specific loop cycle
            pass

        # --- Rate Printing Logic (Triggers once per second) ---
        if current_time - rate_timer >= 1.0:
            # Only print the rate if we are actively connected
            if current_command != "TIMEOUT":
                print(f"--- Data Rate: {packet_count} packets/sec ---")
            
            # Reset counter and timer for the next second
            packet_count = 0
            rate_timer = current_time

        # Connection Timeout Check
        if current_time - last_command_time > TIMEOUT_SECONDS:
            if current_command != "TIMEOUT":
                print("--- CONNECTION LOST (No data for 0.5s) ---")
                current_command = "TIMEOUT"
        
        time.sleep(0.01) # 100Hz loop frequency

except KeyboardInterrupt:
    print("\nShutting down Test Receiver...")
finally:
    sock.close()