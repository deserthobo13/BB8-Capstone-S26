import os
import time
import socket

# --- 1. HARDWARE PWM FIX ---
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

# --- 2. IMPORT MOTOR LIBRARIES ---
from Movement_Functions import BB8Movement

bb8 = BB8Movement()
bb8.enable_system() 

bb8.head_rotate.throttle = -0.05

UDP_IP = "0.0.0.0" 
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) 

print(f"Motor Pi: Listening for Head Pi on port {UDP_PORT}...")

last_command_time = time.time()
TIMEOUT_SECONDS = 0.5
current_command = "STOP"
previous_command = "NONE"

# --- IMPULSE STEERING VARIABLES ---
# Tune these two numbers to find the mechanical "sweet spot"
PULSE_ON_TIME = 0.15  # Seconds the motor kicks ON (High torque jolt)
PULSE_OFF_TIME = 0.10 # Seconds the motor rests OFF (Allows momentum to settle)

is_pulsing_on = False
last_pulse_time = 0

try:
    while True:
        current_time = time.time()
        
        try:
            data, addr = sock.recvfrom(1024)
            new_command = data.decode('utf-8')
            if new_command != current_command:
                current_command = new_command
            last_command_time = current_time 
        except BlockingIOError:
            pass

        # SAFETY WATCHDOG
        if current_time - last_command_time > TIMEOUT_SECONDS:
            if current_command != "STOP":
                current_command = "STOP"
        
        # --- STATE TRANSITION LOGIC (Runs ONCE when command changes) ---
        if current_command != previous_command:
            print(f"[{time.strftime('%H:%M:%S')}] Executing: '{current_command}'")
            
            # CLEANUP: If we just stopped turning, explicitly zero out the steer motor
            if previous_command in ["TURN_RIGHT", "TURN_LEFT"] and current_command not in ["TURN_RIGHT", "TURN_LEFT"]:
                bb8.steer(0.0)
                is_pulsing_on = False
            
            if current_command == "FORWARD":
                bb8.drive(0.3)
                bb8.spin_head(-0.05) 
                print(">>> FORWARD SIGNAL RECEIVED: Attempting to drive forward <<<")
            elif current_command == "BACKWARD":
                bb8.drive(-0.3)
                bb8.spin_head(-0.05) 
                print(">>> BACKWARD SIGNAL RECEIVED: Attempting to drive backward <<<")
            elif current_command == "STOP":
                bb8.stop_all()
            elif current_command == "SPIN_HEAD_LEFT":
                bb8.spin_head(-0.5)
            elif current_command == "SPIN_HEAD_RIGHT":
                bb8.spin_head(0.5)
            elif current_command == "SCAN":
                print(">>> SCAN SIGNAL RECEIVED: Attempting to spin <<<")
                bb8.spin_head(0.5)
            # Notice TURN_LEFT and TURN_RIGHT are missing here! 
            # They are handled continuously below instead.
            
            previous_command = current_command
        
        # --- CONTINUOUS LOGIC (Runs EVERY LOOP for active states) ---
        if current_command in ["TURN_RIGHT", "TURN_LEFT"]:
            # Decide how long we should wait based on whether the pulse is currently ON or OFF
            wait_time = PULSE_ON_TIME if is_pulsing_on else PULSE_OFF_TIME
            
            # If enough time has passed, toggle the motor
            if current_time - last_pulse_time >= wait_time:
                is_pulsing_on = not is_pulsing_on # Flip the state
                last_pulse_time = current_time    # Reset the timer
                
                # Determine direction
                steer_direction = 1.0 if current_command == "TURN_RIGHT" else -1.0
                
                if is_pulsing_on:
                    bb8.steer(steer_direction) # Send the jolt!
                else:
                    bb8.steer(0.0)             # Cut the power!

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nShutting down Body Pi...")
finally:
    bb8.rest_all_servos() 
    bb8.stop_all() 
    bb8.disable_system()
    sock.close()