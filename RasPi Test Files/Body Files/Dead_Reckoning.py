import time
from IMU import IMUSensor

imu = IMUSensor()

# --- CONFIGURATION ---
ACCEL_TRIGGER = 0.15     # Start recording when accel exceeds this (m/s^2)
VELOCITY_STOP = 0.05     # Stop recording when velocity drops below this (m/s)
MIN_MOVE_TIME = 0.5      # Ignore stops that happen instantly (prevents false starts)
METERS_TO_INCHES = 39.37

def run_velocity_test():
    velocity = 0.0
    distance = 0.0
    is_moving = False
    
    print("\n[READY] Waiting for Z-axis motion to trigger...")
    
    last_time = time.time()
    start_time = None

    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        accel_data = imu.get_linear_acceleration()
        if accel_data is None: continue
        
        # Z-axis (Forwards/Backwards)
        accel_z = accel_data[2]

        # 1. TRIGGER START: Based on Acceleration
        if not is_moving:
            if abs(accel_z) > ACCEL_TRIGGER:
                is_moving = True
                start_time = current_time
                print(">>> MOTION DETECTED. Recording velocity...")
            continue

        # 2. INTEGRATE: v = v + a*dt
        # We apply a tiny deadzone to the acceleration to keep the velocity clean
        clean_accel = accel_z if abs(accel_z) > 0.05 else 0.0
        velocity += clean_accel * dt
        
        # 3. INTEGRATE: d = d + v*dt
        distance += velocity * dt

        # 4. TRIGGER STOP: Based on Velocity
        # We check if velocity is near zero AND we've been moving for at least a moment
        elapsed = current_time - start_time
        if elapsed > MIN_MOVE_TIME and abs(velocity) < VELOCITY_STOP:
            break

        # Live telemetry
        print(f"Time: {elapsed:4.1f}s | Vel: {velocity:6.2f} | Dist: {distance * METERS_TO_INCHES:6.2f} in", end="\r")
        time.sleep(0.01)

    print(f"\n\n[HALT DETECTED] Motion Finished.")
    print(f"Duration: {current_time - start_time:.2f} seconds")
    print(f"Total Distance: {abs(distance * METERS_TO_INCHES):.2f} inches")
    print("--------------------------------")

if __name__ == "__main__":
    try:
        while True:
            run_velocity_test()
            if input("Run another test? (y/n): ").lower() != 'y':
                break
    except KeyboardInterrupt:
        print("\nExiting...")