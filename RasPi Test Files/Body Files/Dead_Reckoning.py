class DeadReckoning:
    def __init__(self):
        # --- CONFIGURATION ---
        self.ACCEL_TRIGGER = 0.15     # Start recording when accel exceeds this (m/s^2)
        self.VELOCITY_STOP = 0.05     # Stop recording when velocity drops below this (m/s)
        self.MIN_MOVE_TIME = 0.5      # Ignore stops that happen instantly (prevents false starts)
        self.METERS_TO_INCHES = 39.37

        # --- STATE VARIABLES ---
        self.velocity = 0.0
        self.distance = 0.0
        self.is_moving = False
        self.move_timer = 0.0

    def update(self, accel_z, dt):
        if dt <= 0:
            return self.velocity, self.distance

        # Apply a tiny deadzone to the acceleration
        clean_accel = accel_z if abs(accel_z) > 0.05 else 0.0

        # 1. TRIGGER START
        if not self.is_moving:
            if abs(clean_accel) > self.ACCEL_TRIGGER:
                self.is_moving = True
                self.move_timer = 0.0
                self.velocity = 0.0
                self.distance = 0.0
                self.zero_accel_timer = 0.0
                print("\n>>> MOTION DETECTED. Recording velocity...")
            else:
                return self.velocity, self.distance # Return if we aren't moving yet

        # 2. INTEGRATE: We are currently moving
        self.move_timer += dt
        self.velocity += clean_accel * dt
        self.distance += self.velocity * dt

        # 3. TIMEOUT LOGIC (Fixes the infinite coasting bug)
        if clean_accel == 0.0:
            self.zero_accel_timer += dt
        else:
            self.zero_accel_timer = 0.0 # Reset if we see movement

        # 4. TRIGGER STOP
        # Stop if velocity naturally drops low enough, OR if we've seen zero acceleration for 0.5 seconds
        if self.move_timer > self.MIN_MOVE_TIME:
            if abs(self.velocity) < self.VELOCITY_STOP or self.zero_accel_timer > 0.5:
                self.is_moving = False
                print(f"\n[HALT DETECTED] Motion Finished.")
                print(f"Duration: {self.move_timer:.2f} seconds")
                print(f"Total Distance: {abs(self.distance * self.METERS_TO_INCHES):.2f} inches")
                print("--------------------------------")
                
                # Force velocity to zero so it doesn't bleed into the next movement
                self.velocity = 0.0 

        return self.velocity, self.distance

# --- TESTING BLOCK ---
if __name__ == "__main__":
    import time
    
    # Dummy test to simulate the main 50Hz loop feeding it data
    dr = DeadReckoning()
    print("Simulating a 50Hz control loop feeding data to DeadReckoning...")
    
    # Simulate a sudden forward acceleration, coasting, and stopping
    simulated_accels = [0.0, 0.0, 0.8, 0.8, 0.0, -0.2, -0.2, -0.2, 0.0, 0.0]
    
    last_time = time.time()
    for accel in simulated_accels:
        time.sleep(0.5) # Fake loop delay
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        vel, dist = dr.update(accel, dt)
        if dr.is_moving:
            print(f"Accel: {accel:4.1f} | Vel: {vel:5.2f} | Dist: {dist * dr.METERS_TO_INCHES:5.2f} in")