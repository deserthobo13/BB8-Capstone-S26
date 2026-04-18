import time

class PIDController:
    def __init__(self, kp, ki, kd, alpha_d=0.15):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Defining derivative filter parameter (0.15 is a good shock absorber)
        self.alpha_d = alpha_d
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.smoothed_derivative = 0.0
        self.last_time = time.time()
        
        # --- Values for Telemetry ---
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.error = 0.0
        self.output = 0.0
        
    def reset(self):
        """Call this right before starting the balance loop to prevent massive dt spikes"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.smoothed_derivative = 0.0
        self.last_time = time.time()
        
    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Prevent divide by zero errors if loop runs too fast
        if dt <= 0:
            return 0.0 

        # 1. Calculate Error
        self.error = setpoint - measured_value
        
        # 2. Proportional
        self.p_term = self.kp * self.error
        
        # 3. Integral (with Anti-Windup)
        # Clamped so it doesn't build up infinitely if the robot is stuck
        self.integral += self.error * dt
        self.integral = max(min(self.integral, 10.0), -10.0) 
        self.i_term = self.ki * self.integral
        
        # 4. Derivative (WITH LOW-PASS EMA FILTER)
        raw_derivative = (self.error - self.prev_error) / dt
        
        # Apply the smoothing math!
        self.smoothed_derivative = (self.alpha_d * raw_derivative) + ((1.0 - self.alpha_d) * self.smoothed_derivative)
        
        # Multiply Kd by the SMOOTHED derivative, not the raw one
        self.d_term = self.kd * self.smoothed_derivative
        
        # 5. Output
        self.output = self.p_term + self.i_term + self.d_term
        
        # Save state for next loop
        self.prev_error = self.error
        self.last_time = current_time
        
        return self.output