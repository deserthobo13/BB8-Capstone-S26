import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def reset(self):
        """Call this right before starting the balance loop to prevent massive dt spikes"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Prevent divide by zero errors if loop runs too fast
        if dt <= 0:
            return 0.0 

        # 1. Calculate Error
        error = setpoint - measured_value
        
        # 2. Proportional
        P = self.kp * error
        
        # 3. Integral (with Anti-Windup)
        # Clamped so it doesn't build up infinitely if the robot is stuck
        self.integral += error * dt
        self.integral = max(min(self.integral, 10.0), -10.0) 
        I = self.ki * self.integral
        
        # 4. Derivative
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative
        
        # 5. Output
        output = P + I + D
        
        # Save state for next loop
        self.prev_error = error
        self.last_time = current_time
        
        return output