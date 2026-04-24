import time

class PIDController:
    def __init__(self, kp, ki, kd, alpha_d=0.20):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # alpha_d controls how much of the NEW derivative reading we trust vs the OLD.
        # Lower = smoother but more delayed. 1.0 = no smoothing.
        self.alpha_d = alpha_d
        
        self.prev_error = 0.0
        self.prev_measured_value = 0.0
        self.integral = 0.0
        self.smoothed_derivative = 0.0
        self.last_time = time.time()
        self.first_pass = True  
        
        # Telemetry
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.error = 0.0
        self.output = 0.0
        
    def reset(self):
        """Call this right before starting the balance loop"""
        self.prev_error = 0.0
        self.prev_measured_value = 0.0
        self.integral = 0.0
        self.smoothed_derivative = 0.0
        self.last_time = time.time()
        self.first_pass = True  
        
    def compute(self, setpoint, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return 0.0 

        self.error = setpoint - measured_value
        
        # Prevent startup kick
        if getattr(self, 'first_pass', True):
            self.prev_error = self.error
            self.prev_measured_value = measured_value
            self.first_pass = False

        # 1. Proportional
        self.p_term = self.kp * self.error
        
        # 2. Integral (Clamped to prevent windup)
        # Note: 40.0 is a hardcoded limit. Make sure this makes sense for your servo output limits.
        self.integral += self.error * dt
        self.integral = max(min(self.integral, 40.0), -40.0) 
        self.i_term = self.ki * self.integral
        
        # 3. Derivative (ON MEASUREMENT + FILTERED)
        # Calculate raw change in physical robot movement, ignoring setpoint changes
        raw_derivative = (measured_value - self.prev_measured_value) / dt
        
        # Apply the low-pass filter specifically to the noisy derivative
        self.smoothed_derivative = (self.alpha_d * raw_derivative) + ((1.0 - self.alpha_d) * self.smoothed_derivative)
        
        # D-term is NEGATIVE because we are taking the derivative of the measurement, not the error!
        self.d_term = -self.kd * self.smoothed_derivative
        
        # 4. Output
        self.output = self.p_term + self.i_term + self.d_term
        
        # Save state for next loop
        self.prev_error = self.error
        self.prev_measured_value = measured_value
        self.last_time = current_time
        
        return self.output