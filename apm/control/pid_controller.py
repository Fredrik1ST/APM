import time

class PIDController:
    """2 Degrees-of-Freedom PID controller with setpoint weighting for proportional and derivative terms.

    Useful for mitigating setpoint kick, where the controller output spikes with sudden setpoint changes.

    To use as a standard (1DOF) PID, set beta=1 and gamma=1

    Attributes:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        beta: Setpoint weighting for proportional term (0.0 - 1.0)
        gamma: Setpoint weighting for derivative term (0.0 - 1.0)
    """
    
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, beta = 1.0, gamma = 1.0, enable_dt = False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.beta = beta
        self.gamma = gamma
        self.integral = 0
        self.previous_error = 0
        self.previous_update_time = 0.0
        self.enable_dt = enable_dt
        self.last_update_time = 0.0

    def update(self, setpoint: float, current_value: float):
        '''Calculate a new PID control output.
        '''

        # If update rate is not constant, dt should be calculated each iteration to ensure consistent behavior
        if self.enable_dt:
            now = time.monotonic()
            dt = now - self.last_update_time if self.last_update_time > 0 else 0.0
            self.last_update_time = now
        else:
            dt = 1.0

        error = setpoint - current_value
        p_error = (self.beta * setpoint) - current_value    # Proportional error with setpoint weighting
        d_error = (self.gamma * setpoint) - current_value   # Derivative error with setpoint weighting

        # Proportional term - proportional to the current error (provides immediate correction)
        P = self.kp * p_error 

        # Integral term - proportional to accumulated error over time (eliminates steady-state error)
        self.integral += error * dt
        I = self.ki * self.integral

        # Derivative term - proportional to the error's rate of change (acts as a damper to prevent overshoot)
        D = self.kd * (d_error - self.previous_error) / dt if dt > 0 else 0

        output = P + I + D
        self.previous_error = d_error # Store the derivative error!
        return output