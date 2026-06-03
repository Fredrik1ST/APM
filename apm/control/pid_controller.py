import time

class PIDController:
    ''' A simple PID controller for controlling a single variable based on its setpoint and current value.'''

    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, enable_dt = False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
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
            dt = now - self.last_update_time
            self.last_update_time = now
        else:
            dt = 1.0

        error = setpoint - current_value

        # Proportional term - proportional to the current error (provides immediate correction)
        P = self.kp * error 

        # Integral term - proportional to accumulated error over time (eliminates steady-state error)
        self.integral += error * dt
        I = self.ki * self.integral

        # Derivative term - proportional to the error's rate of change (acts as a damper to prevent overshoot)
        D = self.kd * (error - self.previous_error) / dt if dt > 0 else 0

        output = P + I + D
        self.previous_error = error
        return output