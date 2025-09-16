class GBController:
    def __init__(self, kp, ki, kd, setpoint, gamma, beta):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.gamma = gamma
        self.beta = beta

    def compute(self, current_value):
        error = self.setpoint - current_value
        p_error = (self.beta * self.setpoint) - current_value
        d_error = (self.gamma * self.setpoint) - current_value

        # Proportional term
        P = self.kp * p_error

        # Integral term
        self.integral += error
        I = self.ki * self.integral

        # Derivative term
        D = self.kd * (d_error - self.prev_error)

        # Compute the control signal
        control_signal = P + I + D
        #print("setpoint", self.setpoint)
        #print("PID", P, I, D)

        # Update the previous error for the next iteration
        self.prev_error = error

        return control_signal
    