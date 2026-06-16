'''
Distance controller for the APM. Only concerned with maintaining a set distance ahead of the runner.

In normal mode, if the runner catches up, the APM switches back to cruise control mode, following a pacing profile.

Notes: 

- Relies on the rear camera and body tracking module from the ZED SDK
'''

from apm.control.pid_controller import PIDController
from apm.speed_models import SpeedModel
from apm.control.velocity_profiles import LinearRamp as Ramp

class DistanceController:
    '''PID-based motor controller for maintaining a set distance from the runner.'''

    def __init__(self, pid: PIDController, speed_model: SpeedModel, ramp: Ramp):
        self.pid = pid
        self.speed_model = speed_model
        self.ramp = ramp

    def update(self, runner_distance: float, setpoint_distance: float) -> float:
        """Calculate a PWM signal to maintain the setpoint distance from the runner.

        Args:
            runner_distance: Current distance to the runner in meters. (-1 if no runner detected)
            setpoint_distance: Desired distance to maintain from the runner in meters."""

        if runner_distance == -1:
            desired_speed = 0.0

            desired_speed = self.pid.update(setpoint_distance, runner_distance)
        
        ramped_speed = self.ramp.update(desired_speed)
        return self.speed_model.speed_to_pwm(ramped_speed)

