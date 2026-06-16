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

        # Last-computed intermediates, exposed for telemetry (refreshed each update()).
        self.last_desired_speed = 0.0
        self.last_ramped_speed = 0.0

    def update(self, runner_distance: float, setpoint_distance: float) -> float:
        """Calculate a PWM signal to maintain the setpoint distance from the runner.

        Args:
            runner_distance: Current distance to the runner in meters. (-1 if no runner detected)
            setpoint_distance: Desired distance to maintain from the runner in meters."""

        if runner_distance == -1:
            # No runner in view: command a stop and leave the PID untouched (not updated),
            # so its integral freezes rather than winding up and we resume the learned pace
            # smoothly when the runner reappears. (The mode also drops the run flag here.)
            desired_speed = 0.0
        else:
            # error = setpoint - runner_distance: runner closer than setpoint -> speed up.
            desired_speed = self.pid.update(setpoint_distance, runner_distance)
            desired_speed = max(0.0, desired_speed)  # never command reverse toward the runner

        ramped_speed = self.ramp.update(desired_speed)
        self.last_desired_speed = desired_speed
        self.last_ramped_speed = ramped_speed
        return self.speed_model.speed_to_pwm(ramped_speed)

