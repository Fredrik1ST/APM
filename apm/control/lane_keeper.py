'''
Continually updates steering angle to keep the APM centered in the lane.

Relies on detected lines from the lane detector (see the vision package) to find their vanishing point,
then calculates the angle between the vanishing point and the bottom center of the image.

Also calculates the lane center of the current frame to detect offset.

Combined, these two metrics are used to calculate the desired steering angle to keep the
APM centered in the lane

'''

import logging
import math

from apm.vision.lane_detector import LaneLines
from apm.control.pid_controller import PIDController

logger = logging.getLogger(__name__)


class LaneKeeper:
    """Generates a steering angle to keep the APM centered in its lane.

    Uses two PID controllers:
      - angle_pid:    corrects for the heading angle toward the lane's vanishing point.
      - position_pid: corrects for lateral offset from the lane center at the bottom of the frame.

    Two different angle scales are involved here; do not confuse them:

    1. HEADING scale (input, from LaneLines.get_heading()):
         atan2(vanishing_y - image_bottom, vanishing_x - image_center_x), so it is
         centred on -90° (vanishing point straight above), NOT on 0°.
           -90°            = aligned straight ahead   (STRAIGHT_HEADING)
           toward   0°     = vanishing point is to the RIGHT of centre
           toward -180°    = vanishing point is to the LEFT  of centre

    2. STEERING scale (output, the Arduino servo command):
         The PID corrections are computed around 0° (their natural error scale) and
         then offset by +90 to land on the Arduino's [0°, 180°] servo scale:
           90°             = straight
           < 90° (toward 0)   = steer RIGHT   (servo limit max_right = 45°)
           > 90° (toward 180) = steer LEFT    (servo limit max_left  = 135°)
         This matches [arduino.steer_angle] in config/settings.toml.

    Sign/feedback check: if the lane bends right, the vanishing point moves right,
    heading rises from -90° toward 0°, the (positive-kp) angle PID drives the output
    below 90°, i.e. steer right to follow the lane -- negative feedback, as required.
    """

    # Heading (scale 1 above) the angle PID drives toward, i.e. the heading value
    # that corresponds to driving straight. Not 0° -- see the class docstring.
    STRAIGHT_HEADING = -90.0

    def __init__(self, angle_pid: PIDController, position_pid: PIDController):
        self.angle_pid = angle_pid
        self.position_pid = position_pid

        # Last-computed intermediates, exposed for telemetry (refreshed each update()).
        self.last_heading = 0.0
        self.last_x_center = 0.0
        self.last_image_center = 0.0
        self.last_angle_correction = 0.0
        self.last_position_correction = 0.0
        self.last_steering = 90.0

    def update(self, lane_lines: LaneLines, image_width: int) -> float:
        """Calculate a steering angle from the detected lane lines.

        Args:
            lane_lines:   Detected lane lines from LineDetector.
            Includes image dimensions and getters for vanishing point, lane center and heading.
            image_width:  Width of the source image in pixels.
            image_height: Height of the source image in pixels.

        Returns:
            Steering angle on the Arduino servo scale (see class docstring): 90° = straight,
            <90° = right, >90° = left. Caller clamps this to the configured servo limits.
        """

        image_center = lane_lines.image_width / 2
        x_center = lane_lines.get_lane_center_at_bottom()
        heading = lane_lines.get_heading()

        angle_correction = self.angle_pid.update(
            setpoint=self.STRAIGHT_HEADING, current_value=heading
        )
        
        position_correction = self.position_pid.update(
            setpoint=image_center, current_value = x_center
        )

        # Both corrections are errors centred on 0 (heading vs -90°, x_center vs image
        # centre). The +90 shifts them onto the Arduino servo scale where 90° = straight.
        steering = angle_correction + position_correction + 90.0
        logger.debug("heading=%.1f°  x_center=%.1f  angle_corr=%.2f  pos_corr=%.2f  steering=%.1f°",
                     heading, x_center, angle_correction, position_correction, steering)

        # Expose intermediates for telemetry / tuning.
        self.last_heading = heading
        self.last_x_center = x_center
        self.last_image_center = image_center
        self.last_angle_correction = angle_correction
        self.last_position_correction = position_correction
        self.last_steering = steering
        return steering