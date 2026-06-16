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

    Angle convention: 
        atan2(vanishing_y - image_bottom, vanishing_x - image_center_x).
        When heading straight the vanishing point is directly above, giving -90°.
        Raw steering output is offset by +90 so the final value is in [0°, 180°],
        where 0° = Right, 90° = Straight, 180° = Left
    """

    STRAIGHT_HEADING = -90.0  # atan2 heading (degrees) when the APM is aligned straight

    def __init__(self, angle_pid: PIDController, position_pid: PIDController):
        self.angle_pid = angle_pid
        self.position_pid = position_pid

    def update(self, lane_lines: LaneLines, image_width: int) -> float:
        """Calculate a steering angle from the detected lane lines.

        Args:
            lane_lines:   Detected lane lines from LineDetector.
            Includes image dimensions and getters for vanishing point, lane center and heading.
            image_width:  Width of the source image in pixels.
            image_height: Height of the source image in pixels.

        Returns:
            Steering angle in degrees where 90° = straight, <90° = left, >90° = right.
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

        steering = angle_correction + position_correction + 90.0
        logger.debug("heading=%.1f°  x_center=%.1f  angle_corr=%.2f  pos_corr=%.2f  steering=%.1f°",
                     heading, x_center, angle_correction, position_correction, steering)
        return steering