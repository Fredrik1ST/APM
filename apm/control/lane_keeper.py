'''
Continually updates steering angle to keep the APM centered in the lane.

Relies on detected lines from the line detector (see the vision package) to find their vanishing point,
then calculates the angle between the vanishing point and the bottom center of the image.

Also calculates the lane center of the current frame to detect offset.

Combined, these two metrics are used to calculate the desired steering angle to keep the
APM centered in the lane

'''

import logging
import math

from apm.vision.line_detector import LaneLines
from apm.control.pid_controller import PIDController

logger = logging.getLogger(__name__)


class LaneKeeper:
    """Generates a steering angle to keep the APM centered in its lane.

    Uses two PID controllers:
      - angle_pid:    corrects for the heading angle toward the lane's vanishing point.
      - position_pid: corrects for lateral offset from the lane center at the bottom of the frame.

    Angle convention: atan2(vanishing_y - image_bottom, vanishing_x - image_center_x).
    When heading straight the vanishing point is directly above, giving -90°.
    The raw steering output is offset by +90 so the final value is in [0°, 180°] with
    90° meaning straight ahead.
    """

    STRAIGHT_HEADING = -90.0  # atan2 heading (degrees) when the APM is aligned straight

    def __init__(self, angle_pid: PIDController, position_pid: PIDController):
        self.angle_pid = angle_pid
        self.position_pid = position_pid

    def update(self, lane_lines: LaneLines, image_width: int, image_height: int) -> float:
        """Calculate a steering angle from the detected lane lines.

        Args:
            lane_lines:   Detected lane lines from LineDetector.
            image_width:  Width of the source image in pixels.
            image_height: Height of the source image in pixels.

        Returns:
            Steering angle in degrees where 90° = straight, <90° = left, >90° = right.
        """
        vanishing_x, vanishing_y = self._vanishing_point(lane_lines)
        x_center = self._lane_center_at_bottom(lane_lines, image_height)

        heading = math.degrees(
            math.atan2(vanishing_y - image_height, vanishing_x - image_width / 2)
        )

        angle_correction = self.angle_pid.update(
            setpoint=self.STRAIGHT_HEADING, current_value=heading
        )
        position_correction = self.position_pid.update(
            setpoint=image_width / 2, current_value=x_center
        )

        steering = angle_correction + position_correction + 90.0
        logger.debug("heading=%.1f°  x_center=%.1f  angle_corr=%.2f  pos_corr=%.2f  steering=%.1f°",
                     heading, x_center, angle_correction, position_correction, steering)
        return steering

    def _vanishing_point(self, lines: LaneLines) -> tuple:
        """Intersection of the two lane lines (the vanishing point)."""
        a1, b1 = lines.left_slope, lines.left_intercept
        a2, b2 = lines.right_slope, lines.right_intercept
        if a1 == a2:
            raise ValueError("Lane lines are parallel - vanishing point is at infinity.")
        x = (b2 - b1) / (a1 - a2)
        y = a1 * x + b1
        return x, y

    def _lane_center_at_bottom(self, lines: LaneLines, image_height: int) -> float:
        """X coordinate of the lane midpoint at the bottom row of the image."""
        a1, b1 = lines.left_slope, lines.left_intercept
        a2, b2 = lines.right_slope, lines.right_intercept
        if abs(a1) < 1e-6 or abs(a2) < 1e-6:
            side = "Left" if abs(a1) < 1e-6 else "Right"
            raise ValueError(f" {side} lane line is near-horizontal - cannot compute lane center.")
        x1 = (image_height - b1) / a1
        x2 = (image_height - b2) / a2
        return (x1 + x2) / 2
