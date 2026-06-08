'''
The line detector looks at the front camera feed and detects lane lines.

These lines can be used to find the vanishing point (where the lines intersect)
and lane center (closest midpoint between the two lines), which are used by the
lane keeper controller to calculate the desired steering angle.

Functions are adapted from the line_detection.py module from the 2024 master's thesis 
"Development of an Autonomous Rabbit for Running on a Track" by Kvamme et al. 
'''

import logging
import cv2
import numpy as np
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class LaneLines:
    """Detected left and right lane lines in slope-intercept form: y = slope*x + intercept."""
    left_slope: float
    left_intercept: float
    right_slope: float
    right_intercept: float


class LineDetector:
    """Detects left and right lane lines in a BGR front camera image using the Hough transform.

    Lines are classified as left or right based on their angle: lines within
    `angle_threshold` degrees of vertical are candidates. The final left/right
    line is computed as the median of all candidate rho/theta values.
    """

    DEFAULT_MASKS = [
        [[330, 172], [340, 399], [591, 399], [444, 172]] # Battery box in cropped resolution
        ] 

    def __init__(
        self,
        angle_threshold: float = 60,
        hough_threshold: int = 90,
        mask_polygons: list | None = DEFAULT_MASKS,
        cut_top: float = 4/7,
        output_width: int = 640
    ):
        """
        Args:
            angle_threshold: Max degrees from vertical for a line to be considered a lane line.
            hough_threshold: Minimum Hough accumulator votes to accept a line.
            mask_polygons: Optional list of polygons, each a list of [x, y] vertices, defining
                           regions to black out before edge detection (e.g. vehicle body, battery
                           box). Coordinates must match the preprocessed image resolution.
                           [0, 0] is the top-left corner.
            cut_top: Fraction of the image height to discard from the top before detection (0.0-1.0).
                     Use this to exclude distant scenery that would produce false lane lines.
            output_width: Width in pixels to resize the (cropped) image to before detection.
                          Height is scaled proportionally to preserve the cropped aspect ratio.
        """
        self.angle_threshold = angle_threshold
        self.hough_threshold = hough_threshold
        self.mask_polygons = mask_polygons
        self.cut_top = cut_top
        self.output_width = output_width


    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """Crop the top of the image and resize to a standard width.

        Removes the top `cut_top` fraction of the image to only include the ROI (the track),
        then resizes to `output_width` pixels wide (height scaled proportionally).

        Note: LaneLines returned by detect() are in the preprocessed image's coordinates. 
        Pass the preprocessed image's dimensions to LaneKeeper.update(), not the original ones.
        """
        height, width = image.shape[:2]
        cropped = image[int(height * self.cut_top):, :]
        cropped_height = cropped.shape[0]
        target_height = int(cropped_height / width * self.output_width)
        return cv2.resize(cropped, (self.output_width, target_height), interpolation=cv2.INTER_AREA)


    def detect(self, image: np.ndarray) -> LaneLines | None:
        """Detect left and right lane lines in the given BGR image.

        Applies preprocessing (crop + resize) before detection if cut_top > 0.
        Returns LaneLines with slope/intercept coefficients, or None if both
        lines cannot be found.
        """
        if self.cut_top > 0.0:
            image = self.preprocess(image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = self._canny_with_mask(gray)

        raw_lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=self.hough_threshold)
        if raw_lines is None:
            logger.debug("No Hough lines found")
            return None

        threshold_rad = self.angle_threshold * np.pi / 180
        left_rho_theta = []
        right_rho_theta = []

        for line in raw_lines:
            rho, theta = line[0]
            if theta < threshold_rad:
                left_rho_theta.append((rho, theta))
            elif theta > (np.pi - threshold_rad):
                right_rho_theta.append((rho, theta))

        if not (left_rho_theta and right_rho_theta):
            logger.debug("Incomplete lane: found %d left, %d right candidates", len(left_rho_theta), len(right_rho_theta))
            return None

        left_arr = np.array(left_rho_theta)
        right_arr = np.array(right_rho_theta)

        a1, b1 = self._hough_to_slope_intercept(np.median(left_arr[:, 0]), np.median(left_arr[:, 1]))
        a2, b2 = self._hough_to_slope_intercept(np.median(right_arr[:, 0]), np.median(right_arr[:, 1]))

        return LaneLines(left_slope=a1, left_intercept=b1, right_slope=a2, right_intercept=b2)


    def _canny_with_mask(self, gray: np.ndarray, sigma: float = 0.33) -> np.ndarray:
        """Apply automatic Canny edge detection with a mask to remove unwanted edges (e.g. battery box)."""
        v = np.median(gray) * 2.5 # Higher multiplier = filters out more edges
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edges = cv2.Canny(gray, lower, upper)

        if self.mask_polygons:
            mask = np.ones_like(edges) * 255
            for polygon in self.mask_polygons:
                cv2.fillPoly(mask, [np.array(polygon, dtype=np.int32)], 0)
            edges = cv2.bitwise_and(edges, mask)

        return edges


    def _hough_to_slope_intercept(self, rho: float, theta: float) -> tuple:
        """Convert a Hough line (rho, theta) to slope-intercept form (a, b) where y = ax + b.

        The Hough normal form is: rho = x * cos(theta) + y * sin(theta)
        Solving for y gives: y = (rho/sin(theta)) + (-cos(theta)/sin(theta)) * x
        So: a = -cos(theta)/sin(theta),  b = rho/sin(theta)
        """
        sin_theta = np.sin(theta)
        if abs(sin_theta) < 1e-6:
            sin_theta = 1e-6  # near-vertical line: avoid division by zero
        a = -np.cos(theta) / sin_theta
        b = rho / sin_theta
        return a, b
