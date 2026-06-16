'''
The lane detector looks at the front camera feed and detects lane lines.

These lines can be used to find the vanishing point (where the lines intersect)
and lane center (closest midpoint between the two lines), which are used by the
lane keeper controller to calculate the desired steering angle.

Functions are adapted from the line_detection.py module from the 2024 master's thesis 
"Development of an Autonomous Rabbit for Running on a Track" by Kvamme et al. 
'''

import math
import logging
from dataclasses import dataclass

import cv2
import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class LaneLines:
    """Detected left and right lane lines in slope-intercept form: y = slope*x + intercept.

    methods:
        - get_vanishing_point(): XY-coordinates where the two lane lines intersect (the vanishing point). May be off-screen.
        - get_lane_center_at_bottom(): X coordinate of the lane midpoint at the bottom row of the image.
        - get_heading(): Calculate the heading angle toward the lane's vanishing point.
    """
    left_slope: float
    left_intercept: float
    right_slope: float
    right_intercept: float
    image_height: int
    image_width: int

    def get_vanishing_point(self) -> tuple[float, float]:
        """XY-coordinates where the two lane lines intersect (the vanishing point). May be off-screen."""
        a1, b1 = self.left_slope, self.left_intercept
        a2, b2 = self.right_slope, self.right_intercept
        if a1 == a2:
            raise ValueError("Lane lines are parallel - vanishing point is at infinity.")
        x = (b2 - b1) / (a1 - a2)
        y = a1 * x + b1
        return x, y

    def get_lane_center_at_bottom(self) -> float:
        """X coordinate of the lane midpoint at the bottom row of the image."""
        a1, b1 = self.left_slope, self.left_intercept
        a2, b2 = self.right_slope, self.right_intercept
        side = ""
        if abs(a1) < 1e-6:
            side += " Left "
        if abs(a2) < 1e-6:
            side += " Right "
        if side:
            raise ValueError(f"{side}lane line is near-horizontal - cannot compute lane center.")

        x1 = (self.image_height - b1) / a1
        x2 = (self.image_height - b2) / a2
        return (x1 + x2) / 2

    def get_heading(self) -> float:
        """Calculate the heading angle toward the lane's vanishing point.
        
        Angle convention:
            :code:`atan2(vanishing_y - image_bottom, vanishing_x - image_center_x)`.
            When heading straight the vanishing point is directly above, giving -90°."""
        vanishing_x, vanishing_y = self.get_vanishing_point()
        heading = math.degrees(
            math.atan2(vanishing_y - self.image_height, vanishing_x - self.image_width / 2)
        )
        return heading


class LaneDetector:
    """Detects left and right lane lines in a BGR front camera image using the Hough transform.

    Lines are classified as left or right based on their angle: lines within
    `angle_threshold` degrees of vertical are candidates. The final left/right
    line is computed as the median of all candidate rho/theta values.
    """

    DEFAULT_MASKS = [       # Polygonal masks, based on 4/7 cut-top preprocesssed image
        [[324, 172], [324, 70], [440,69], [610, 172]],  # Mask battery box
        [[257,172], [270, 155], [336, 150], [336, 172]] # Mask car hood
        ]

    def __init__(
        self,
        angle_threshold: float = 60,
        hough_threshold: int = 90,
        canny_sigma: float = 0.33,
        mask_polygons: list | None = DEFAULT_MASKS,
        cut_top: float = 4/7,
        output_width: int = 640,
    ):
        """
        Args:
            angle_threshold: Max degrees from vertical for a line to be considered a lane line.
            hough_threshold: Minimum Hough accumulator votes to accept a line.
            canny_sigma: Controls the width of the auto-threshold range for Canny edge detection.
                         Higher values = wider threshold range = more edges detected.
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
        self.canny_sigma = canny_sigma
        self.mask_polygons = mask_polygons
        self.cut_top = cut_top
        self.output_width = output_width
        self.resized_image = None # Store the last resized image for debugging/visualization
        self.image_canny = None   # Store the last Canny edges (w/mask) for debugging/visualization

    def resize_image(self, image: np.ndarray) -> np.ndarray:
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
        self.resized_image = cv2.resize(cropped, (self.output_width, target_height), interpolation=cv2.INTER_AREA)
        return self.resized_image

    def detect(self, image: np.ndarray) -> LaneLines | None:
        """Detect left and right lane lines in the given BGR image.
                
        Returns LaneLines with slope/intercept coefficients, or None if both
        lines cannot be found.

        NB! Resize the input image first with resize_image() if needed (cuts top of image and applies masks).
        """

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = self.canny_with_mask(gray)
        self.image_canny = edges # Store for debugging/visualization

        raw_lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=self.hough_threshold)
        if raw_lines is None:
            logger.debug("No Hough lines found (threshold=%d, image=%dx%d)",
                         self.hough_threshold, image.shape[1], image.shape[0])
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
            logger.debug("Incomplete lane: %d left, %d right candidates after angle filter "
                         "(threshold=%d°, hough_threshold=%d, image=%dx%d)",
                         len(left_rho_theta), len(right_rho_theta),
                         self.angle_threshold, self.hough_threshold,
                         image.shape[1], image.shape[0])
            return None

        left_arr = np.array(left_rho_theta)
        right_arr = np.array(right_rho_theta)

        a1, b1 = self.hough_to_slope_intercept(np.median(left_arr[:, 0]), np.median(left_arr[:, 1]))
        a2, b2 = self.hough_to_slope_intercept(np.median(right_arr[:, 0]), np.median(right_arr[:, 1]))

        return LaneLines(left_slope=a1, left_intercept=b1, right_slope=a2, right_intercept=b2, image_height=image.shape[0], image_width=image.shape[1])


    def canny_with_mask(self, gray: np.ndarray) -> np.ndarray:
        """Apply automatic Canny edge detection with a mask to remove unwanted edges (e.g. battery box)."""
        v = np.median(gray) * 2.5 # Higher multiplier = filters out more edges
        lower = int(max(0, (1.0 - self.canny_sigma) * v))
        upper = int(min(255, (1.0 + self.canny_sigma) * v))
        edges = cv2.Canny(gray, lower, upper)

        if self.mask_polygons:
            mask = np.ones_like(edges) * 255
            for polygon in self.mask_polygons:
                cv2.fillPoly(mask, [np.array(polygon, dtype=np.int32)], 0)
            edges = cv2.bitwise_and(edges, mask)

        return edges


    def hough_to_slope_intercept(self, rho: float, theta: float) -> tuple:
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
