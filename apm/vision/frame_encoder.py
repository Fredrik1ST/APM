''' Frame encoders for converting BGRA camera frames to common image formats.'''

import cv2
import numpy as np
from apm.drivers.camera import CameraSnapshot
from apm.vision.lane_detector import LaneLines

def encode_jpeg(image: np.ndarray, quality: int = 80, scale: float = 1.0, display: bool = False) -> bytes:
    '''Convert a BGRA image to JPEG bytes with specified quality and scale.'''
    bgr = image[:, :, :3]  # Drop alpha channel (BGRA -> BGR) for OpenCV
    if scale != 1.0:
        bgr = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('JPEG encode failed')
    if display:
        cv2.imshow('Encoded JPEG', buf)
    return buf.tobytes()


def encode_lane_keeping_jpeg(
    image: np.ndarray,
    lane_lines: LaneLines | None = None,
    steering_angle: float | None = None,
    heading_angle: float | None = None,
    mask_polygons: list | None = None,
    quality: int = 80,
    scale: float = 1.0,
) -> bytes:
    '''Draw lane lines, steering angle, heading angle, and mask polygons over the image, then encode as JPEG.

    Lane lines are drawn in green.
    Steering angle (yellow): output of the lane keeper controller [0°-180°], 90° = straight.
    Heading angle (cyan): raw atan2 heading from LaneLines.get_heading(), -90° = straight.
    Both are shown as arrows from the bottom-center of the image (straight = pointing up).
    Mask polygons are drawn in red; matches the format used by LaneDetector: [[polygon1], [polygon2], ...].
    '''
    overlay = image.copy()
    h, w = overlay.shape[:2]

    if mask_polygons is not None:
        for polygon in mask_polygons:
            vertices = np.array(polygon, dtype=np.int32)
            cv2.polylines(overlay, [vertices], isClosed=True, color=(0, 0, 200, 255), thickness=2)

    if lane_lines is not None:
        for slope, intercept in (
            (lane_lines.left_slope, lane_lines.left_intercept),
            (lane_lines.right_slope, lane_lines.right_intercept),
        ):
            if abs(slope) > 1e-6:
                x_top = int(-intercept / slope)
                x_bot = int((h - 1 - intercept) / slope)
                cv2.line(overlay, (x_top, 0), (x_bot, h - 1), (0, 200, 0, 255), 2)

    arrow_len = h // 5
    base_x, base_y = w // 2, h - 10

    if heading_angle is not None:
        # heading_angle uses atan2 convention: -90° = straight. Convert to [0°, 180°] for display.
        display_heading = heading_angle + 180.0
        deviation = np.radians(display_heading - 90.0)
        tip_x = int(base_x + arrow_len * np.sin(deviation))
        tip_y = int(base_y - arrow_len * np.cos(deviation))
        cv2.arrowedLine(overlay, (base_x, base_y), (tip_x, tip_y), (255, 200, 0, 255), 2, tipLength=0.2)
        cv2.putText(overlay, f'hdg {heading_angle:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0, 255), 2)

    if steering_angle is not None:
        deviation = np.radians(steering_angle - 90.0)
        tip_x = int(base_x + arrow_len * np.sin(deviation))
        tip_y = int(base_y - arrow_len * np.cos(deviation))
        cv2.arrowedLine(overlay, (base_x, base_y), (tip_x, tip_y), (0, 220, 255, 255), 2, tipLength=0.2)
        cv2.putText(overlay, f'steer {steering_angle:.1f}', (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255, 255), 2)

    return encode_jpeg(overlay, quality=quality, scale=scale)
