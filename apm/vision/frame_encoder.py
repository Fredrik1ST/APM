''' Frame encoders for converting BGRA camera frames to common image formats.'''

import cv2
import numpy as np
from apm.drivers.camera import CameraSnapshot
from apm.vision.line_detector import LaneLines

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
    mask_polygon: list | None = None,
    quality: int = 80,
    scale: float = 1.0,
) -> bytes:
    '''Draw lane lines, steering angle, and mask polygon over the image, then encode as JPEG.

    Lane lines are drawn in green. The steering angle is shown as an arrow from
    the bottom-center of the image (straight up = 90°/straight, tilted right = turning right).
    The mask polygon is drawn in red to show the masked-out region.
    '''
    overlay = image.copy()
    h, w = overlay.shape[:2]

    if mask_polygon is not None:
        vertices = np.array(mask_polygon, dtype=np.int32)
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

    if steering_angle is not None:
        arrow_len = h // 5
        base_x, base_y = w // 2, h - 10
        deviation = np.radians(steering_angle - 90.0)
        tip_x = int(base_x + arrow_len * np.sin(deviation))
        tip_y = int(base_y - arrow_len * np.cos(deviation))
        cv2.arrowedLine(overlay, (base_x, base_y), (tip_x, tip_y), (0, 220, 255, 255), 2, tipLength=0.2)
        cv2.putText(overlay, f'{steering_angle:.1f} deg', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255, 255), 2)

    return encode_jpeg(overlay, quality=quality, scale=scale)
