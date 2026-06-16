'''
Main loop of the Front Camera Test mode.

Starts the front camera and runs the lane detector on each frame.
Encodes JPEG with lane overlay and makes it visible for the orchestrator -> web UI via the set_front_image callback. 
Logs detected lane lines periodically and tracks angle stability over a rolling window to quantify how "jumpy" the detector is.
'''

import time
import logging
import threading
from collections import deque
from typing import Callable

from tomlkit import TOMLDocument
import numpy as np

from apm.drivers.camera import CameraDriver
from apm.vision.lane_detector import LaneDetector
from apm.vision.frame_encoder import encode_lane_keeping_jpeg

log = logging.getLogger(__name__)

_LOG_INTERVAL      = 1.0   # Periodic lane-line status log [s]
_STABILITY_WINDOW  = 30    # Number of frames to include in jumpiness stats
_STABILITY_INTERVAL = 10.0 # How often to log jumpiness stats [s]


def camera_test_front(
    camera: CameraDriver,
    stop_event: threading.Event,
    set_front_image: Callable[[bytes], None], # Lambda to float the JPEG bytes back to the orchestrator -> web UI
    cfg: TOMLDocument
) -> None:

    cfg_detector = cfg["lane_detector"]
    detector = LaneDetector(
        cut_top         = float(cfg_detector["cut_top"]),
        angle_threshold = float(cfg_detector["angle_threshold"]),
        hough_threshold = int(cfg_detector["hough_threshold"]),
        canny_sigma     = float(cfg_detector["canny_sigma"]),
        mask_polygons   = cfg_detector["mask_polygons"],
    )

    slope_history: deque[tuple[float, float]] = deque(maxlen=_STABILITY_WINDOW)  # (left_slope, right_slope)
    last_log_time      = 0.0
    last_stability_log = 0.0
    lanes_present      = False

    while not stop_event.is_set():
        now  = time.monotonic()
        snap = camera.get_snapshot()

        if snap is not None and snap.image is not None:
            img_bgr          = snap.image[:, :, :3]
            img_resized = detector.resize_image(img_bgr)
            lanes            = detector.detect(img_bgr)

            # Log state transitions immediately
            if lanes is not None and not lanes_present:
                log.info(
                    'Lane lines acquired  '
                    f'left_slope={lanes.left_slope:.3f}  '
                    f'right_slope={lanes.right_slope:.3f}'
                )
                lanes_present = True
            elif lanes is None and lanes_present:
                log.info('Lane lines lost')
                lanes_present = False

            # Rolling slope history (only when both lines are found)
            if lanes is not None:
                slope_history.append((lanes.left_slope, lanes.right_slope))

            # Periodic status log
            if now - last_log_time >= _LOG_INTERVAL:
                if lanes is not None:
                    log.info(
                        f'Lanes detected  '
                        f'left_slope={lanes.left_slope:.3f}  left_intercept={lanes.left_intercept:.1f}  '
                        f'right_slope={lanes.right_slope:.3f}  right_intercept={lanes.right_intercept:.1f}'
                    )
                else:
                    log.info('No lane lines detected')
                last_log_time = now

            # Periodic jumpiness report
            if now - last_stability_log >= _STABILITY_INTERVAL and len(slope_history) >= 2:
                arr = np.array(slope_history)
                log.info(
                    f'Slope stability over last {len(slope_history)} frames  '
                    f'left_std={arr[:, 0].std():.4f}  right_std={arr[:, 1].std():.4f}  '
                    f'left_range={arr[:, 0].ptp():.4f}  right_range={arr[:, 1].ptp():.4f}'
                )
                last_stability_log = now

            # Encode and publish the annotated frame
            jpg = encode_lane_keeping_jpeg(
                img_resized,
                lane_lines=lanes,
                mask_polygons=detector.mask_polygons,
            )
            set_front_image(jpg)

        stop_event.wait(1 / camera.fps)

    log.info('Front camera test complete.')
