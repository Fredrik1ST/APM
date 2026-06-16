'''
Main loop of the Front Camera Test mode.

Starts the front camera and runs the lane detector on each frame.
Encodes JPEG with lane overlay and makes it visible for the orchestrator -> web UI via the set_front_image callback. 
Logs detected lane lines periodically and tracks angle stability over a rolling window to quantify how "jumpy" the detector is.
'''

import time
import logging
import threading
import contextlib
from collections import deque
from typing import Callable

from tomlkit import TOMLDocument
import numpy as np

from apm.drivers.camera import CameraDriver
from apm.telemetry import TelemetryLogger
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
    cfg: TOMLDocument,
    telemetry: TelemetryLogger | None = None,  # Inject a shared logger (e.g. from camera_test); else own one
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
    last_fps_seq       = 0       # camera.frame_seq at the previous FPS report
    lanes_present      = False

    # The driver records per-frame timing to the 'camera_front' stream while attached.
    # Use the injected logger if given (caller owns its lifecycle); otherwise own one.
    own_logger = telemetry is None
    tlm = telemetry if telemetry is not None else TelemetryLogger('camera_test_front')
    with (tlm if own_logger else contextlib.nullcontext(tlm)):
        camera.set_telemetry(tlm)
        try:
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

                    # Periodic status + achieved-FPS log. FPS is measured from the driver's
                    # frame counter (true grab rate), not this loop's poll rate.
                    if now - last_log_time >= _LOG_INTERVAL:
                        achieved_fps = (camera.frame_seq - last_fps_seq) / (now - last_log_time) if last_log_time else 0.0
                        if lanes is not None:
                            lane_status = (
                                f'Lanes detected  '
                                f'left_slope={lanes.left_slope:.3f}  left_intercept={lanes.left_intercept:.1f}  '
                                f'right_slope={lanes.right_slope:.3f}  right_intercept={lanes.right_intercept:.1f}'
                            )
                        else:
                            lane_status = 'No lane lines detected'
                        log.info(f'{lane_status}  |  {achieved_fps:.1f}/{camera.fps} FPS  failures={camera.grab_failures}')
                        last_log_time = now
                        last_fps_seq = camera.frame_seq

                    # Periodic jumpiness report
                    if now - last_stability_log >= _STABILITY_INTERVAL and len(slope_history) >= 2:
                        arr = np.array(slope_history)
                        log.info(
                            f'Slope stability over last {len(slope_history)} frames  '
                            f'left_std={arr[:, 0].std():.4f}  right_std={arr[:, 1].std():.4f}  '
                            f'left_range={np.ptp(arr[:, 0]):.4f}  right_range={np.ptp(arr[:, 1]):.4f}'
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
        finally:
            camera.set_telemetry(None)

    log.info('Front camera test complete.')
