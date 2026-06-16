'''
Main loop of the Lane Keeper Test mode.

Starts the front camera and runs the lane detector on each frame.
Feeds detected lane lines into the LaneKeeper controller and sends the resulting steering angle to the Arduino. 
Speed is fixed at zero. This mode is for steering tuning only.
Encodes JPEG with lane overlay and makes it visible for the orchestrator -> web UI via the set_front_image callback.
Logs detected lane lines periodically and tracks angle stability over a rolling window.
'''

import time
import logging
import threading
from collections import deque
from typing import Callable

from tomlkit import TOMLDocument
import numpy as np

from apm.drivers.arduino import ArduinoDriver, MessageCommands
from apm.drivers.camera import CameraDriver
from apm.vision.lane_detector import LaneDetector
from apm.vision.frame_encoder import encode_lane_keeping_jpeg
from apm.control.lane_keeper import LaneKeeper
from apm.control.pid_controller import PIDController
from apm.telemetry import TelemetryLogger

log = logging.getLogger(__name__)

_LOG_INTERVAL      = 1.0   # Periodic lane-line status log [s]
_STABILITY_WINDOW  = 30    # Number of frames to include in jumpiness stats
_STABILITY_INTERVAL = 10.0 # How often to log jumpiness stats [s]


def lane_keeper_test(
    camera: CameraDriver,
    arduino: ArduinoDriver,
    stop_event: threading.Event,
    cfg: TOMLDocument,
    set_front_image: Callable[[bytes], None],
) -> None:

    # Read lane detector parameters from config
    cfg_detector = cfg["lane_detector"]
    detector = LaneDetector(
        cut_top         = float(cfg_detector["cut_top"]),
        angle_threshold = float(cfg_detector["angle_threshold"]),
        hough_threshold = int(cfg_detector["hough_threshold"]),
        canny_sigma     = float(cfg_detector["canny_sigma"]),
        mask_polygons   = cfg_detector["mask_polygons"],
    )

    # Read Lane Keeper PID parameters from config
    cfg_angle = cfg["lane_keeping"]["pid"]["angle"]
    cfg_pos   = cfg["lane_keeping"]["pid"]["position"]
    lane_keeper = LaneKeeper(
        angle_pid=PIDController(
            kp=float(cfg_angle["kp"]),
            ki=float(cfg_angle["ki"]),
            kd=float(cfg_angle["kd"]),
            beta=float(cfg_angle["beta"]),
            gamma=float(cfg_angle["gamma"]),
        ),
        position_pid=PIDController(
            kp=float(cfg_pos["kp"]),
            ki=float(cfg_pos["ki"]),
            kd=float(cfg_pos["kd"]),
            beta=float(cfg_pos["beta"]),
            gamma=float(cfg_pos["gamma"]),
        ),
    )

    slope_history: deque[tuple[float, float]] = deque(maxlen=_STABILITY_WINDOW)
    last_log_time      = 0.0
    last_stability_log = 0.0
    lanes_present      = False

    # Telemetry: per-tick heading/position setpoints vs measured values and the steering
    # output (+ PID term breakdown), so the lane keeper's tracking can be plotted/tuned.
    with TelemetryLogger('lane_keeper_test') as tlm:
        try:
            while not stop_event.is_set():
                now  = time.monotonic()
                snap = camera.get_snapshot()

                if snap is not None and snap.image is not None:
                    img_bgr     = snap.image[:, :, :3]
                    img_resized = detector.resize_image(img_bgr)
                    lanes       = detector.detect(img_bgr)

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

                    if lanes is not None:
                        slope_history.append((lanes.left_slope, lanes.right_slope))

                        steering = lane_keeper.update(lanes, image_width=img_resized.shape[1])
                        steering = float(np.clip(steering, 0.0, 180.0))
                        arduino.write_msg(MessageCommands(run=True, steer_angle=steering))
                        log.debug(f'Steering angle: {steering:.1f}°')
                    else:
                        # Hold straight when no lanes are visible
                        steering = 90.0
                        arduino.write_msg(MessageCommands(run=True, steer_angle=steering))

                    tlm.log('lane', {
                        'lanes':       lanes is not None,
                        'heading':     round(lane_keeper.last_heading, 3),
                        'heading_sp':  LaneKeeper.STRAIGHT_HEADING,
                        'x_center':    round(lane_keeper.last_x_center, 2),
                        'x_center_sp': round(lane_keeper.last_image_center, 2),
                        'ang_corr':    round(lane_keeper.last_angle_correction, 3),
                        'ang_p':       round(lane_keeper.angle_pid.p, 3),
                        'ang_i':       round(lane_keeper.angle_pid.i, 3),
                        'ang_d':       round(lane_keeper.angle_pid.d, 3),
                        'pos_corr':    round(lane_keeper.last_position_correction, 3),
                        'pos_p':       round(lane_keeper.position_pid.p, 3),
                        'pos_i':       round(lane_keeper.position_pid.i, 3),
                        'pos_d':       round(lane_keeper.position_pid.d, 3),
                        'steering':    round(steering, 2),
                    })

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
            # Safe stop on exit
            arduino.write_msg(MessageCommands(run=False, steer_angle=90.0))

    log.info('Lane keeper test complete.')
