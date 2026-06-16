'''
Main loop of the Back Camera Test mode.

Starts the back camera with body tracking enabled and runs until the user stops it.
Logs periodically whether a runner is detected and their distance, and logs immediately
when the runner appears or disappears from the camera feed.
'''

import cv2
import time
import threading
import logging
import contextlib

import tomlkit

from apm.drivers.camera import CameraDriver
from apm.telemetry import TelemetryLogger

log = logging.getLogger(__name__)

_LOG_INTERVAL = 1.0  # How often to print a detection status reading [s]


def camera_test_back(camera: CameraDriver, stop_event: threading.Event,
                          cfg: tomlkit.TOMLDocument,
                          telemetry: TelemetryLogger | None = None,  # Inject a shared logger; else own one
                          show_window: bool = True) -> None:         # cv2 window (off when run combined)

    last_log_time = 0.0
    last_fps_seq = 0           # camera.frame_seq at the previous FPS report
    runner_present = False     # Tracks whether a runner was detected in the previous frame

    # The driver records per-frame timing to the 'camera_back' stream while attached.
    # Use the injected logger if given (caller owns its lifecycle); otherwise own one.
    own_logger = telemetry is None
    tlm = telemetry if telemetry is not None else TelemetryLogger('camera_test_back')
    with (tlm if own_logger else contextlib.nullcontext(tlm)):
        camera.set_telemetry(tlm)
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                snap = camera.get_snapshot()

                if snap is not None:
                    detected = len(snap.bodies) > 0

                    # Log state transitions immediately
                    if detected and not runner_present:
                        distance = snap.bodies[0].position[2]
                        log.info(f'Runner detected - Distance: {distance:.2f} m')
                        runner_present = True
                    elif not detected and runner_present:
                        log.info('Runner lost - No bodies in frame')
                        runner_present = False

                    # Periodic status + achieved-FPS log. FPS is measured from the driver's
                    # frame counter (true grab rate), not this loop's poll rate.
                    if now - last_log_time >= _LOG_INTERVAL:
                        achieved_fps = (camera.frame_seq - last_fps_seq) / (now - last_log_time) if last_log_time else 0.0
                        runner = f'Runner present - Distance: {snap.bodies[0].position[2]:.2f} m' if detected else 'No runner detected'
                        log.info(f'{runner}  |  {achieved_fps:.1f}/{camera.fps} FPS  failures={camera.grab_failures}')
                        last_log_time = now
                        last_fps_seq = camera.frame_seq

                    if show_window:
                        cv2.imshow('Back Camera', snap.image)

                stop_event.wait(1 / camera.fps) # Sleep until next frame is expected or stop event is set
        finally:
            camera.set_telemetry(None)

    log.info('Back camera test complete.')
