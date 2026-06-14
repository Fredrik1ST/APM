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

import tomlkit

from apm.drivers.camera import CameraDriver
from apm.vision.frame_encoder import encode_jpeg

log = logging.getLogger(__name__)

_LOG_INTERVAL = 1.0  # How often to print a detection status reading [s]


def camera_test_back_mode(camera: CameraDriver, stop_event: threading.Event,
                          cfg: tomlkit.TOMLDocument) -> None:
    
    back_cfg = cfg['camera']['back']
    serial = back_cfg['serial_number']
    fps = back_cfg['fps']
    confidence = back_cfg['body_tracking']['detection_confidence']

    log.info('Starting back camera with body tracking. Press Stop to exit.')
    camera.start(serial=serial, fps=fps, body_tracking=True,
                 body_tracking_confidence=confidence)

    last_log_time = 0.0
    runner_present = False  # Tracks whether a runner was detected in the previous frame

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

            # Periodic status log
            if now - last_log_time >= _LOG_INTERVAL:
                if detected:
                    distance = snap.bodies[0].position[2]
                    log.info(f'Runner present - Distance: {distance:.2f} m')
                else:
                    log.info('No runner detected')
                last_log_time = now

            jpg = encode_jpeg(snap.image, quality=80, scale=1.0)
            cv2.imshow('Back Camera', snap.image)

        stop_event.wait(1 / fps) # Sleep until next frame is expected or stop event is set

    camera.stop()
    log.info('Back camera test complete.')
