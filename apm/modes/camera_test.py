'''
Main loop of the (combined) Camera Test mode.

Runs the front and back camera tests at the same time, each in its own thread, sharing a
single telemetry run and stop signal. This is essentially camera_test_front + camera_test_back
running concurrently - useful for gauging system performance with both cameras (and both
vision workloads: lane detection + body tracking) active at once, at different settings.

Each camera driver records its own per-frame timing to the 'camera_front' / 'camera_back'
streams in the shared run directory, so the two can be compared and aligned on t_mono offline
(e.g. against a single-camera run to see the cost of the second camera).
'''

import logging
import threading
from typing import Callable

from tomlkit import TOMLDocument

from apm.drivers.camera import CameraDriver
from apm.telemetry import TelemetryLogger
from apm.modes.camera_test_front import camera_test_front
from apm.modes.camera_test_back import camera_test_back

log = logging.getLogger(__name__)


def camera_test(
    front_camera: CameraDriver,
    back_camera: CameraDriver,
    stop_event: threading.Event,
    set_front_image: Callable[[bytes], None],
    cfg: TOMLDocument,
) -> None:

    log.info('Combined camera test started (front + back).')

    # One telemetry run shared by both cameras so their streams land in the same directory.
    with TelemetryLogger('camera_test') as tlm:
        threads = [
            threading.Thread(
                target=camera_test_front,
                args=(front_camera, stop_event, set_front_image, cfg),
                kwargs={'telemetry': tlm},
                name='camera_test_front',
                daemon=True,
            ),
            threading.Thread(
                target=camera_test_back,
                args=(back_camera, stop_event, cfg),
                kwargs={'telemetry': tlm, 'show_window': False},  # avoid cv2 GUI off the main thread
                name='camera_test_back',
                daemon=True,
            ),
        ]
        for t in threads:
            t.start()
        try:
            # Block until both workers exit (they stop when stop_event is set). If one raises,
            # signal the other to stop too so we never leave a half-running test behind.
            while any(t.is_alive() for t in threads):
                for t in threads:
                    t.join(timeout=0.2)
                if not stop_event.is_set() and not all(t.is_alive() for t in threads):
                    log.warning('A camera test thread exited early; stopping the other.')
                    stop_event.set()
        finally:
            stop_event.set()
            for t in threads:
                t.join(timeout=5.0)

    log.info('Combined camera test complete.')
