'''
Driver for a single ZED X stereo camera.

Based on Stereolabs Python API examples for multithreading and body tracking: 
https://github.com/stereolabs/zed-python-api

Also includes optional recording of the camera feed to an .SVO file for later review. Based on:
https://www.stereolabs.com/docs/video/recording

The driver sets up the camera via the ZED SDK and runs a background daemon thread that continously captures frames.
Body tracking is optional and enabled via the start() call's arguments.

Thread-safety note:
    The grab loop calls camera.grab() directly — a blocking call that waits up to
    1/fps seconds for the next frame. No sleep is needed; every frame the camera
    produces is captured.

    Image data is copied *before* the lock is acquired, so the critical section
    is only a reference swap. The main thread cannot stall the grab loop regardless
    of how long it holds the lock while reading a snapshot.
'''

import numpy as np
import threading
import logging
import time
import pyzed.sl as sl
from dataclasses import dataclass, field

log = logging.getLogger(__name__)


@dataclass
class DetectedBody:
    id: int
    position: tuple[float, float, float]  # (x, y, z) in meters; z is forward distance


@dataclass
class CameraSnapshot:
    image: np.ndarray           # HxWx4 BGRA left image
    timestamp_ms: int
    received_at: float = field(default_factory=time.monotonic)  # Monotonic clock at capture time (seconds)
    bodies: list[DetectedBody] = field(default_factory=list)  # Empty if body tracking is off


class CameraDriver:
    '''
    Driver for a single ZED X stereo camera.

    Usage:
        cam = CameraDriver()
        cam.start(serial=42146143, body_tracking=False)
        ...
        snap = cam.get_snapshot()
        if snap:
            image = snap.image
            distance = snap.bodies[0].position[2] if snap.bodies else -1
        ...
        cam.stop() # When current program mode has completed
    '''

    def __init__(self):
        self._stop = threading.Event()
        self._snapshot: CameraSnapshot | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.is_opened = False


    def start(self,
              serial: int,
              fps: int = 15,
              resolution: sl.RESOLUTION = sl.RESOLUTION.SVGA,
              coord_units: sl.UNIT = sl.UNIT.METER,
              coord_system: sl.COORDINATE_SYSTEM = sl.COORDINATE_SYSTEM.IMAGE,
              body_tracking: bool = False,
              body_tracking_confidence: int = 40,
              svo_path: str | None = None,
              ) -> bool:
        '''Open the camera and start the background capture thread. Returns True on success.

        Args:
            svo_path: If set, record the session to this .svo2 file using H.265 compression.
        '''
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(serial, fps, resolution, coord_units, coord_system,
                  body_tracking, body_tracking_confidence, svo_path),
            daemon=True,
        )
        self._thread.start()
        return True


    def _run(self, serial, fps, resolution, coord_units, coord_system,
             body_tracking, body_tracking_confidence, svo_path):
        init_params = sl.InitParameters()
        init_params.set_from_serial_number(serial)
        init_params.camera_resolution = resolution
        init_params.camera_fps = fps
        init_params.coordinate_units = coord_units
        init_params.coordinate_system = coord_system
        camera = sl.Camera()

        if camera.open(init_params) != sl.ERROR_CODE.SUCCESS:
            log.error(f'Failed to open camera {serial}')
            return

        self.is_opened = True

        # Optional recording of camera feed to .SVO file for later review
        if svo_path:
            rec_params = sl.RecordingParameters(svo_path, sl.SVO_COMPRESSION_MODE.H265)
            err = camera.enable_recording(rec_params)
            if err != sl.ERROR_CODE.SUCCESS:
                log.error(f'Camera {serial}: failed to enable SVO recording to {svo_path}: {err}')
                camera.close()
                self.is_opened = False
                self._stop.set()
                return
            log.info(f'Camera {serial}: recording to {svo_path}')

        body_runtime_params = None
        if body_tracking:
            body_params = sl.BodyTrackingParameters()
            body_params.enable_tracking = True
            body_params.enable_body_fitting = False # Optimize joint position
            body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST

            pos_tracking = sl.PositionalTrackingParameters() # Necessary for body tracking
            pos_tracking.set_floor_as_origin = True
            camera.enable_positional_tracking(pos_tracking)

            log.info(f'Camera {serial}: loading body tracking module...')
            err = camera.enable_body_tracking(body_params)
            if err != sl.ERROR_CODE.SUCCESS:
                log.error(f'Camera {serial}: failed to enable body tracking: {err}')
                camera.close()
                self.is_opened = False
                self._stop.set()
                return

            body_runtime_params = sl.BodyTrackingRuntimeParameters()
            body_runtime_params.detection_confidence_threshold = body_tracking_confidence

        runtime = sl.RuntimeParameters()
        image = sl.Mat()
        bodies = sl.Bodies() if body_tracking else None
        log.info(f'Camera {serial} started (body_tracking={body_tracking})')

        while not self._stop.is_set():
            if camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                camera.retrieve_image(image, sl.VIEW.LEFT)

                # Copy image data before taking lock so the critical section is just a reference swap
                frame = image.get_data().copy()
                ts_ms = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()

                detected_bodies: list[DetectedBody] = []
                if body_tracking:
                    camera.retrieve_bodies(bodies, body_runtime_params)
                    detected_bodies = [
                        DetectedBody(id=b.id, position=tuple(b.position[:3]))
                        for b in bodies.body_list
                    ]

                with self._lock:
                    self._snapshot = CameraSnapshot(image=frame, timestamp_ms=ts_ms, bodies=detected)

        if body_tracking:
            camera.disable_body_tracking()
        if svo_path:
            camera.disable_recording()
        camera.close()
        self.is_opened = False
        log.info(f'Camera {serial} closed')


    def get_snapshot(self) -> CameraSnapshot | None:
        '''Return the latest camera snapshot, or None if no frame has been captured yet.'''
        with self._lock:
            return self._snapshot


    def stop(self):
        '''Signal the capture thread to stop and wait for it to finish.'''
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                log.warning('Camera thread did not stop cleanly within timeout')
        log.info('CameraDriver stopped')
