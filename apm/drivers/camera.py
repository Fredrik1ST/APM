'''
Driver for a single ZED X stereo camera.

Based on Stereolabs Python API examples for multithreading and body tracking: 
https://github.com/stereolabs/zed-python-api

Includes optional recording of the camera feed to an .SVO file for later review. Based on:
https://www.stereolabs.com/docs/video/recording
'''

import numpy as np
import threading
import logging
import time
import pyzed.sl as sl
from dataclasses import dataclass, field

from apm.telemetry import TelemetryLogger


@dataclass
class DetectedBody:
    id: int
    position: tuple[float, float, float]  # (x, y, z) in meters, where z is distance


@dataclass
class CameraSnapshot:
    """Copy of the latest camera frame and detected bodies"""
    timestamp_monotonic: float = field(default_factory=time.monotonic)  # Monotonic clock at copy time [s]
    timestamp_ms: int = 0               # Timestamp from the camera's internal clock [ms]
    seq: int = 0                        # Frame counter, increments per successful grab (gaps => dropped)
    image: np.ndarray | None = None     # HxWx4 BGRA left image
    depth: np.ndarray | None = None     # HxW depth map [m], or None if depth disabled
    bodies: list[DetectedBody] = field(default_factory=list)  # Empty if body tracking is off
    runner_distance: float = -1.0 # Distance to first detected body (-1 if no bodies)

class CameraDriver:
    """
    Driver for a single ZED X stereo camera.

    Sets up the camera via the ZED SDK and runs a background daemon thread that continously captures frames.
    Frames are stored in thread-safe snapshots that can be retrieved via get_snapshot() in main thread.
    Body tracking & depth perception is optional and enabled via the start() call's arguments.

    Attributes:
        ok (bool): True if the camera is capturing
        serial (int): The camera's serial number (printed on a sticker on the camera)
        fps (int): Frames per second (15, 30, 60, or 120 for ZED X)
        coord_units (sl.UNIT): Units for measurements (e.g. sl.UNIT.METER)
        coord_system (sl.COORDINATE_SYSTEM): Coordinate system for measurements (defaults to sl.COORDINATE_SYSTEM.IMAGE)
        depth_enabled (bool): Grab depth map aligned to the left image
        body_tracking_enabled (bool): Track human bodies and return their 3D coordinates
        body_tracking_confidence (int): Minimum confidence [0-100] to consider a body detection valid
        svo_path (str | None): If set, record the session to this path


    Usage:
        cam = CameraDriver()
        cam.start(serial=42146143, body_tracking_enabled=True)
        ...
        snap = cam.get_snapshot()
        if snap:
            image = snap.image
            distance = snap.bodies[0].position[2] if snap.bodies else -1
        ...
        cam.stop() # When current program mode has completed
    """

    def __init__(self, name: str = "Camera"):
        self.name = name
        self.log = logging.getLogger(f"{__name__}.{name}")
        self.is_opened = False
        self.ok = False # True if camera is successfully capturing frames

        self.frame_seq = 0      # Successful grabs so far (read by modes to measure achieved FPS)
        self.grab_failures = 0  # Cumulative non-SUCCESS grabs

        self.serial: int = 0
        self.fps: int = 15
        self.resolution: sl.RESOLUTION = sl.RESOLUTION.SVGA
        self.svo_path: str | None = None
        self.coord_units: sl.UNIT = sl.UNIT.METER
        self.coord_system: sl.COORDINATE_SYSTEM = sl.COORDINATE_SYSTEM.IMAGE
        self.depth_enabled: bool = False

        self.body_tracking_enabled: bool = False
        self.body_tracking_confidence: int = 40
        self.body_tracking_model: sl.BODY_TRACKING_MODEL = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
    
        self._snapshot: CameraSnapshot | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None

        # Optional telemetry sink. When set, every successful grab is logged to the
        # 'camera_<name>' stream. Reference assignment is atomic under the GIL, so the
        # capture thread reads it without a lock; TelemetryLogger.log() is thread/close-safe.
        self._telemetry = None
        

    def start(self,
              serial: int,
              fps: int = 15,
              resolution: sl.RESOLUTION = sl.RESOLUTION.SVGA,
              coord_units: sl.UNIT = sl.UNIT.METER,
              coord_system: sl.COORDINATE_SYSTEM = sl.COORDINATE_SYSTEM.IMAGE,
              body_tracking_enabled: bool = False,
              body_tracking_confidence: int = 40,
              body_tracking_model: sl.BODY_TRACKING_MODEL = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST,
              depth_enabled: bool = False,
              svo_path: str | None = None,
              ) -> bool:
        '''Open the camera and start the background capture thread. Returns True on success.

        Args:
            svo_path: If set, record the session to this .svo2 file using H.265 compression.
        '''
        # Store the requested configuration so _run() actually uses it (otherwise the camera
        # would open with the __init__ defaults - serial 0, 15 fps - ignoring the config).
        self.serial = serial
        self.fps = fps
        self.resolution = resolution
        self.coord_units = coord_units
        self.coord_system = coord_system
        self.body_tracking_enabled = body_tracking_enabled
        self.body_tracking_confidence = body_tracking_confidence
        self.body_tracking_model = body_tracking_model
        self.depth_enabled = depth_enabled
        self.svo_path = svo_path

        self.frame_seq = 0
        self.grab_failures = 0

        self._stop.clear()
        self._thread = threading.Thread(target=self._run,daemon=True)
        self._thread.start()
        return True


    def _run(self):
        init_params = sl.InitParameters()
        init_params.set_from_serial_number(self.serial)
        init_params.camera_resolution = self.resolution
        init_params.camera_fps = self.fps
        init_params.coordinate_units = self.coord_units
        init_params.coordinate_system = self.coord_system
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE if self.depth_enabled else sl.DEPTH_MODE.NONE
        camera = sl.Camera()

        errCode = camera.open(init_params)
        if errCode != sl.ERROR_CODE.SUCCESS:
            self.log.error(f'Failed to open camera {self.serial}: {errCode}')
            return

        self.is_opened = True

        # Optional recording of camera feed to .SVO file for later review
        if self.svo_path:
            rec_params = sl.RecordingParameters(self.svo_path, sl.SVO_COMPRESSION_MODE.H265)
            err = camera.enable_recording(rec_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.log.error(f'Camera {self.serial}: failed to enable SVO recording to {self.svo_path}: {err}')
                camera.close()
                self.is_opened = False
                self._stop.set()
                return
            self.log.info(f'Camera {self.serial}: recording to {self.svo_path}')

        # Set up body tracking if enabled
        body_runtime_params = None
        if self.body_tracking_enabled:
            body_params = sl.BodyTrackingParameters()
            body_params.enable_tracking = True
            body_params.enable_body_fitting = False # Optimize joint position
            body_params.detection_model = self.body_tracking_model
            body_params.body_selection = self.body_tracking_confidence
            body_params.body_format = sl.BODY_FORMAT.BODY_18 # Simplest skeleton 

            pos_tracking = sl.PositionalTrackingParameters() # Necessary for body tracking
            # pos_tracking.set_floor_as_origin = True # Not sure if needed
            camera.enable_positional_tracking(pos_tracking)

            self.log.info(f'Camera {self.serial}: loading body tracking module...')
            err = camera.enable_body_tracking(body_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.log.error(f'Camera {self.serial}: failed to enable body tracking: {err}')
                camera.close()
                self.is_opened = False
                self._stop.set()
                return

            body_runtime_params = sl.BodyTrackingRuntimeParameters()
            body_runtime_params.detection_confidence_threshold = self.body_tracking_confidence

        runtime = sl.RuntimeParameters()
        image = sl.Mat()
        depth = sl.Mat() if self.depth_enabled else None
        bodies = sl.Bodies() if self.body_tracking_enabled else None
        self.log.info(f'Camera {self.serial} started (body_tracking_enabled={self.body_tracking_enabled})')

        prev_grab_mono: float | None = None  # monotonic time of the previous successful grab
        prev_cam_ts_ms: int | None = None    # camera clock of the previous frame

        while not self._stop.is_set():
            grab_status = camera.grab(runtime)
            if grab_status == sl.ERROR_CODE.SUCCESS:
                grab_mono = time.monotonic()
                camera.retrieve_image(image, sl.VIEW.LEFT)
                if self.depth_enabled:
                    camera.retrieve_measure(depth, sl.MEASURE.DEPTH)

                # Copy image / depth data to NumPy arrays before lock
                image_np = image.get_data().copy()
                depth_np = depth.get_data().copy() if self.depth_enabled else None
                ts_ms = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()

                detected_bodies: list[DetectedBody] = []
                if self.body_tracking_enabled:
                    camera.retrieve_bodies(bodies, body_runtime_params)
                    detected_bodies = [
                        DetectedBody(id=b.id, position=tuple(b.position[:3])) for b in bodies.body_list
                    ]

                self.frame_seq += 1
                self.ok = True
                with self._lock:
                    self._snapshot = CameraSnapshot(
                        timestamp_ms = ts_ms,
                        seq = self.frame_seq,
                        image = image_np,
                        depth = depth_np,
                        bodies = detected_bodies,
                        runner_distance = detected_bodies[0].position[2] if detected_bodies else -1.0)

                # Per-frame timing telemetry. dt_grab is our loop cadence (-> achieved FPS);
                # dt_cam is the camera-clock interval (a jump of ~2x the frame period means the
                # SDK dropped a frame before we grabbed it); proc is our post-grab processing cost.
                if self._telemetry is not None:
                    self._log_frame(
                        grab_status='SUCCESS',
                        ts_ms=ts_ms,
                        dt_grab_ms=(grab_mono - prev_grab_mono) * 1e3 if prev_grab_mono is not None else None,
                        dt_cam_ms=(ts_ms - prev_cam_ts_ms) if prev_cam_ts_ms is not None else None,
                        proc_ms=(time.monotonic() - grab_mono) * 1e3,
                        n_bodies=len(detected_bodies),
                    )
                prev_grab_mono = grab_mono
                prev_cam_ts_ms = ts_ms
            else:
                # grab() failed (e.g. CAMERA_NOT_DETECTED, no new frame yet). Count it and
                # avoid a tight busy-loop hammering the SDK on persistent failure.
                self.grab_failures += 1
                if self.grab_failures % 100 == 1:
                    self.log.warning(f'Camera {self.serial}: grab failed ({grab_status}), total failures={self.grab_failures}')
                if self._telemetry is not None:
                    self._log_frame(grab_status=str(grab_status))
                self._stop.wait(0.002)

        if self.body_tracking_enabled:
            camera.disable_body_tracking()
        if self.svo_path:
            camera.disable_recording()
        camera.close()
        self.is_opened = False
        self.ok = False
        self.log.info(f'Camera {self.serial} closed')


    def get_snapshot(self) -> CameraSnapshot | None:
        '''Return the latest camera snapshot, or None if no frame has been captured yet.'''
        with self._lock:
            return self._snapshot


    def set_telemetry(self, telemetry: "TelemetryLogger | None") -> None:
        '''Attach (or detach with None) a telemetry sink. While attached, every successful grab
        is recorded to the 'camera_<name>' stream so the achieved frame rate and dropped frames
        can be analyzed offline.'''
        self._telemetry = telemetry


    def _log_frame(self, grab_status, ts_ms=None, dt_grab_ms=None, dt_cam_ms=None,
                   proc_ms=None, n_bodies=None) -> None:
        '''Record one grab attempt to telemetry (called from the capture thread).

        Both successful frames and failed grabs share this single schema so the
        'camera_<name>' stream has a stable header regardless of which occurs first.
        dt_grab_ms is the loop cadence (-> achieved FPS); dt_cam_ms is the camera-clock
        interval (a jump of ~2x the frame period means the SDK dropped a frame before we
        grabbed it); proc_ms is the post-grab processing cost.
        '''
        tlm = self._telemetry  # local copy; may be set to None concurrently
        if tlm is None:
            return
        tlm.log(f'camera_{self.name}', {
            'seq':           self.frame_seq,
            'grab_status':   grab_status,
            'cam_ts_ms':     ts_ms,
            'dt_grab_ms':    round(dt_grab_ms, 3) if dt_grab_ms is not None else None,
            'dt_cam_ms':     dt_cam_ms,
            'proc_ms':       round(proc_ms, 3) if proc_ms is not None else None,
            'n_bodies':      n_bodies,
            'grab_failures': self.grab_failures,
            'target_fps':    self.fps,
        })


    def stop(self):
        '''Signal the capture thread to stop and wait for it to finish.'''
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                self.log.warning('Camera thread did not stop cleanly within timeout')
        self.log.info('CameraDriver stopped')
