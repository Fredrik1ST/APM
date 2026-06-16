'''
This module provides the APM's Orchestrator class.

The Orchestrator is a finite state machine that manages the main program flow and glues its components together.
'''

import time
import logging
import threading
import datetime
from pathlib import Path
from enum import IntEnum

from apm import config_handler as config

# ---- Drivers: Responsible for interfacing with asynchronous data sources (cameras, GNSS, Arduino) ----
from apm.drivers.arduino import ArduinoDriver
from apm.drivers.gnss import GNSSDriver
from apm.drivers.camera import CameraDriver

# ---- Program modes (different main loops for different functionalities, e.g. debug modes) ----
from apm import modes

log = logging.getLogger(__name__)
RECORDINGS_DIR = Path(__file__).resolve().parent.parent / "logs" / "recordings"


class State(IntEnum):
    IDLE = 0
    CONFIG = 10
    STARTING = 20
    RUNNING = 21
    RUNNING_PACE = 22
    RUNNING_DIST = 23
    RUNNING_CONST = 24
    STOPPING = 30
    FINISHED = 31
    ERROR = 60

class Mode(IntEnum):
    NONE = 0
    NORMAL = 1
    PACE_ONLY = 2
    DISTANCE_ONLY = 3
    CONSTANT_SPEED = 4
    CAMERA_TEST = 100
    CAMERA_TEST_FRONT = 101
    CAMERA_TEST_BACK = 102
    ARDUINO_TEST = 200
    GNSS_TEST = 300
    LANE_KEEPER_TEST = 400

class Orchestrator:
    """
    Finite state machine responsible for main program flow, user interface and component drivers.

    Basic principle:
        1. Initialized by main.py
        2. Runs a web interface on local network
        3. Waits for user input (config / start / stop)
        4. Run selected program mode until completed or stopped by user

    States: 
        - Idle: waiting for user input to start or configure parameters
        - Config: allowing user to set parameters (e.g. pace profile, distance tolerance)
        - Starting: triggered by user, initializing hardware and software components
        - Running: running selected program mode
            - Running_Pace: following pace profile
            - Running_Dist: maintaining distance to runner
            - Running_Const: maintaining constant speed
        - Stopping: exiting mode and stopping all components, triggered by user
        - Stopped: same as idle, but indicates a successful run has completed
        - Error: Stopped due to an error
    """

    def __init__(self):
        config.initialize()
        self.cfg    = config.load()
        self.state  = State.IDLE
        self.mode   = Mode.NONE

        # Drivers for hardware components
        # Handles communication, data retrieval and processing in separate threads
        self.arduino        = ArduinoDriver()
        self.gnss           = GNSSDriver()
        self.front_camera   = CameraDriver(name="front")
        self.back_camera    = CameraDriver(name="back")

        # Signals and status to/from user interface (e.g. web app)
        self._run_event     = threading.Event() # set by request_start()
        self._stop_event    = threading.Event() # set by request_stop()
        self.front_image: bytes | None = None  # Store post-processed camera image (e.g. with lane overlay)
        self.back_image: bytes | None = None   # Store post-processed camera image (e.g. with runner tracking overlay)


    # -------------------------------------------------------------------------
    # Start / stop signals from user interface (e.g. web app)
    # -------------------------------------------------------------------------

    def request_start(self) -> None:
        '''Signal that the user has pressed Start.'''
        if self.state != State.IDLE:
            return
        self._stop_event.clear()
        self._run_event.set()

    def request_stop(self) -> None:
        '''Signal the orchestrator to stop the current run gracefully.'''
        self._stop_event.set()

    def force_start(self, mode: Mode) -> None:
        '''Force start a specific mode, useful for testing without the web UI.
        
        Usage::

            # Create test script or edit main.py to include:
            from apm.orchestrato import Orchestrator, Mode
            orchestrator = Orchestrator()
            orchestrator.force_start(Mode.CAMERA_TEST_BACK)'''
        if self.state != State.IDLE:
            log.warning('Cannot force start while not in IDLE state.')
            return
        self.mode = mode
        self._stop_event.clear()
        self._run_event.set()


    # -------------------------------------------------------------------------
    # Main entry point
    # -------------------------------------------------------------------------

    def run(self) -> None:
        '''
        Wait for a mode to be set + start signal from user (e.g. web app, button press, or force_start() method).
        
        Config can be adjusted while waiting in IDLE state.
        '''

        while True:
            self.state = State.IDLE
            log.info('Waiting for start signal from web UI...')
            self._run_event.wait()
            self._run_event.clear()

            try:
                self._do_run()
            except Exception:
                log.exception('Orchestrator error')
                self.state = State.ERROR
                time.sleep(3)
                self._stop_event.clear()
            finally:
                self._shutdown()


    # -------------------------------------------------------------------------
    # Program modes - Start / stop routines
    # For each mode, start necessary drivers and enter the main loop for that mode (logic from /modes)
    # -------------------------------------------------------------------------

    def _do_run(self) -> None:
        """Start the selected mode. Run until completion or stop signal received."""
        self.state = State.STARTING
        self.cfg = config.load()
        log.info(f'Starting in {self.mode.name} mode')

        match self.mode:
            case Mode.NONE:
                log.warning('No mode selected!')
            case Mode.NORMAL:
                pass # TODO
            case Mode.PACE_ONLY:
                pass # TODO
            case Mode.DISTANCE_ONLY:
                pass # TODO
            case Mode.CONSTANT_SPEED:
                pass # TODO
            case Mode.CAMERA_TEST:
                pass # TODO
            case Mode.CAMERA_TEST_FRONT:
                self._run_camera_test_front()
            case Mode.CAMERA_TEST_BACK:
                self._run_camera_test_back()
            case Mode.GNSS_TEST:
                self._run_gnss_test()
            case Mode.ARDUINO_TEST:
                self._run_arduino_test()
            case Mode.LANE_KEEPER_TEST:
                self._run_lane_keeper_test()


    def _run_camera_test_front(self) -> None:
        """Start the front camera with lane detection enabled. Log detected lanes and get a sense of accuracy"""
        kwargs = self._camera_kwargs(name='front')
        self.front_camera.start(**kwargs)
        try:
            modes.camera_test_front(self.front_camera, self._stop_event, self.cfg,
                                        lambda img: setattr(self, 'front_image', img))
        finally:
            self.mode = Mode.STOPPING
            self.front_camera.stop()


    def _run_camera_test_back(self) -> None:
        """Start the back camera with body tracking enabled. Log whether a runner is detected and their distance."""
        kwargs = self._camera_kwargs(name='back')
        self.back_camera.start(**kwargs)
        try:
            modes.camera_test_back(self.back_camera, self._stop_event, self.cfg)
        finally:
            self.mode = Mode.STOPPING
            self.back_camera.stop()


    def _run_gnss_test(self) -> None:
        """Verify that the GNSS receiver is working + GPSD daemon is running by logging position and speed."""
        kwargs = self._gnss_kwargs()
        if self.gnss.start(**kwargs) != 0:
            log.error('GNSS failed to start - aborting test.')
            return
        try:
            modes.gnss_test(self.gnss, self._stop_event, self.cfg)
        finally:
            self.mode = Mode.STOPPING
            self.gnss.stop()


    def _run_arduino_test(self) -> None:
        """Send commands to the Arduino and verify feedback until user stops the test."""
        kwargs = self._arduino_kwargs()
        self.arduino.start(**kwargs)
        try:
            modes.arduino_test(self.arduino, self._stop_event, self.cfg)
        finally:
            self.mode = Mode.STOPPING
            self.arduino.stop()


    def _run_lane_keeper_test(self) -> None:
        """Test the Lane Keeper controller without any speed control for tuning / debugging."""
        cam_kwargs = self._camera_kwargs(name='front')
        self.front_camera.start(**cam_kwargs)

        arduino_kwargs = self._arduino_kwargs()
        self.arduino.start(arduino_kwargs)
        try:
            modes.lane_keeper_test(self.front_camera, self.arduino, self._stop_event, self.cfg, self.front_image)
        finally:
            self.mode = Mode.STOPPING
            self.front_camera.stop()
            self.arduino.stop()


    def _shutdown(self) -> None:
        self.back_image = None
        self.front_image = None
        self._stop_event.clear()
        log.info('Orchestrator stopped')
        self.state = State.IDLE


    # -------------------------------------------------------------------------
    # Helper methods for cleaner code in state machine
    # -------------------------------------------------------------------------

    def _camera_kwargs(self, name: str) -> dict:
        """Convert the camera config to keyword arguments for the CameraDriver.start() method"""
        try:
            cam_cfg = self.cfg['camera'][name]
        except KeyError:
            log.error(f"Config for 'camera.{name}' not found.")
            raise
        return {
            'serial': cam_cfg['serial_number'],
            'fps': cam_cfg['fps'],
            'depth_enabled': cam_cfg['depth']['enable'],
            'body_tracking_enabled': cam_cfg['body_tracking']['enable'],
            'body_tracking_confidence': cam_cfg['body_tracking']['detection_confidence'],
            'svo_path': self._svo_path(name) if cam_cfg.get('record') else None
        }

    def _gnss_kwargs(self) -> dict:
        """Convert the GNSS config to keyword arguments for the GNSSDriver.start() method"""
        gnss_cfg = self.cfg['gnss']
        return {
            'measurement_rate_hz': gnss_cfg['measurement_rate']
        }

    def _arduino_kwargs(self) -> dict:
        """Convert the Arduino config to keyword arguments for the ArduinoDriver.start() method"""
        arduino_cfg = self.cfg['arduino']
        return {
            'host': arduino_cfg['socket']['host'],
            'port': arduino_cfg['socket']['port'],
        }
    
    def _svo_path(self, camera_name: str) -> str | None:
        """Return a timestamped SVO recording path for the given camera, or None if recording is disabled."""
        if not self.cfg['camera'][camera_name].get('record'):
            return None
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        return str(RECORDINGS_DIR / f'{camera_name}_{timestamp}.svo2')