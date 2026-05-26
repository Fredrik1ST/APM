'''
The brain that carries out the main logic of the Autonomous Pacemaker.

Main states:
- Idle: waiting for user input to start or configure parameters
- Config: allowing user to set parameters (e.g. pace profile, distance tolerance)
- Starting: triggered by user, initializing hardware and software components
- Running: main program loop started
    - Pace: following pace profile
    - Dist: maintaining distance to runner
    - Const: maintaining constant speed
- Stopping: triggered by user or error, stopping all components
- Stopped: same as idle, but indicates a successful run has completed
- Error: if stopped due to error, waiting for user to reset

'''

import time
import logging
import threading
from enum import IntEnum
import config_handler as config

# ---- Wrappers: Responsible for interfacing with asynchronous data sources (cameras, GNSS, Arduino) ----
from arduino_comm import ArduinoWrapper, blink
from gnss import GNSSWrapper
# From the other modules, import their respective wrappers


# ---- Program modes (different main loops for different functionalities, e.g. debug modes) ----
from modes.arduino_test import arduino_test_mode
from modes.gnss_test import gnss_test_mode


log = logging.getLogger(__name__)


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

class Orchestrator:

    def __init__(self):
        config.initialize()
        self.cfg = config.load()
        self.state = State.IDLE
        self.mode = Mode.NONE

        # Wrappers for hardware components - Handle communication, data retrieval and processing in separate threads
        self.arduino = ArduinoWrapper()
        self.gnss = GNSSWrapper()
        #self.front_camera = CameraWrapper()
        #self.back_camera = CameraWrapper()

        # Signals and status to/from user interface (e.g. web app)
        self._run_event  = threading.Event()  # set by request_start()
        self._stop_event = threading.Event()  # set by request_stop()

        self.arduino_connected: bool = False
        self.front_camera_ok: bool = False
        self.back_camera_ok: bool = False


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


    # -------------------------------------------------------------------------
    # Main entry point
    # -------------------------------------------------------------------------

    def run(self) -> None:
        '''
        Start the web interface then sit in an idle/run cycle until the
        process is killed. The web UI drives request_start() / request_stop().
        '''
        from webapp.app import start as start_webapp
        start_webapp(self)

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
                self._stop_event.clear()
            finally:
                self._shutdown()


    # -------------------------------------------------------------------------
    # Program modes - Start / stop routines
    # For each mode, start necessary wrappers and enter the main loop for that mode (logic from /modes)
    # -------------------------------------------------------------------------

    def _do_run(self) -> None:
        self.state = State.STARTING
        self.cfg = config.load()
        log.info(f'Starting in {self.mode.name} mode')

        match self.mode:
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
                pass # TODO
            case Mode.CAMERA_TEST_BACK:
                pass # TODO
            case Mode.GNSS_TEST:
                self._run_gnss_test()
            case Mode.ARDUINO_TEST:
                self._run_arduino_test()
    

    def _run_gnss_test(self) -> None:
        """Verify that the GNSS receiver is working + GPSD daemon is running by logging position and speed."""
        if self.gnss.start() != 0:
            log.error('GNSS failed to start — aborting test.')
            return
        try:
            gnss_test_mode(self.gnss, self._stop_event, self.cfg)
            self.state = State.FINISHED
            time.sleep(3)
        finally:
            self.gnss.stop()


    def _run_arduino_test(self) -> None:
        """Send commands to the Arduino and verify feedback until user stops the test"""
        self.arduino.start(
            host = self.cfg["arduino"]["socket"]["host"],
            port = self.cfg["arduino"]["socket"]["port"])
        try:
            arduino_test_mode(self.arduino, self._stop_event, self.cfg)

            self.state = State.FINISHED
            time.sleep(3) # Allow user to see finished state before going back to idle
        finally:
            self.arduino.stop()
            self._shutdown()


    def _shutdown(self) -> None:
        self.state = State.STOPPING
        self._stop_event.clear()
        log.info('Orchestrator stopped')
        time.sleep(3)
        self.state = State.IDLE
