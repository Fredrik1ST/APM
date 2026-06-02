'''
Main loop of the Arduino Test mode.

Once connected, it briefly tests communication both ways and hardware components.

Notes:

    - Wrapper code for starting/stopping modes, threads and hardware communication is handled by the main Orchestrator.
      Modes simply contain the relevant logic for their use cases while "borrowing" the necessary wrappers as arguments.
    - Motors will run briefly during the test. Make sure the car is either elevated or has ample space!

'''

import time
import logging
import threading

import tomlkit
from apm.drivers.arduino import ArduinoWrapper, MessageCommands, blink, mps_to_pwm

log = logging.getLogger(__name__)

_TICK = 0.05        # Main loop interval [s] - 20 Hz
_LOG_INTERVAL = 1.0 # How often to print feedback during a phase [s]


def arduino_test_mode(arduino: ArduinoWrapper, stop_event: threading.Event,
                      cfg: tomlkit.TOMLDocument) -> None:
    angle_neutral     = cfg["arduino"]["steer_angle"]["neutral"]
    angle_right       = cfg["arduino"]["steer_angle"]["max_right"]
    angle_left        = cfg["arduino"]["steer_angle"]["max_left"]
    neutral_speed_pwm = cfg["arduino"]["speed_pwm"]["neutral"]
    pwm_factor        = cfg["arduino"]["speed_pwm"]["factor"]

    SPEED_1 = 0.3 # m/s
    SPEED_2 = 0.6
    SPEED_3 = 0.9

    log.info('Arduino test mode started - waiting for Arduino to connect...')
    if not _wait_for_connection(arduino, stop_event):
        return

    fb = arduino.read_msg()
    log.info(
        f'Arduino connected. ESC limits: '
        f'min={fb.esc_min_pwm:.0f} max={fb.esc_max_pwm:.0f} '
        f'speed_limit={fb.pwm_speed_limit:.0f}'
    )


    # Phase 1 - LED blink
    log.info('--- Phase 1: LED test (3 s) ---')
    if stop_event.is_set():
        return
    _run_phase(arduino, stop_event, duration=3.0,
               run=False, steer_angle=angle_neutral, speed_pwm=neutral_speed_pwm,
               blink_leds=True)


    # Phase 2 - Steering
    log.info('--- Phase 2: Steering test ---')
    if stop_event.is_set():
        return
    for label, angle in [
        ('right',   angle_right),
        ('neutral', angle_neutral),
        ('left',    angle_left),
        ('neutral', angle_neutral),
    ]:
        log.info(f'  Steering {label} ({angle:.0f}°)')
        _run_phase(arduino, stop_event, duration=2.0,
                   run=True, steer_angle=angle, speed_pwm=neutral_speed_pwm)


    # Phase 3 - Motor
    log.info('--- Phase 3: Motor test - ensure car is elevated or has space! ---')
    log.info(f'  Spinning at {SPEED_1} m/s...')
    _run_phase(arduino, stop_event, duration=2.0,
               run=True, steer_angle=angle_neutral,
               speed_pwm=mps_to_pwm(SPEED_1, pwm_factor, neutral_speed_pwm))
    
    log.info(f'  Spinning at {SPEED_2} m/s...')
    _run_phase(arduino, stop_event, duration=2.0,
               run=True, steer_angle=angle_neutral,
               speed_pwm=mps_to_pwm(SPEED_2, pwm_factor, neutral_speed_pwm))

    log.info(f'  Spinning at {SPEED_3} m/s...')
    _run_phase(arduino, stop_event, duration=2.0,
               run=True, steer_angle=angle_neutral,
               speed_pwm=mps_to_pwm(SPEED_3, pwm_factor, neutral_speed_pwm))

    if stop_event.is_set():
        return


    # Phase 4 - Brake test
    log.info('--- Phase 4: Brake test ---')
    _run_phase(arduino, stop_event, duration=4.0,
               run=False, steer_angle=angle_neutral, speed_pwm=neutral_speed_pwm, brake=blink(period=2))

    if stop_event.is_set():
        return


    log.info('Stopping motor...')
    _run_phase(arduino, stop_event, duration=1.0,
               run=False, steer_angle=angle_neutral, speed_pwm=neutral_speed_pwm)

    log.info('Arduino test complete.')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wait_for_connection(arduino: ArduinoWrapper, stop_event: threading.Event,
                         timeout: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout
    while not arduino.is_connected:
        if stop_event.is_set():
            log.info('Stop requested while waiting for connection.')
            return False
        if time.monotonic() > deadline:
            log.error('Timed out waiting for Arduino to connect.')
            return False
        stop_event.wait(0.5)
    return True


def _run_phase(arduino: ArduinoWrapper, stop_event: threading.Event,
               duration: float, run: bool, steer_angle: float,
               speed_pwm: float, blink_leds: bool = False, brake: bool = False) -> None:
    """Send a fixed command for X seconds, logging feedback once per second."""
    deadline = time.monotonic() + duration
    last_log  = 0.0

    msg = MessageCommands()
    msg.run         = run
    msg.steer_angle = steer_angle
    msg.speed_pwm   = speed_pwm

    while not stop_event.is_set() and time.monotonic() < deadline:
        if blink_leds:
            msg.green_led = blink(period=0.5)
            msg.red_led   = blink(period=1.0, offset=0.5)

        arduino.write_msg(msg)

        now = time.monotonic()
        if now - last_log >= _LOG_INTERVAL:
            fb = arduino.read_msg()
            log.info(
                f'  Feedback | run={fb.running} brake={fb.emergency_brake} '
                f'green={fb.green_led} red={fb.red_led} msg_nr={fb.message_nr}'
            )
            last_log = now

        stop_event.wait(_TICK)
