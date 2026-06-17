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
from collections.abc import Callable

import tomlkit

from apm.drivers.arduino import ArduinoDriver, MessageCommands, blink
from apm.control.velocity_profiles import LinearRamp, ExponentialRamp, SigmoidRamp
from apm.speed_models.models import AffineSpeedModel as SpeedModel
from apm.telemetry import TelemetryLogger

log = logging.getLogger(__name__)

_TICK = 1.0/50.0    # Main loop interval [s]
_LOG_INTERVAL = 1.0 # How often to print feedback during a phase [s]


def arduino_test(arduino: ArduinoDriver, stop_event: threading.Event,
                      cfg: tomlkit.TOMLDocument) -> None:
    angle_neutral     = cfg["arduino"]["steer_angle"]["neutral"]
    angle_right       = cfg["arduino"]["steer_angle"]["max_right"]
    angle_left        = cfg["arduino"]["steer_angle"]["max_left"]

    _TICK = cfg["control"]["dt"]                    # Main loop interval [s]
    _LOG_INTERVAL = cfg["control"]["log_interval"]  # How often to print feedback during a phase [s]

    feedforward = SpeedModel(
        gain = cfg["speed_model"]["gain"],
        bias = cfg["speed_model"]["bias"],
    )
    neutral_speed_pwm = feedforward.speed_to_pwm(0.0)

    SPEED_1 = 1 # m/s
    SPEED_2 = 2
    SPEED_3 = 0

    set_speed = LinearRamp(t_accel=4.0) # Velocity profile for smooth speed changes

    log.info('Arduino test mode started - waiting for Arduino to connect...')
    if not _wait_for_connection(arduino, stop_event):
        return

    fb = arduino.read_msg()
    log.info(
        f'Arduino connected. ESC limits: '
        f'min={fb.esc_min_pwm:.0f} max={fb.esc_max_pwm:.0f} '
        f'speed_limit={fb.pwm_speed_limit:.0f}'
    )

    # Capture per-tick command/feedback samples for offline round-trip analysis.
    with TelemetryLogger('arduino_test') as tlm:

        # Phase 1 - LED blink
        log.info('--- Phase 1: LED test (3 s) ---')
        if stop_event.is_set():
            return
        _run_phase(arduino, stop_event, duration=3.0, phase='led',
                   run=False, steer_angle=angle_neutral, speed_pwm=neutral_speed_pwm,
                   blink_leds=True, tlm=tlm)


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
            _run_phase(arduino, stop_event, duration=2.0, phase=f'steer_{label}',
                       run=True, steer_angle=angle, speed_pwm=neutral_speed_pwm, tlm=tlm)


        # Phase 3 - Motor
        log.info('--- Phase 3: Motor test - ensure car is elevated or has space! ---')
        log.info(f'  Spinning at {SPEED_1} m/s...')
        _run_phase(arduino, stop_event, duration=8.0, phase='motor_1',
                   run=True, steer_angle=angle_neutral,
                   speed_pwm=lambda: feedforward.speed_to_pwm(set_speed.update(SPEED_1, _TICK)), tlm=tlm)

        log.info(f'  Spinning at {SPEED_2} m/s...')
        _run_phase(arduino, stop_event, duration=8.0, phase='motor_2',
                   run=True, steer_angle=angle_neutral,
                   speed_pwm=lambda: feedforward.speed_to_pwm(set_speed.update(SPEED_2, _TICK)), tlm=tlm)

        log.info(f'  Spinning at {SPEED_3} m/s...')
        _run_phase(arduino, stop_event, duration=5.0, phase='motor_0',
                   run=True, steer_angle=angle_neutral,
                   speed_pwm=lambda: feedforward.speed_to_pwm(set_speed.update(SPEED_3, _TICK)), tlm=tlm)

        if stop_event.is_set():
            return


        # Phase 4 - Brake test
        log.info('--- Phase 4: Brake test ---')
        _run_phase(arduino, stop_event, duration=4.0, phase='brake',
                   run=False, steer_angle=angle_neutral, speed_pwm=neutral_speed_pwm,
                   brake=blink(period=2), tlm=tlm)

        if stop_event.is_set():
            return


        log.info('Stopping motor...')
        _run_phase(arduino, stop_event, duration=1.0, phase='stop',
                   run=False, steer_angle=angle_neutral, speed_pwm=neutral_speed_pwm, tlm=tlm)

        log.info('Arduino test complete.')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wait_for_connection(arduino: ArduinoDriver, stop_event: threading.Event,
                         timeout: float = 10.0) -> bool:
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


def _run_phase(arduino: ArduinoDriver, stop_event: threading.Event,
               duration: float, run: bool, steer_angle: float,
               speed_pwm: float | Callable[[], float],
               blink_leds: bool = False, brake: bool = False,
               phase: str = '', tlm: TelemetryLogger | None = None) -> None:
    """Run a phase for `duration` seconds. `speed_pwm` may be a fixed float or a callable
    evaluated each tick (use a callable to advance a ramp or other dynamic setpoint).

    If `tlm` is given, one row is recorded to the 'arduino' stream every tick so the
    command stream and feedback message numbers can be analyzed offline (loss / stalls).
    """
    deadline = time.monotonic() + duration
    last_log  = 0.0

    msg = MessageCommands()
    msg.run         = run
    msg.steer_angle = steer_angle

    while not stop_event.is_set() and time.monotonic() < deadline:
        msg.speed_pwm = speed_pwm() if callable(speed_pwm) else speed_pwm

        if blink_leds:
            msg.green_led = blink(period=0.5)
            msg.red_led   = blink(period=1.0, offset=0.5)

        arduino.write_msg(msg)
        fb = arduino.read_msg()

        if tlm is not None:
            link = arduino.get_link_stats()
            tlm.log('arduino', {
                'phase':       phase,
                'cmd_run':     msg.run,
                'cmd_brake':   msg.brake,
                'steer_angle': round(msg.steer_angle, 2),
                'speed_pwm':   round(msg.speed_pwm, 1),
                'sent_nr':     link.sent_nr,
                'fb_nr':       link.fb_nr,
                'rtt_ms':      round(link.rtt_s * 1e3, 3),
                'fb_running':  fb.running,
                'fb_brake':    fb.brake,
            })

        now = time.monotonic()
        if now - last_log >= _LOG_INTERVAL:
            log.info(
                f'  Feedback | run={fb.running} brake={fb.brake} '
                f'speed_pwm={msg.speed_pwm:.0f} '
                f'green={fb.green_led} red={fb.red_led} msg_nr={fb.message_nr}'
            )
            last_log = now

        stop_event.wait(_TICK)
