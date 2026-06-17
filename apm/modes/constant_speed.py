'''
Main loop of the Constant Speed mode.

Open-loop speed control: hold a fixed target speed using the feedforward speed model and a
velocity profile, with no GNSS feedback (that is cruise control, handled elsewhere). The
velocity profile ramps smoothly from standstill up to the target so the car doesn't lurch;
the speed model converts the (ramped) desired speed in m/s to an ESC PWM pulse width.

    desired_speed (m/s)  --velocity profile-->  ramped_speed  --speed model-->  PWM  -> Arduino

Useful on its own and as the basis for speed-model validation/calibration: command a known
speed and (with GNSS logging) compare it to the measured ground speed.
'''

import time
import logging
import threading
from typing import TYPE_CHECKING

import tomlkit

from apm.drivers.arduino import ArduinoDriver, MessageCommands
from apm.control.velocity_profiles import LinearRamp, ExponentialRamp, SigmoidRamp
from apm.speed_models import AffineSpeedModel
from apm.telemetry import TelemetryLogger

if TYPE_CHECKING:
    from apm.drivers.gnss import GNSSDriver

log = logging.getLogger(__name__)

_TICK = 1/50 # Control loop interval [s]


def _make_ramp(cfg: tomlkit.TOMLDocument, name: str):
    '''Build the velocity profile selected by `name` from the [velocity_profile.*] config.'''
    vp = cfg['velocity_profile']
    if name == 'linear':
        return LinearRamp(t_accel=float(vp['linear']['t_accel']))
    if name == 'exponential':
        return ExponentialRamp(alpha=float(vp['exponential']['alpha']))
    if name == 'sigmoid':
        return SigmoidRamp(k=float(vp['sigmoid']['k']), s=float(vp['sigmoid']['s']))
    raise ValueError(f"Unknown velocity profile '{name}' (expected linear|exponential|sigmoid)")


def constant_speed(arduino: ArduinoDriver, stop_event: threading.Event,
                   cfg: tomlkit.TOMLDocument,
                   gnss: "GNSSDriver | None" = None) -> None:
    '''Open-loop constant speed control.

    If a started `gnss` driver is given, its fixes are also recorded (full-rate 'gnss'
    stream) and the latest measured ground speed is sampled into each 'control' row, so
    commanded speed/PWM and measured speed sit side by side for speed-model calibration.
    '''

    cfg_cs = cfg['constant_speed']
    target_speed = float(cfg_cs['target_speed'])
    target_duration = float(cfg_cs['target_duration'])
    target_distance = float(cfg_cs.get('target_distance', 0.0))
    profile_name = str(cfg_cs.get('profile', 'linear'))

    cfg_sm = cfg['speed_model']
    speed_model = AffineSpeedModel(gain=float(cfg_sm['gain']), bias=float(cfg_sm['bias']))
    ramp = _make_ramp(cfg, profile_name)
    neutral_pwm = speed_model.speed_to_pwm(0.0)

    log.info(f'Constant speed mode: target={target_speed:.2f} m/s, profile={profile_name} (open loop)'
             f'{" + GNSS logging" if gnss is not None else ""}')
    if not _wait_for_connection(arduino, stop_event):
        return

    last_log_time = 0.0

    start_time = time.monotonic()
    distance = 0.0  # integrated commanded distance [m]

    with TelemetryLogger('constant_speed') as tlm:
        if gnss is not None:
            gnss.set_telemetry(tlm)  # full-rate gnss.csv in the same run directory
        try:
            while not stop_event.is_set():

                if target_distance == 0.0 and target_duration == 0.0:
                    log.warning('No target duration or distance set: exiting to avoid running indefinitely.')
                    break

                cmd_speed = ramp.update(target_speed, _TICK)
                pwm = speed_model.speed_to_pwm(cmd_speed)
                arduino.write_msg(MessageCommands(run=True, speed_pwm=pwm))

                elapsed = time.monotonic() - start_time
                distance += cmd_speed * _TICK  # integrate commanded speed (open loop)

                measured = None
                if gnss is not None:
                    snap = gnss.get_snapshot()
                    measured = snap.speed if snap is not None else None

                link = arduino.get_link_stats()
                tlm.log('control', {
                    'target_speed':   target_speed,
                    'cmd_speed':      round(cmd_speed, 4),
                    'pwm':            round(pwm, 1),
                    'measured_speed': round(measured, 4) if measured is not None else None,
                    'fb_nr':          link.fb_nr,
                    'rtt_ms':         round(link.rtt_s * 1e3, 3),
                })

                now = time.monotonic()
                if now - last_log_time >= 1.0:
                    meas_str = f'  measured={measured:.2f} m/s' if measured is not None else ''
                    log.info(f'cmd_speed={cmd_speed:.2f} m/s (target {target_speed:.2f})  PWM={pwm:.0f}{meas_str}')
                    last_log_time = now

                if target_duration > 0.0 and elapsed >= target_duration:
                    log.info(f'Target duration reached: {elapsed:.1f} s >= {target_duration:.1f} s.')
                    break
                if target_distance > 0.0 and distance >= target_distance:
                    log.info(f'Target distance reached: {distance:.1f} m >= {target_distance:.1f} m.')
                    break

                stop_event.wait(_TICK)
        finally:
            if gnss is not None:
                gnss.set_telemetry(None)
            # Open-loop stop: drop the run flag and command neutral immediately.
            arduino.write_msg(MessageCommands(run=False, speed_pwm=neutral_pwm))

    log.info('Constant speed mode complete.')


def _wait_for_connection(arduino: ArduinoDriver, stop_event: threading.Event,
                         timeout: float = 1.0) -> bool:
    '''Block until the Arduino TCP client connects, so we don't ramp while disconnected.'''
    deadline = time.monotonic() + timeout
    while not arduino.is_connected:
        if stop_event.is_set():
            log.info('Stop requested while waiting for Arduino connection.')
            return False
        if time.monotonic() > deadline:
            log.error('Timed out waiting for Arduino to connect.')
            return False
        stop_event.wait(0.5)
    return True
