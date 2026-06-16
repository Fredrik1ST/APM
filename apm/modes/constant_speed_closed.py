'''
Main loop of the Constant Speed (Closed Loop) mode.

Closed-loop speed control: hold a fixed target speed using the CruiseController's
inner velocity loop (feedforward speed model + PI trim on the PWM, with GNSS speed
fused through a complementary filter). The outer schedule/distance loop is forced
off (k_s = 0) since a constant speed has no schedule to track.

    target_speed --ramp--> v_d --speed model--> u_ff --+--> PWM -> Arduino
                                  GNSS --complementary filter--> v_hat
                                  PI(v_d - v_hat) --> u_fb --------^

This is the isolated first hardware test of the cruise inner loop and the basis
for the full cruise mode (which adds pacing-profile parsing and the outer loop).
GNSS is required here (unlike the open-loop constant_speed calibration mode).
'''

import time
import logging
import threading
from typing import TYPE_CHECKING

import tomlkit

from apm.drivers.arduino import ArduinoDriver, MessageCommands
from apm.control.cruise_controller import CruiseController, ComplementaryFilter
from apm.control.pid_controller import PIDController
from apm.speed_models import AffineSpeedModel
from apm.modes.constant_speed import _make_ramp, _wait_for_connection
from apm.telemetry import TelemetryLogger

if TYPE_CHECKING:
    from apm.drivers.gnss import GNSSDriver

log = logging.getLogger(__name__)

_TICK = 0.05  # Control loop interval [s] - 20 Hz


def constant_speed_closed(arduino: ArduinoDriver, gnss: "GNSSDriver",
                          stop_event: threading.Event,
                          cfg: tomlkit.TOMLDocument) -> None:
    '''Closed-loop constant speed control via the CruiseController inner loop.'''

    target_speed = float(cfg['constant_speed']['target_speed'])
    profile_name = str(cfg['constant_speed'].get('profile', 'linear'))

    cfg_cc = cfg['cruise_control']
    cfg_pid = cfg_cc['pid']
    pid = PIDController(
        kp=float(cfg_pid['kp']),
        ki=float(cfg_pid['ki']),
        kd=float(cfg_pid['kd']),
        beta=float(cfg_pid['beta']),
        gamma=float(cfg_pid['gamma']),
    )

    cfg_sm = cfg['speed_model']
    speed_model = AffineSpeedModel(gain=float(cfg_sm['gain']), bias=float(cfg_sm['bias']))
    ramp = _make_ramp(cfg, profile_name)
    cf = ComplementaryFilter(tau=float(cfg_cc['cf_tau']))
    controller = CruiseController(
        pid=pid,
        speed_model=speed_model,
        ramp=ramp,
        cf=cf,
        k_s=0.0,  # outer schedule loop off: a constant speed has no schedule to track
        pwm_min=float(cfg_cc['pwm_min']),
        pwm_max=float(cfg_cc['pwm_max']),
    )
    neutral_pwm = speed_model.speed_to_pwm(0.0)

    log.info(f'Constant speed (closed loop): target={target_speed:.2f} m/s, profile={profile_name}')
    if not _wait_for_connection(arduino, stop_event):
        return

    controller.reset(0.0)
    last_log_time = 0.0
    last_tick = 0.0
    measured = 0.0  # most recent valid GNSS speed; held through brief fix gaps

    # Telemetry: commanded vs estimated vs measured speed plus the full PWM breakdown, so the
    # inner loop's tracking and the complementary filter can be plotted/tuned offline.
    with TelemetryLogger('constant_speed_closed') as tlm:
        gnss.set_telemetry(tlm)  # full-rate gnss.csv in the same run directory
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                dt = now - last_tick if last_tick else _TICK
                last_tick = now

                snap = gnss.get_snapshot()
                if snap is not None and snap.speed is not None:
                    measured = snap.speed

                pwm = controller.update(target_speed, measured, dt)
                arduino.write_msg(MessageCommands(run=True, speed_pwm=pwm))

                link = arduino.get_link_stats()
                tlm.log('control', {
                    'target_speed':   target_speed,
                    'desired_speed':  round(controller.last_desired_speed, 4),   # v_d
                    'speed_estimate': round(controller.last_speed_estimate, 4),  # v_hat
                    'measured_speed': round(measured, 4),                        # v_gnss
                    'u_ff':           round(controller.last_feedforward, 1),
                    'u_fb':           round(controller.last_feedback, 1),
                    'pwm':            round(pwm, 1),
                    'saturated':      controller.last_saturated,
                    'pid_p':          round(pid.p, 4),
                    'pid_i':          round(pid.i, 4),
                    'fb_nr':          link.fb_nr,
                    'rtt_ms':         round(link.rtt_s * 1e3, 3),
                })

                if now - last_log_time >= 1.0:
                    log.info(f'target={target_speed:.2f}  v_d={controller.last_desired_speed:.2f}  '
                             f'v_hat={controller.last_speed_estimate:.2f}  measured={measured:.2f} m/s  '
                             f'PWM={pwm:.0f}{"  (saturated)" if controller.last_saturated else ""}')
                    last_log_time = now

                stop_event.wait(_TICK)
        finally:
            gnss.set_telemetry(None)
            arduino.write_msg(MessageCommands(run=False, speed_pwm=neutral_pwm))

    log.info('Constant speed (closed loop) mode complete.')
