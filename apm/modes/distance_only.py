'''
Main loop of the Distance Only mode.

Starts the back camera with body tracking and the Arduino, then runs the
DistanceController in a closed loop: camera distance -> PID -> PWM -> Arduino.
Logs distance and PWM output periodically and on runner appearance/disappearance.
'''

import time
import logging
import threading

import tomlkit

from apm.drivers.arduino import ArduinoDriver, MessageCommands
from apm.drivers.camera import CameraDriver
from apm.control.distance_controller import DistanceController
from apm.control.pid_controller import PIDController
from apm.control.velocity_profiles import LinearRamp
from apm.control.run_filter import RunSignalFilter
from apm.speed_models import AffineSpeedModel
from apm.telemetry import TelemetryLogger

log = logging.getLogger(__name__)

_TICK = 1/50  # Control loop interval [s]
_LOG_INTERVAL = 1.0  # Periodic status log [s]


def distance_only(
    camera: CameraDriver,
    arduino: ArduinoDriver,
    stop_event: threading.Event,
    cfg: tomlkit.TOMLDocument,
) -> None:

    cfg_pid = cfg['distance_control']['pid']
    pid = PIDController(
        kp=float(cfg_pid['kp']),
        ki=float(cfg_pid['ki']),
        kd=float(cfg_pid['kd']),
        beta=float(cfg_pid['beta']),
        gamma=float(cfg_pid['gamma']),
    )

    cfg_sm = cfg['speed_model']
    speed_model = AffineSpeedModel(gain=float(cfg_sm['gain']), bias=float(cfg_sm['bias']))
    ramp = LinearRamp(t_accel=float(cfg['velocity_profile']['linear']['t_accel']))
    controller = DistanceController(pid=pid, speed_model=speed_model, ramp=ramp)

    cfg_dc = cfg['distance_control']
    setpoint = (float(cfg_dc['d_min']) + float(cfg_dc['d_max'])) / 2.0

    # Debounce the flickery body-tracking detection into a stable run signal so brief
    # dropouts don't cut the motor and lurch the car. While bridging a dropout we hold the
    # last known distance so the controller keeps tracking instead of seeing "no runner".
    run_filter = RunSignalFilter(
        release_s=float(cfg_dc.get('runner_lost_timeout', 0.5)),
        acquire_s=float(cfg_dc.get('runner_acquire_time', 0.0)),
    )

    last_log_time = 0.0
    last_distance = -1.0    # most recent valid runner distance (for bridging dropouts)
    dropouts = 0           # Count how many times the runner is truly lost (after debounce)

    # Telemetry: per-tick setpoint vs measured distance and the full control output, so the
    # distance controller's tracking performance can be plotted/tuned offline.
    with TelemetryLogger('distance_only') as tlm:
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                snap = camera.get_snapshot()

                if snap is not None:
                    detected = len(snap.bodies) > 0
                    was_present = run_filter.present
                    present = run_filter.update(detected, now)

                    if detected:
                        last_distance = snap.bodies[0].position[2]
                        runner_distance = last_distance
                    elif present:
                        runner_distance = last_distance  # bridge a brief dropout with the last distance
                    else:
                        runner_distance = -1.0           # truly lost -> controller commands a stop

                    if present and not was_present:
                        log.info(f'Runner detected - Distance: {runner_distance:.2f} m')
                    elif was_present and not present:
                        log.info('Runner lost')
                        dropouts += 1

                    pwm = controller.update(runner_distance, setpoint)
                    arduino.write_msg(MessageCommands(run=present, speed_pwm=pwm))

                    link = arduino.get_link_stats()
                    tlm.log('control', {
                        'setpoint':      round(setpoint, 3),
                        'distance':      round(runner_distance, 3),   # value fed to controller (-1 if lost)
                        'detected':      detected,
                        'present':       present,
                        'pid_error':     round(pid.error, 4),
                        'pid_p':         round(pid.p, 4),
                        'pid_i':         round(pid.i, 4),
                        'pid_d':         round(pid.d, 4),
                        'desired_speed': round(controller.last_desired_speed, 4),
                        'cmd_speed':     round(controller.last_ramped_speed, 4),
                        'pwm':           round(pwm, 1),
                        'fb_nr':         link.fb_nr,
                        'rtt_ms':        round(link.rtt_s * 1e3, 3),
                    })

                    if now - last_log_time >= _LOG_INTERVAL:
                        if present:
                            bridging = '' if detected else '  (bridging dropout)'
                            log.info(f'Distance: {runner_distance:.2f} m  Setpoint: {setpoint:.2f} m  PWM: {pwm:.1f}{bridging}')
                        else:
                            log.info('No runner detected')
                        last_log_time = now

                stop_event.wait(1 / _TICK)
        finally:
            arduino.write_msg(MessageCommands(run=False))

    log.info('Distance only mode complete.')
    log.info(f'Total dropouts: {dropouts}')