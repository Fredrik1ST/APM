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
from apm.speed_models import AffineSpeedModel

log = logging.getLogger(__name__)

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

    last_log_time = 0.0
    runner_present = False
    dropouts = 0 # Count how many times runner is lost after being detected

    while not stop_event.is_set():
        now = time.monotonic()
        snap = camera.get_snapshot()

        if snap is not None:
            detected = len(snap.bodies) > 0
            runner_distance = snap.bodies[0].position[2] if detected else -1.0

            if detected and not runner_present:
                log.info(f'Runner detected - Distance: {runner_distance:.2f} m')
                runner_present = True
            elif not detected and runner_present:
                log.info('Runner lost')
                runner_present = False
                dropouts += 1

            pwm = controller.update(runner_distance, setpoint)
            arduino.write_msg(MessageCommands(run=detected, speed_pwm=pwm))

            if now - last_log_time >= _LOG_INTERVAL:
                if detected:
                    log.info(f'Distance: {runner_distance:.2f} m  Setpoint: {setpoint:.2f} m  PWM: {pwm:.1f}')
                else:
                    log.info('No runner detected')
                last_log_time = now

        stop_event.wait(1 / camera.fps)

    arduino.write_msg(MessageCommands(run=False))
    log.info('Distance only mode complete.')
    log.info(f'Total dropouts: {dropouts}')