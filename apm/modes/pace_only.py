'''
Main loop of Pace Only mode.

Normal mode without the distance controller: the rabbit always follows the pacing
profile (cruise control) and steers with the lane keeper, no matter whether the runner
keeps up. There is no falling-back to distance control - this is a pure pace setter
(e.g. the runner is being tested against the schedule and the rabbit must not wait).

The runner is still watched for safety (same watchdog as Normal mode): if they are lost
(not detected) or the car is at a standstill while motion is commanded for
`cruise_control.runner_timeout` seconds, the run stops.

Requires the Arduino, GNSS (cruise speed feedback), the front camera (lane keeping) and
the back camera (runner-lost watchdog).
'''

import time
import logging
import threading
from typing import TYPE_CHECKING

import tomlkit
import numpy as np

from apm.drivers.arduino import ArduinoDriver, MessageCommands, blink
from apm.drivers.camera import CameraDriver
from apm.vision.lane_detector import LaneDetector
from apm.control.cruise_controller import CruiseController, ComplementaryFilter
from apm.control.lane_keeper import LaneKeeper
from apm.control.run_filter import RunSignalFilter
from apm.control.pacing_profile import PacingProfile, PROFILES_DIR
from apm.speed_models import AffineSpeedModel
from apm.modes.normal import _make_pid, _STRAIGHT
from apm.modes.constant_speed import _make_ramp, _wait_for_connection
from apm.telemetry import TelemetryLogger

if TYPE_CHECKING:
    from apm.drivers.gnss import GNSSDriver

log = logging.getLogger(__name__)

_TICK = 1 / 50            # Control loop interval [s]
_LOG_INTERVAL = 1.0       # Periodic status log [s]


def pace_only(arduino: ArduinoDriver, gnss: "GNSSDriver",
              front_camera: CameraDriver, back_camera: CameraDriver,
              stop_event: threading.Event, cfg: tomlkit.TOMLDocument) -> None:
    '''Pace Only mode: lane keeping + cruise control, with a runner-lost safety watchdog.'''

    cfg_cc = cfg['cruise_control']
    # Safety watchdog: quit if the runner is lost or at a standstill this long (0 = disabled).
    runner_timeout = float(cfg_cc.get('runner_timeout', 0.0))
    speed_floor = float(cfg['gnss']['speed_threshold'])  # [m/s] below this counts as "not moving"

    speed_model = AffineSpeedModel(gain=float(cfg['speed_model']['gain']),
                                   bias=float(cfg['speed_model']['bias']))
    neutral_pwm = speed_model.speed_to_pwm(0.0)

    # --- Steering: lane detector + lane keeper (front camera) ---
    cfg_det = cfg['lane_detector']
    detector = LaneDetector(
        cut_top=float(cfg_det['cut_top']),
        angle_threshold=float(cfg_det['angle_threshold']),
        hough_threshold=int(cfg_det['hough_threshold']),
        canny_sigma=float(cfg_det['canny_sigma']),
        mask_polygons=cfg_det['mask_polygons'],
    )
    lane_keeper = LaneKeeper(
        angle_pid=_make_pid(cfg['lane_keeping']['pid']['angle']),
        position_pid=_make_pid(cfg['lane_keeping']['pid']['position']),
    )

    # --- Cruise controller: follows the pacing profile (always active in this mode) ---
    cruise = CruiseController(
        pid=_make_pid(cfg_cc['pid']),
        speed_model=speed_model,
        ramp=_make_ramp(cfg, str(cfg_cc.get('profile', 'linear'))),
        cf=ComplementaryFilter(tau=float(cfg_cc['cf_tau'])),
        k_s=float(cfg_cc['k_s']),
        pwm_min=float(cfg_cc['pwm_min']),
        pwm_max=float(cfg_cc['pwm_max']),
    )

    # Debounce the flickery body-tracking detection into a stable presence signal for the watchdog.
    cfg_dc = cfg['distance_control']
    run_filter = RunSignalFilter(
        release_s=float(cfg_dc.get('runner_lost_timeout', 0.5)),
        acquire_s=float(cfg_dc.get('runner_acquire_time', 0.0)),
    )

    profile = PacingProfile.from_csv(PROFILES_DIR / str(cfg_cc['pacing_profile']))

    log.info(f"Pace only mode: profile='{cfg_cc['pacing_profile']}' "
             f"({profile.total_duration:.0f} s, {profile.total_distance:.0f} m), "
             f"runner timeout {runner_timeout:.0f} s")
    if not _wait_for_connection(arduino, stop_event):
        return

    cruise.reset(0.0)
    measured = 0.0          # latest valid GNSS ground speed (held through fix gaps)
    last_distance = -1.0    # latest valid runner distance (for logging / bridging dropouts)
    steering = _STRAIGHT
    last_tick = 0.0
    last_log = 0.0

    with TelemetryLogger('pace_only') as tlm:
        gnss.set_telemetry(tlm)  # full-rate gnss.csv in the same run directory
        run_start = time.monotonic()
        last_active = run_start   # watchdog: last time the runner was present and moving
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                dt = now - last_tick if last_tick else _TICK
                last_tick = now

                profile_t = now - run_start  # wall-clock schedule
                if profile.is_finished(profile_t):
                    log.info(f'Pacing profile complete ({profile_t:.0f} s).')
                    break
                target_speed = profile.speed_at(profile_t)

                # --- Steering: lane keeping from the front camera ---
                snap_f = front_camera.get_snapshot()
                lanes = None
                if snap_f is not None and snap_f.image is not None:
                    lanes = detector.detect(snap_f.image[:, :, :3])
                    if lanes is not None:
                        steering = float(np.clip(
                            lane_keeper.update(lanes, image_width=detector.output_width), 0.0, 180.0))
                    # else: hold last steering angle when lanes are lost

                # --- Sensors: GNSS ground speed + runner presence (for the watchdog) ---
                snap_g = gnss.get_snapshot()
                if snap_g is not None and snap_g.speed is not None:
                    measured = snap_g.speed

                snap_b = back_camera.get_snapshot()
                detected = bool(snap_b and snap_b.bodies)
                if detected:
                    last_distance = snap_b.bodies[0].position[2]
                present = run_filter.update(detected, now)
                runner_distance = last_distance if present else -1.0

                # --- Safety watchdog: quit if the runner is lost or at a standstill too long ---
                # "Standstill" = commanding motion (target > floor) but nothing is moving; an
                # intentional zero-speed profile segment (target <= floor) does not trip it.
                stalled = target_speed > speed_floor and measured < speed_floor
                if present and not stalled:
                    last_active = now
                if runner_timeout > 0.0 and now - last_active >= runner_timeout:
                    reason = 'runner lost' if not present else 'runner at standstill'
                    log.warning(f'Watchdog: {reason} for {runner_timeout:.0f} s - stopping Pace only mode.')
                    break

                # --- Cruise control -> PWM (always; the rabbit never waits for the runner) ---
                pwm = cruise.update(target_speed, measured, dt)

                # --- Rear LED status: green solid running; red blinks while lost or at standstill ---
                green_led = True
                red_led = blink(period=0.5) if (not present or stalled) else False

                # --- One combined command: steering + speed ---
                arduino.write_msg(MessageCommands(run=True, steer_angle=steering, speed_pwm=pwm,
                                                  green_led=green_led, red_led=red_led))

                link = arduino.get_link_stats()
                tlm.log('control', {
                    'profile_t':       round(profile_t, 2),
                    'target_speed':    round(target_speed, 4),
                    'desired_speed':   round(cruise.last_desired_speed, 4),
                    'measured_speed':  round(measured, 4),
                    'speed_estimate':  round(cruise.last_speed_estimate, 4),
                    'runner_distance': round(runner_distance, 3),
                    'present':         present,
                    'schedule_error':  round(cruise.schedule_error, 3),
                    'pwm':             round(pwm, 1),
                    'steering':        round(steering, 2),
                    'lanes':           lanes is not None,
                    'fb_nr':           link.fb_nr,
                    'rtt_ms':          round(link.rtt_s * 1e3, 3),
                })

                if now - last_log >= _LOG_INTERVAL:
                    log.info(f't={profile_t:.0f}s  target={target_speed:.2f}  '
                             f'measured={measured:.2f} m/s  PWM={pwm:.0f}  steer={steering:.0f}'
                             f'{"" if present else "  (no runner)"}')
                    last_log = now

                stop_event.wait(_TICK)
        finally:
            gnss.set_telemetry(None)
            arduino.write_msg(MessageCommands(run=False, steer_angle=_STRAIGHT, speed_pwm=neutral_pwm))

    log.info('Pace only mode complete.')
