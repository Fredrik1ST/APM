'''
Main loop of Normal mode: the full pace-following behaviour. The crown jewel. Rosinen i pølsa.

Two things are happening simultaneously here:

    Steering    LaneKeeper keeps the car on-track between the front camera's lane lines.
    Speed       A pacing profile (rows of speed & duration) is followed over time.
                Two speed controllers, switching between them based on whether the runner is on pace or falls behind:

        CRUISE  Follow the pacing profile (CruiseController: feedforward speed model + PI
                trim, GNSS-fused speed estimate, and the outer schedule/distance loop).
                The rabbit runs the planned splits.
        DIST    Hold a fixed gap to the runner (DistanceController) when they fall outside
                the tolerance, until their pace recovers.

Commands to the arduino are collected and sent at the end (pwm, steering angle, LED status)

Speed switching (both directions debounced/filtered by `cruise_control.t_control_switch` seconds):

    CRUISE -> DIST   the runner falls outside the distance tolerance
                     (runner_distance > distance_control.d_max) or is lost.
    DIST  -> CRUISE  the runner has made up the lost ground: the schedule deficit
                     (planned vs actually-travelled distance) is back within tolerance.

Schedule policy: the pacing-profile clock is always ticking, so the planned splits stay anchored in time.
While the rabbit paces a fallen-behind runner, the schedule deficit (s_ref - s_meas = the cruise outer
loop's `schedule_error`, kept live via `integrate_schedule`) keeps growing; control returns to cruise
only once the runner has closed that deficit back within tolerance, with the small residual handed to
the outer loop to null rather than sprinting to recover it in one go.
The run ends when the profile is exhausted (or the runner... hehe).

Requires the Arduino, GNSS (cruise speed feedback + odometry), the back camera (runner
distance via body tracking) and the front camera (lane keeping).
'''

import time
import logging
import threading
from enum import Enum
from typing import TYPE_CHECKING, Callable

import tomlkit
import numpy as np

from apm.states import State

from apm.drivers.arduino import ArduinoDriver, MessageCommands, blink
from apm.drivers.camera import CameraDriver
from apm.vision.lane_detector import LaneDetector
from apm.control.cruise_controller import CruiseController, ComplementaryFilter
from apm.control.distance_controller import DistanceController
from apm.control.lane_keeper import LaneKeeper
from apm.control.pid_controller import PIDController
from apm.control.velocity_profiles import LinearRamp
from apm.control.run_filter import RunSignalFilter
from apm.control.pacing_profile import PacingProfile, PROFILES_DIR
from apm.speed_models import AffineSpeedModel
from apm.modes.constant_speed import _make_ramp, _wait_for_connection
from apm.telemetry import TelemetryLogger

if TYPE_CHECKING:
    from apm.drivers.gnss import GNSSDriver

log = logging.getLogger(__name__)

_TICK = 1 / 50            # Control loop interval [s]
_LOG_INTERVAL = 1.0       # Periodic status log [s]
_STRAIGHT = 90.0          # Neutral steering angle [deg]


class _Active(Enum):
    CRUISE = 'cruise'   # following the pacing profile
    DIST = 'dist'       # maintaining distance to runner after falling behind


def _make_pid(cfg_pid) -> PIDController:
    return PIDController(
        kp=float(cfg_pid['kp']),
        ki=float(cfg_pid['ki']),
        kd=float(cfg_pid['kd']),
        beta=float(cfg_pid['beta']),
        gamma=float(cfg_pid['gamma']),
    )


def normal(arduino: ArduinoDriver, gnss: "GNSSDriver",
           front_camera: CameraDriver, back_camera: CameraDriver,
           stop_event: threading.Event, cfg: tomlkit.TOMLDocument,
           set_state: Callable[[State], None] | None = None) -> None:
    '''Normal mode: lane keeping + pace-following with cruise <-> distance switching.

    `set_state`, if given, is called with `State.RUNNING_PACE` while cruise control is active
    and `State.RUNNING_DIST` while distance control is, so the UI shows which controller is
    currently driving.'''

    def _report_state(active: "_Active") -> None:
        if set_state is not None:
            set_state(State.RUNNING_PACE if active is _Active.CRUISE else State.RUNNING_DIST)

    # How close the runner needs to reach the pacing profile to switch back from distance -> cruise control
    catchup_tolerance = float(cfg['cruise_control']['catchup_tolerance_m'])
    # Safety watchdog: quit if the runner is missing or stationary this long (0 = disabled).
    runner_timeout = float(cfg['cruise_control'].get('runner_timeout', 0.0))
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

    # --- Cruise controller: follows the pacing profile using closed-loop controller ---
    cfg_cc = cfg['cruise_control']
    cruise = CruiseController(
        pid=_make_pid(cfg_cc['pid']),                               # A 2DOF PID
        speed_model=speed_model,                                    # Can be interchanged with any SpeedModel class (affine or polynomial)
        ramp=_make_ramp(cfg, str(cfg_cc.get('profile', 'linear'))), # Velocity ramp to avoid jumpy transitions
        cf=ComplementaryFilter(tau=float(cfg_cc['cf_tau'])), # How much to trust speed model vs GNSS (lower tau = more trusting of GNSS, but also inherits its noise)
        k_s=float(cfg_cc['k_s']), # Schedule correction gain: how much of a PI nudge to apply based on schedule / odometry error (0 = none)
        pwm_min=float(cfg_cc['pwm_min']),
        pwm_max=float(cfg_cc['pwm_max']),
    )

    # --- Distance controller: maintain set distance until runner catches back up ---
    cfg_dc = cfg['distance_control']
    d_max = float(cfg_dc['d_max'])
    setpoint = (float(cfg_dc['d_min']) + d_max) / 2.0
    distance = DistanceController(
        pid=_make_pid(cfg_dc['pid']),
        speed_model=speed_model,
        ramp=LinearRamp(t_accel=float(cfg['velocity_profile']['linear']['t_accel'])),
    )

    # Debounce the flickery body-tracking detection into a stable run signal
    # bridge brief dropouts with the last known distance (see distance_only mode).
    run_filter = RunSignalFilter(
        release_s=float(cfg_dc.get('runner_lost_timeout', 0.5)),
        acquire_s=float(cfg_dc.get('runner_acquire_time', 0.0)),
    )

    profile = PacingProfile.from_csv(PROFILES_DIR / str(cfg_cc['pacing_profile']))
    t_switch = float(cfg_cc['t_control_switch'])

    log.info(f"Normal mode: profile='{cfg_cc['pacing_profile']}' "
             f"({profile.total_duration:.0f} s, {profile.total_distance:.0f} m), "
             f"distance band {cfg_dc['d_min']}-{d_max} m, switch debounce {t_switch:.1f} s")
    if not _wait_for_connection(arduino, stop_event):
        return

    cruise.reset(0.0)
    active = _Active.CRUISE
    _report_state(active)
    switch_since: float | None = None   # when the pending speed-mode switch condition first became true
    measured = 0.0                      # latest valid GNSS ground speed (held through fix gaps)
    last_distance = -1.0                # latest valid runner distance (for bridging dropouts)
    steering = _STRAIGHT
    last_tick = 0.0
    last_log = 0.0

    with TelemetryLogger('normal') as tlm:
        gnss.set_telemetry(tlm)  # full-rate gnss.csv in the same run directory
        run_start = time.monotonic()
        last_active = run_start  # watchdog: last time the runner was present and moving
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
                    else:
                        steering = steering # hold last angle when lanes are lost

                # --- Speed sensors: GNSS ground speed + runner distance ---
                snap_g = gnss.get_snapshot()
                if snap_g is not None and snap_g.speed is not None:
                    measured = snap_g.speed

                snap_b = back_camera.get_snapshot()
                detected = bool(snap_b and snap_b.bodies) # Bodies are detected!
                if detected:
                    last_distance = snap_b.bodies[0].position[2]
                present = run_filter.update(detected, now)
                if detected:
                    runner_distance = last_distance
                elif present:
                    runner_distance = last_distance   # bridge a brief dropout
                else:
                    runner_distance = -1.0            # truly lost :(

                # --- Safety watchdog: quit if the runner is missing or stationary too long ---
                # "Stationary" = we're commanding motion (target > floor) but nothing is moving.
                # Intentional zero-speed profile segments don't trip it (target <= floor).
                stalled = target_speed > speed_floor and measured < speed_floor
                if present and not stalled:
                    last_active = now
                if runner_timeout > 0.0 and now - last_active >= runner_timeout:
                    reason = 'no runner detected' if not present else 'runner stationary'
                    log.warning(f'Watchdog: {reason} for {runner_timeout:.0f} s - stopping Normal mode.')
                    break

                # --- Supervisory speed-mode switching (debounced by t_switch) ---
                # While distance control holds the actuator, keep the cruise outer-loop schedule
                # integral live (s_ref - s_meas), so we can tell when the runner is back on schedule.
                if active is _Active.DIST:
                    cruise.integrate_schedule(target_speed, measured, dt)

                fell_behind = runner_distance < 0 or runner_distance > d_max
                caught_up = present and cruise.schedule_error <= catchup_tolerance
                want_switch = fell_behind if active is _Active.CRUISE else caught_up
                if want_switch:
                    switch_since = switch_since if switch_since is not None else now
                    if now - switch_since >= t_switch:
                        if active is _Active.CRUISE:
                            active = _Active.DIST
                            distance.ramp.reset(cruise.last_speed_estimate)  # speed continuity
                            log.info(f'Runner fell behind ({runner_distance:.1f} m) -> distance control')
                        else:
                            active = _Active.CRUISE
                            # Bumpless resume: seed the inner loop to current speed but keep the
                            # (now ~0) schedule deficit so the outer loop nulls the residual.
                            cruise.reset(measured, keep_schedule=True)
                            log.info(f'Runner back on schedule ({cruise.schedule_error:.1f} m deficit) -> cruise control')
                        _report_state(active)
                        switch_since = None
                else:
                    switch_since = None

                # --- Active speed controller -> PWM ---
                if active is _Active.CRUISE:
                    pwm = cruise.update(target_speed, measured, dt)
                    run = True
                    desired_speed = cruise.last_desired_speed
                else:
                    pwm = distance.update(runner_distance, setpoint)
                    run = present  # truly-lost runner -> DistanceController commands 0, drop the run flag
                    desired_speed = distance.last_desired_speed

                # --- Rear LED status (see docs/LED_PATTERNS.md) ---
                if active is _Active.DIST:
                    green_led, red_led = blink(period=1.0), False  # 🟢✨ ⚫    ruuuunn! catch up! (distance control)
                elif fell_behind:
                    green_led, red_led = True, blink(period=0.5)   # 🟢 🔴✨    runner falling behind (debouncing switch)
                else:
                    green_led, red_led = True, False               # 🟢 ⚫      on pace (cruise control)

                # --- Combined commands to arduino: ---
                arduino.write_msg(MessageCommands(run=run, steer_angle=steering, speed_pwm=pwm,
                                                  green_led=green_led, red_led=red_led))

                link = arduino.get_link_stats()
                tlm.log('control', {
                    'profile_t':       round(profile_t, 2),
                    'active':          active.value,
                    'target_speed':    round(target_speed, 4),
                    'desired_speed':   round(desired_speed, 4),
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
                    log.info(f'[{active.value}] t={profile_t:.0f}s  target={target_speed:.2f}  '
                             f'measured={measured:.2f} m/s  runner={runner_distance:.2f} m  '
                             f'PWM={pwm:.0f}  steer={steering:.0f}'
                             f'{"" if present else "  (no runner)"}')
                    last_log = now

                stop_event.wait(_TICK)
        finally:
            gnss.set_telemetry(None)
            arduino.write_msg(MessageCommands(run=False, steer_angle=_STRAIGHT, speed_pwm=neutral_pwm))

    log.info('Normal mode completed - good job!')
