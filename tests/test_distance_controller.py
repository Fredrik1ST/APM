'''Tests for the DistanceController (camera distance -> PID -> ramp -> speed model -> PWM).'''

import pytest

from apm.control.distance_controller import DistanceController
from apm.control.pid_controller import PIDController
from apm.control.velocity_profiles import LinearRamp
from apm.speed_models import AffineSpeedModel

GAIN = 20.23
BIAS = 1540.0          # PWM for 0 m/s (neutral)


def make_controller(kp=0.5, ki=0.0, kd=0.0, t_accel=0.0):
    # t_accel=0 makes the ramp a pass-through so we test the controller, not the ramp.
    return DistanceController(
        pid=PIDController(kp=kp, ki=ki, kd=kd),
        speed_model=AffineSpeedModel(gain=GAIN, bias=BIAS),
        ramp=LinearRamp(t_accel=t_accel),
    )


def test_no_runner_commands_neutral_pwm():
    '''runner_distance == -1 must command zero speed (neutral PWM), not crash.'''
    ctrl = make_controller()
    pwm = ctrl.update(runner_distance=-1.0, setpoint_distance=4.0)
    assert pwm == pytest.approx(BIAS)


def test_runner_detected_does_not_raise():
    '''Regression: a detected runner used to hit a NameError (no else branch).'''
    ctrl = make_controller()
    pwm = ctrl.update(runner_distance=3.0, setpoint_distance=4.0)
    assert pwm == pytest.approx(BIAS + GAIN * (0.5 * (4.0 - 3.0)))  # P-only: kp*error


def test_runner_too_close_speeds_up():
    '''Runner closer than setpoint -> positive speed -> PWM above neutral.'''
    ctrl = make_controller()
    pwm = ctrl.update(runner_distance=2.0, setpoint_distance=4.0)
    assert pwm > BIAS


def test_runner_too_far_clamps_to_non_negative():
    '''Runner farther than setpoint -> would be negative speed -> clamped to neutral (no reverse).'''
    ctrl = make_controller()
    pwm = ctrl.update(runner_distance=6.0, setpoint_distance=4.0)
    assert pwm == pytest.approx(BIAS)  # max(0, negative) -> 0 m/s -> neutral PWM


def test_pid_integral_frozen_while_no_runner():
    '''The integral must not change while the runner is absent (no windup, pace retained).'''
    ctrl = make_controller(kp=0.0, ki=1.0)
    ctrl.update(runner_distance=2.0, setpoint_distance=4.0)   # error +2 -> integral 2
    integral_after_detect = ctrl.pid.integral
    for _ in range(10):
        ctrl.update(runner_distance=-1.0, setpoint_distance=4.0)
    assert ctrl.pid.integral == integral_after_detect
