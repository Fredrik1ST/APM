'''Test velocity profiles for smoothing out speed transitions.'''

import os
import pytest
import matplotlib.pyplot as plt
from apm.control.velocity_profiles import LinearRamp, ExponentialRamp, SigmoidRamp

SECONDS = 8
DT = 0.01   # seconds per step
STEPS = int(SECONDS / DT)

# Velocity profile arguments
lin_t_accel = 6 # Seconds to reach target
exp_alpha = 0.7 # 4.6 / SECONDS
sig_k = 2 # Steepness: higher = faster transition through the middle 1.5
sig_s = 3.0 # Inflection point: time after a target change where the transition is at its midpoint

# Plot options
#plt_width = 345.0 / 72.27 # LaTeX pt to inches
plt_width = 360.0 / 72.27 # LaTeX pt to inches
golden_ratio = (1 + 5 ** 0.5) / 2
plt_height = plt_width * golden_ratio


def simulate(profile, targets: list[tuple[float, float]]) -> list[float]:
    """Run a profile through a sequence of (time, target) setpoints, return velocity history."""
    velocities = []
    t = 0.0
    target_queue = list(targets)
    current_target = 0.0

    for _ in range(STEPS):
        while target_queue and t >= target_queue[0][0]:
            current_target = target_queue.pop(0)[1]
        velocities.append(profile.update(current_target, DT))
        t += DT

    return velocities


# --- Correctness tests ---

def test_linear_reaches_target():
    profile = LinearRamp(t_accel=lin_t_accel)
    for _ in range(int(lin_t_accel / DT) + 10):
        v = profile.update(1.0, DT)
    assert abs(v - 1.0) < 0.01

def test_exponential_reaches_target():
    profile = ExponentialRamp(alpha=exp_alpha)
    for _ in range(STEPS):
        v = profile.update(1.0, DT)
    assert abs(v - 1.0) < 0.01

def test_sigmoid_reaches_target():
    profile = SigmoidRamp(k=sig_k, s=sig_s)
    for _ in range(STEPS):
        v = profile.update(1.0, DT)
    assert abs(v - 1.0) < 0.01


# --- Visual output (run with: pytest -s --plot) ---

@pytest.fixture
def plot(request):
    return request.config.getoption("--plot")


def test_plot_all_profiles(plot):
    #setpoints = [(0.0, 1.0), (2.5, 0.4), (4.0, 1.5)]
    setpoints = [(0.0, 0.0), (1.0, 1.0)]
    time_axis = [i * DT for i in range(STEPS)]

    profiles = {
        "Linear (t_accel={})".format(lin_t_accel): LinearRamp(t_accel=lin_t_accel),
        "Exponential (α={})".format(exp_alpha):  ExponentialRamp(alpha=exp_alpha),
        "Sigmoid (k={}, s={})".format(sig_k, sig_s): SigmoidRamp(k=sig_k, s=sig_s),
    }

    fig, axes = plt.subplots(len(profiles), 1, figsize=(plt_width, plt_height), sharex=False)
    fig.suptitle("Velocity Profile Curves", fontsize=16)

    # Track setpoint history for plotting
    setpoint_history = []
    current_target = 0.0
    sq = list(setpoints)
    for t in time_axis:
        while sq and t >= sq[0][0]:
            current_target = sq.pop(0)[1]
        setpoint_history.append(current_target)

    # Track velocities for each profile for plotting
    for ax, (name, profile) in zip(axes, profiles.items()):
        velocities = simulate(profile, list(setpoints))
        ax.plot(time_axis, velocities, label="Speed", drawstyle="steps-post")
        ax.plot(time_axis, setpoint_history, color="red", linestyle=":", linewidth=1, label="Setpoint", drawstyle="steps-post")
        ax.set_ylabel("Velocity [m/s]")
        ax.set_title(name)
        ax.grid(True)
        ax.legend(loc="lower right")


    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()

    if plot:
        plt.show()
    else:
        plt.savefig("tests/output/velocity_profiles.pdf")
        print("\nPlot saved to tests/output/velocity_profiles.pdf")
