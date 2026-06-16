#!/usr/bin/env python3
"""Analyze distance-controller telemetry (the 'control' stream from DISTANCE_ONLY mode).

Plots the setpoint vs the measured runner distance over time, plus the control output
(desired/commanded speed, PWM) and the PID term breakdown, and reports tracking error.

Usage:
    python scripts/analyze_distance.py                     # latest DISTANCE_ONLY run
    python scripts/analyze_distance.py logs/telemetry/<run_dir>
    python scripts/analyze_distance.py path/to/control.csv --save out/ --no-show
"""

import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from telemetry_utils import load_stream, resolve_stream, finish_plot


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", help="run dir, control.csv, or run-name substring (default: latest)")
    ap.add_argument("--save", metavar="DIR", help="save plot to this directory/file")
    ap.add_argument("--no-show", action="store_true", help="don't open an interactive window")
    args = ap.parse_args()

    csv_path = resolve_stream(args.path, "control")
    df = load_stream(csv_path)
    print(f"\nDistance-controller telemetry: {csv_path}  ({len(df)} ticks)\n")
    if len(df) < 2:
        print("  Not enough samples to analyze.\n")
        return

    # Tracking error only where the controller had a real measurement (runner present, >0).
    tracking = df[(df["present"] == True) & (df["distance"] > 0)] if "present" in df.columns else df
    if len(tracking):
        err = tracking["distance"] - tracking["setpoint"]
        print("Distance tracking (runner present):")
        print(f"    setpoint={tracking['setpoint'].iloc[-1]:.2f} m   "
              f"mean abs error={err.abs().mean():.3f} m   std={err.std():.3f} m   max|e|={err.abs().max():.3f} m")
        on_target = (err.abs() <= 0.25).mean() * 100
        print(f"    within ±0.25 m: {on_target:.0f}% of present samples")
    if "detected" in df.columns:
        bridged = int(((df["present"] == True) & (df["detected"] == False)).sum())
        print(f"    bridged (debounced) ticks: {bridged}   lost ticks: {int((df['present'] == False).sum())}")

    # --- Plots --------------------------------------------------------------
    t = df["t_rel"].to_numpy()
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(f"Distance controller  -  {csv_path.parent.name}")

    # 1) setpoint vs measured distance (mask the -1 'lost' sentinel so it doesn't distort)
    dist = df["distance"].where(df["distance"] > 0)
    axs[0].plot(t, df["setpoint"], color="black", ls="--", label="setpoint")
    axs[0].plot(t, dist, color="tab:blue", label="measured distance")
    axs[0].set(ylabel="distance [m]", title="Setpoint vs measured")
    axs[0].legend(); axs[0].grid(True, alpha=0.3)

    # 2) control output
    if "desired_speed" in df.columns:
        axs[1].plot(t, df["desired_speed"], label="desired speed (PID)")
    if "cmd_speed" in df.columns:
        axs[1].plot(t, df["cmd_speed"], label="commanded (ramped)")
    axs[1].set(ylabel="speed [m/s]", title="Control output")
    ax_pwm = axs[1].twinx()
    ax_pwm.plot(t, df["pwm"], color="tab:red", alpha=0.4, label="PWM")
    ax_pwm.set_ylabel("PWM [µs]")
    axs[1].legend(loc="upper left"); axs[1].grid(True, alpha=0.3)

    # 3) PID term breakdown
    for term, color in [("pid_p", "tab:green"), ("pid_i", "tab:orange"), ("pid_d", "tab:purple")]:
        if term in df.columns:
            axs[2].plot(t, df[term], label=term, color=color)
    axs[2].axhline(0, color="grey", lw=0.5)
    axs[2].set(xlabel="time [s]", ylabel="contribution", title="PID terms")
    axs[2].legend(); axs[2].grid(True, alpha=0.3)

    fig.tight_layout()
    finish_plot(fig, args.save, not args.no_show, f"{csv_path.parent.name}_distance.png")
    print()


if __name__ == "__main__":
    main()
