#!/usr/bin/env python3
"""Build a speed-model calibration dataset from one or more constant_speed runs.

Because the track isn't always available, calibration data is gathered as several short
constant_speed runs (each holding a different target speed). This script pulls the
steady-state samples from each run's control.csv, merges them into a single calibration CSV
(the format SpeedModelCalibrator expects: setpoint_mps, gnss_mps, pwm), and optionally fits
and compares an affine vs polynomial speed model.

"Steady state" = the commanded speed has plateaued at the run's target, a settling delay has
passed (so the vehicle/GNSS have caught up), and the car is actually moving. Ramp-up samples
are discarded because the commanded PWM and measured speed don't correspond there.

Usage:
    # Merge every constant_speed run and fit:
    python scripts/build_calibration.py --all --fit

    # Specific runs (dirs, control.csv paths, or run-name substrings):
    python scripts/build_calibration.py 2026-06-16_21 run2_dir path/to/control.csv --fit

    # One averaged point per run, custom settle time, save plot:
    python scripts/build_calibration.py --all --reduce mean --settle 3 --fit --save out/
"""

import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from telemetry_utils import load_stream, resolve_stream, list_runs, finish_plot
from apm.speed_models import SpeedModelCalibrator


def gather_control_csvs(specs: list[str], use_all: bool) -> list[Path]:
    """Resolve the requested runs to control.csv paths."""
    if use_all:
        return [r / "control.csv" for r in list_runs() if (r / "control.csv").is_file()]
    return [resolve_stream(s, "control") for s in specs]


def extract_steady(df, settle_s: float, speed_min: float):
    """Return the steady-state rows of one run as (cmd_speed, measured_speed, pwm) arrays."""
    needed = {"cmd_speed", "measured_speed", "pwm", "target_speed", "t_mono"}
    if not needed.issubset(df.columns) or df["measured_speed"].notna().sum() == 0:
        return np.array([]), np.array([]), np.array([])

    target = float(df["target_speed"].iloc[-1])
    plateau = df["cmd_speed"] >= target - 1e-3          # ramp has reached the target
    if not plateau.any():
        return np.array([]), np.array([]), np.array([])

    t_plateau = df.loc[plateau, "t_mono"].iloc[0]
    mask = (
        plateau
        & (df["t_mono"] >= t_plateau + settle_s)         # let speed settle after the ramp
        & (df["measured_speed"] > speed_min)             # actually moving
        & df["measured_speed"].notna()
    )
    g = df.loc[mask]
    return g["cmd_speed"].to_numpy(), g["measured_speed"].to_numpy(), g["pwm"].to_numpy()


def fit_and_report(cal: SpeedModelCalibrator, degree: int):
    """Fit affine (degree 1) and a higher-degree polynomial; print config-ready params."""
    moving = [s for s in cal.samples if s.gnss_mps > 0.0]
    if len(moving) < degree + 1:
        print(f"\n  Not enough moving samples ({len(moving)}) to fit degree {degree}.")
        return None
    speed = np.array([s.gnss_mps for s in moving])
    pwm = np.array([s.pwm for s in moving])

    def r2(coeffs):
        resid = pwm - np.polyval(coeffs, speed)
        ss_res = float(np.sum(resid ** 2))
        ss_tot = float(np.sum((pwm - pwm.mean()) ** 2))
        return (1 - ss_res / ss_tot) if ss_tot > 0 else float("nan"), float(np.std(resid))

    lin = np.polyfit(speed, pwm, 1)
    lin_r2, lin_res = r2(lin)
    # Residual expressed as a speed error (µs / (µs per m/s)) is more interpretable than raw µs.
    lin_res_mps = lin_res / abs(lin[0]) if lin[0] != 0 else float("nan")
    print("\nAffine model  pwm = gain * speed + bias  (for [speed_model] gain/bias):")
    print(f"    gain = {lin[0]:.4f}   bias = {lin[1]:.2f}   R²={lin_r2:.4f}  "
          f"residual_std={lin_res:.2f} µs (~{lin_res_mps:.3f} m/s)  (n={len(moving)})")

    if degree >= 2:
        poly = np.polyfit(speed, pwm, degree)
        poly_r2, poly_res = r2(poly)
        coeffs = ", ".join(f"{c:.5g}" for c in poly)
        improvement = (lin_res - poly_res) / lin_res if lin_res > 0 else 0.0
        print(f"\nPolynomial (degree {degree})  (for [speed_model] coefficients, highest degree first):")
        print(f"    coefficients = [{coeffs}]   R²={poly_r2:.4f}  residual_std={poly_res:.2f} µs")
        # Only recommend a nonlinear model when the affine fit has a physically meaningful
        # residual (> ~5 cm/s) AND the polynomial cuts it substantially - otherwise the
        # "improvement" is just fitting measurement noise on an already-linear relationship.
        if lin_res_mps < 0.05:
            verdict = "affine is sufficient (linear residual already < 5 cm/s)"
        elif improvement > 0.15:
            verdict = f"polynomial worth considering (cuts residual {100*improvement:.0f}%)"
        else:
            verdict = f"affine looks sufficient (polynomial only cuts residual {100*improvement:.0f}%)"
        print(f"    -> {verdict}")
        return lin, poly
    return lin, None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run dirs / control.csv paths / run-name substrings")
    ap.add_argument("--all", action="store_true", help="use every telemetry run that has a control.csv")
    ap.add_argument("--settle", type=float, default=2.0, help="seconds to skip after the ramp plateaus (default 2)")
    ap.add_argument("--speed-min", type=float, default=0.1, help="min measured speed [m/s] to count as moving")
    ap.add_argument("--reduce", choices=["all", "mean"], default="all",
                    help="'all' keeps every steady sample; 'mean' uses one averaged point per run")
    ap.add_argument("--out", default=None, help="combined calibration CSV (default: logs/calibration.csv)")
    ap.add_argument("--fit", action="store_true", help="fit and compare affine vs polynomial")
    ap.add_argument("--degree", type=int, default=2, help="polynomial degree to compare against affine (default 2)")
    ap.add_argument("--save", metavar="DIR", help="save the plot to this directory/file")
    ap.add_argument("--no-show", action="store_true", help="don't open an interactive window")
    args = ap.parse_args()

    if not args.runs and not args.all:
        ap.error("specify one or more runs, or --all")

    csvs = gather_control_csvs(args.runs, args.all)
    if not csvs:
        print("No control.csv files found for the given runs.")
        return

    cal = SpeedModelCalibrator()
    print(f"Building calibration from {len(csvs)} run(s):")
    for path in csvs:
        cmd, meas, pwm = extract_steady(load_stream(path), args.settle, args.speed_min)
        if len(meas) == 0:
            print(f"  {path.parent.name:<40} no steady-state samples (skipped)")
            continue
        if args.reduce == "mean":
            cal.record(setpoint_mps=float(cmd.mean()), gnss_mps=float(meas.mean()), pwm=float(pwm.mean()))
            print(f"  {path.parent.name:<40} 1 point  (cmd={cmd.mean():.2f}  meas={meas.mean():.2f} m/s  pwm={pwm.mean():.0f})")
        else:
            for c, m, p in zip(cmd, meas, pwm):
                cal.record(setpoint_mps=float(c), gnss_mps=float(m), pwm=float(p))
            print(f"  {path.parent.name:<40} {len(meas)} samples (meas {meas.min():.2f}-{meas.max():.2f} m/s)")

    if not cal.samples:
        print("\nNo usable samples extracted. Try a smaller --settle or --speed-min, or check the runs are moving.")
        return

    out = Path(args.out) if args.out else SpeedModelCalibrator.DEFAULT_PATH
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()  # overwrite so re-running is idempotent rather than appending duplicates
    cal.save(out)
    print(f"\nWrote {len(cal.samples)} samples -> {out}")

    if args.fit:
        fits = fit_and_report(cal, args.degree)
        if fits:
            lin, poly = fits
            speed = np.array([s.gnss_mps for s in cal.samples])
            pwm = np.array([s.pwm for s in cal.samples])
            fig, ax = plt.subplots(figsize=(9, 5))
            fig.suptitle("Speed-model calibration")
            ax.scatter(speed, pwm, s=10, alpha=0.4, label="steady-state samples")
            xs = np.linspace(speed.min(), speed.max(), 200)
            ax.plot(xs, np.polyval(lin, xs), color="red", label=f"affine (gain={lin[0]:.2f})")
            if poly is not None:
                ax.plot(xs, np.polyval(poly, xs), color="green", ls="--", label=f"poly deg {args.degree}")
            ax.set(xlabel="measured speed [m/s]", ylabel="PWM [µs]")
            ax.legend()
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            finish_plot(fig, args.save, not args.no_show, "speed_model_calibration.png")
    print()


if __name__ == "__main__":
    main()
