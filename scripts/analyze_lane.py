#!/usr/bin/env python3
"""Analyze lane-keeper telemetry (the 'lane' stream from LANE_KEEPER_TEST mode).

Plots the two control loops' setpoints vs measured values (heading toward the vanishing
point, and lateral lane-center position) plus the steering output, and reports tracking
error and jumpiness.

Usage:
    python scripts/analyze_lane.py                         # latest LANE_KEEPER_TEST run
    python scripts/analyze_lane.py logs/telemetry/<run_dir>
    python scripts/analyze_lane.py path/to/lane.csv --save out/ --no-show
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
    ap.add_argument("path", nargs="?", help="run dir, lane.csv, or run-name substring (default: latest)")
    ap.add_argument("--save", metavar="DIR", help="save plot to this directory/file")
    ap.add_argument("--no-show", action="store_true", help="don't open an interactive window")
    args = ap.parse_args()

    csv_path = resolve_stream(args.path, "lane")
    df = load_stream(csv_path)
    print(f"\nLane-keeper telemetry: {csv_path}  ({len(df)} ticks)\n")
    if len(df) < 2:
        print("  Not enough samples to analyze.\n")
        return

    seen = df[df["lanes"] == True] if "lanes" in df.columns else df
    print(f"Lanes detected: {len(seen)}/{len(df)} ticks ({100*len(seen)/len(df):.0f}%)")
    if len(seen):
        head_err = seen["heading"] - seen["heading_sp"]
        pos_err = seen["x_center"] - seen["x_center_sp"]
        print("Heading loop (toward vanishing point):")
        print(f"    error: mean={head_err.mean():+.2f}°  std={head_err.std():.2f}°  max|e|={head_err.abs().max():.2f}°")
        print("Position loop (lane center offset):")
        print(f"    error: mean={pos_err.mean():+.1f}px  std={pos_err.std():.1f}px  max|e|={pos_err.abs().max():.1f}px")
        steer = seen["steering"]
        print(f"Steering: mean={steer.mean():.1f}°  std={steer.std():.2f}°  range=[{steer.min():.1f}, {steer.max():.1f}]  (90°=straight)")

    # --- Plots --------------------------------------------------------------
    t = df["t_rel"].to_numpy()
    # Mask values to ticks where lanes were detected so 'no lane' holds don't distort lines.
    mask = (df["lanes"] == True) if "lanes" in df.columns else np.ones(len(df), bool)
    def m(col):
        return df[col].where(mask)

    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(f"Lane keeper  -  {csv_path.parent.name}")

    axs[0].plot(t, df["heading_sp"], color="black", ls="--", label="setpoint")
    axs[0].plot(t, m("heading"), color="tab:blue", label="heading")
    axs[0].set(ylabel="heading [°]", title="Heading loop (setpoint vs measured)")
    axs[0].legend(); axs[0].grid(True, alpha=0.3)

    axs[1].plot(t, m("x_center_sp"), color="black", ls="--", label="setpoint (image center)")
    axs[1].plot(t, m("x_center"), color="tab:blue", label="lane center")
    axs[1].set(ylabel="x [px]", title="Position loop (setpoint vs measured)")
    axs[1].legend(); axs[1].grid(True, alpha=0.3)

    axs[2].axhline(90, color="grey", ls="--", lw=1, label="straight (90°)")
    axs[2].plot(t, df["steering"], color="tab:red", label="steering")
    if "ang_corr" in df.columns:
        axs[2].plot(t, m("ang_corr"), alpha=0.5, label="angle corr")
    if "pos_corr" in df.columns:
        axs[2].plot(t, m("pos_corr"), alpha=0.5, label="position corr")
    axs[2].set(xlabel="time [s]", ylabel="steering [°]", title="Steering output")
    axs[2].legend(); axs[2].grid(True, alpha=0.3)

    fig.tight_layout()
    finish_plot(fig, args.save, not args.no_show, f"{csv_path.parent.name}_lane.png")
    print()


if __name__ == "__main__":
    main()
