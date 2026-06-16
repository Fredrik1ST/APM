#!/usr/bin/env python3
"""Analyze Arduino round-trip telemetry (the 'arduino' stream from arduino_test mode).

Reports round-trip latency (send -> feedback) distribution, overall and per test phase,
and checks feedback liveness (how often the feedback message number failed to advance,
i.e. comms stalled).

Usage:
    python scripts/analyze_arduino.py                       # latest run with arduino.csv
    python scripts/analyze_arduino.py logs/telemetry/2026-06-16_19-30-00_arduino_test
    python scripts/analyze_arduino.py path/to/arduino.csv --save logs/telemetry/plots/
    python scripts/analyze_arduino.py --no-show --save out/
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
    ap.add_argument("path", nargs="?", help="run dir, arduino.csv, or run-name substring (default: latest)")
    ap.add_argument("--save", metavar="DIR", help="save plot to this directory/file instead of only showing")
    ap.add_argument("--no-show", action="store_true", help="don't open an interactive window")
    args = ap.parse_args()

    csv_path = resolve_stream(args.path, "arduino")
    df = load_stream(csv_path)
    print(f"\nArduino telemetry: {csv_path}  ({len(df)} samples)\n")

    # --- Round-trip latency -------------------------------------------------
    rtt = df["rtt_ms"].dropna()
    rtt = rtt[rtt > 0]  # 0 means no feedback had been paired yet (startup)
    if rtt.empty:
        print("  No valid round-trip samples (rtt_ms all zero/empty).")
    else:
        print("Round-trip latency (ms):")
        print(f"    n={len(rtt)}  mean={rtt.mean():.2f}  median={rtt.median():.2f}  "
              f"std={rtt.std():.2f}")
        print(f"    min={rtt.min():.2f}  p95={rtt.quantile(0.95):.2f}  "
              f"p99={rtt.quantile(0.99):.2f}  max={rtt.max():.2f}")

        if "phase" in df.columns:
            print("\n  By phase (mean / p95 / max ms):")
            for phase, g in df.groupby("phase", sort=False):
                r = g["rtt_ms"].dropna()
                r = r[r > 0]
                if not r.empty:
                    print(f"    {str(phase):<14} mean={r.mean():6.2f}  p95={r.quantile(0.95):6.2f}  max={r.max():6.2f}")

    # --- Feedback liveness / stalls ----------------------------------------
    if "fb_nr" in df.columns:
        fb = df["fb_nr"].to_numpy()
        advanced = np.diff(fb)
        stalls = int(np.sum(advanced == 0))
        backwards = int(np.sum(advanced < 0))
        print(f"\nFeedback liveness ({len(df)} ticks):")
        print(f"    ticks where fb_nr did not advance (stalled): {stalls} "
              f"({100*stalls/max(len(advanced),1):.1f}%)")
        if backwards:
            print(f"    fb_nr went backwards {backwards} time(s) (Arduino reset / reconnect?)")

    # --- Plot ---------------------------------------------------------------
    if not rtt.empty:
        x = df.loc[rtt.index, "t_rel"] if "t_rel" in df.columns else rtt.index
        fig, (ax_ts, ax_hist) = plt.subplots(1, 2, figsize=(12, 4))
        fig.suptitle(f"Arduino round-trip latency  -  {csv_path.parent.name}")

        ax_ts.plot(x, rtt, lw=0.8)
        ax_ts.axhline(rtt.median(), color="red", ls="--", lw=1, label=f"median {rtt.median():.2f} ms")
        ax_ts.set_xlabel("time [s]")
        ax_ts.set_ylabel("RTT [ms]")
        ax_ts.legend()
        ax_ts.grid(True, alpha=0.3)

        ax_hist.hist(rtt, bins=50)
        ax_hist.axvline(rtt.quantile(0.95), color="orange", ls="--", lw=1, label=f"p95 {rtt.quantile(0.95):.2f} ms")
        ax_hist.set_xlabel("RTT [ms]")
        ax_hist.set_ylabel("count")
        ax_hist.legend()
        fig.tight_layout()
        finish_plot(fig, args.save, not args.no_show, f"{csv_path.parent.name}_arduino_rtt.png")
    print()


if __name__ == "__main__":
    main()
