#!/usr/bin/env python3
"""Analyze camera acquisition telemetry (the 'camera_front' / 'camera_back' streams).

Answers "is the camera grabbing at the set FPS?" and quantifies dropped frames, grab
failures, and per-frame processing cost. If a run contains both cameras, they are analyzed
side by side (useful for the 1-vs-2-camera comparison: compare a back-only run against a
combined run).

  - Achieved FPS:    from the grab-to-grab interval (dt_grab_ms), vs target_fps.
  - Dropped frames:  from the camera-clock interval (dt_cam_ms); a gap of ~k frame periods
                     means k-1 frames were dropped by the SDK before we grabbed.
  - proc_ms:         post-grab processing (retrieve + body tracking); if it approaches the
                     frame period, processing is the bottleneck.

Usage:
    python scripts/analyze_camera.py                       # latest run with a camera stream
    python scripts/analyze_camera.py logs/telemetry/2026-06-16_21-00-00_camera_test
    python scripts/analyze_camera.py --stream camera_back path/to/run --save out/ --no-show
"""

import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from telemetry_utils import load_stream, resolve_stream, list_runs, finish_plot


def _analyze_one(name: str, df) -> dict:
    """Print a report for one camera stream and return series needed for plotting."""
    frames = df[df["grab_status"] == "SUCCESS"] if "grab_status" in df.columns else df
    n = len(frames)
    print(f"\n=== {name} ===  ({n} frames)")
    if n < 2:
        print("  Not enough frames to analyze.")
        return {}

    target = frames["target_fps"].dropna()
    target_fps = float(target.iloc[0]) if not target.empty else float("nan")

    # Achieved FPS from grab cadence, and overall from the seq span / wall time.
    dt_grab = frames["dt_grab_ms"].dropna().to_numpy()
    achieved = 1000.0 / dt_grab.mean() if len(dt_grab) else float("nan")
    span_s = frames["t_mono"].iloc[-1] - frames["t_mono"].iloc[0]
    overall = (n - 1) / span_s if span_s > 0 else float("nan")
    print(f"  target FPS:          {target_fps:.0f}")
    print(f"  achieved FPS:        {achieved:.2f} (mean of 1/dt_grab)   {overall:.2f} (frames/wall-time)")
    if np.isfinite(target_fps) and target_fps > 0:
        print(f"  -> reaching {100*achieved/target_fps:.0f}% of target")
    if len(dt_grab):
        print(f"  grab interval [ms]:  mean={dt_grab.mean():.2f}  p95={np.percentile(dt_grab,95):.2f}  max={dt_grab.max():.2f}")

    # Dropped frames from the camera clock: estimate frames missing per gap.
    if np.isfinite(target_fps) and target_fps > 0:
        period = 1000.0 / target_fps
        dt_cam = frames["dt_cam_ms"].dropna().to_numpy()
        if len(dt_cam):
            missing = np.maximum(0, np.round(dt_cam / period) - 1).astype(int)
            dropped = int(missing.sum())
            expected = n + dropped
            print(f"  dropped frames (est): {dropped}  ({100*dropped/max(expected,1):.2f}% of expected {expected})")

    # Grab failures
    if "grab_failures" in df.columns and df["grab_failures"].notna().any():
        print(f"  grab failures (cumulative max): {int(df['grab_failures'].max())}")
    if "grab_status" in df.columns:
        nbad = int((df["grab_status"] != "SUCCESS").sum())
        if nbad:
            print(f"  non-SUCCESS grab rows: {nbad}")

    # Processing time
    proc = frames["proc_ms"].dropna().to_numpy()
    if len(proc):
        print(f"  proc time [ms]:      mean={proc.mean():.2f}  p95={np.percentile(proc,95):.2f}  max={proc.max():.2f}")
        if np.isfinite(target_fps) and target_fps > 0 and proc.mean() > 1000.0 / target_fps:
            print(f"  !! mean proc ({proc.mean():.1f} ms) exceeds frame period ({1000/target_fps:.1f} ms) - processing-bound")

    return {"t_rel": frames["t_rel"].to_numpy(), "dt_grab": frames["dt_grab_ms"].to_numpy(),
            "target_fps": target_fps, "name": name}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", help="run dir, a camera CSV, or run-name substring (default: latest)")
    ap.add_argument("--stream", help="analyze only this stream (e.g. camera_front); default: all present")
    ap.add_argument("--save", metavar="DIR", help="save plot to this directory/file")
    ap.add_argument("--no-show", action="store_true", help="don't open an interactive window")
    args = ap.parse_args()

    # Resolve which streams to analyze. If --stream or a direct CSV is given, use that;
    # otherwise find every camera_*.csv in the resolved run directory.
    streams: list[tuple[str, Path]] = []
    if args.stream:
        p = resolve_stream(args.path, args.stream)
        streams.append((args.stream, p))
    else:
        probe = resolve_stream(args.path, "camera_front") if _has(args.path, "camera_front") else \
                resolve_stream(args.path, "camera_back")
        run_dir = probe.parent
        for csv in sorted(run_dir.glob("camera_*.csv")):
            streams.append((csv.stem, csv))

    results = []
    for name, path in streams:
        print(f"\nCamera telemetry: {path}")
        results.append(_analyze_one(name, load_stream(path)))

    results = [r for r in results if r]
    if results:
        fig, ax = plt.subplots(figsize=(12, 4))
        run_name = streams[0][1].parent.name
        fig.suptitle(f"Camera grab interval  -  {run_name}")
        for r in results:
            ax.plot(r["t_rel"], r["dt_grab"], lw=0.7, label=r["name"])
            if np.isfinite(r["target_fps"]) and r["target_fps"] > 0:
                ax.axhline(1000.0 / r["target_fps"], ls="--", lw=1, alpha=0.6,
                           label=f"{r['name']} target ({r['target_fps']:.0f} FPS)")
        ax.set(xlabel="time [s]", ylabel="grab interval [ms]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        finish_plot(fig, args.save, not args.no_show, f"{run_name}_camera.png")
    print()


def _has(arg, stream) -> bool:
    try:
        resolve_stream(arg, stream)
        return True
    except FileNotFoundError:
        return False


if __name__ == "__main__":
    main()
