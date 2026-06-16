#!/usr/bin/env python3
"""Analyze GNSS telemetry (the 'gnss' stream, logged for the whole run by the GNSS driver).

Reports:
  - Actual delivered update rate (from the receiver clock and from arrival time) vs the
    configured rate -- use this to compare 10 Hz vs 15 Hz settings.
  - Position spread in metres (at rest this is the position noise / accuracy) and eph/epv.
  - Displacement two ways: integrating Doppler speed (speed*dt) vs differentiating position
    (haversine between consecutive fixes). These should agree; divergence is informative.

Usage:
    python scripts/analyze_gnss.py                         # latest run with gnss.csv
    python scripts/analyze_gnss.py logs/telemetry/2026-06-16_20-00-00_gnss_test
    python scripts/analyze_gnss.py path/to/gnss.csv --save out/ --no-show
"""

import sys
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from telemetry_utils import load_stream, resolve_stream, finish_plot

_EARTH_R = 6_371_000.0  # m
_DEG_TO_M = np.pi / 180.0 * _EARTH_R  # metres per degree of latitude


def _haversine_m(lat, lon):
    """Distance [m] between consecutive (lat, lon) points (degrees). Returns len-1 array."""
    phi = np.radians(lat)
    dphi = np.radians(np.diff(lat))
    dlam = np.radians(np.diff(lon))
    a = np.sin(dphi / 2) ** 2 + np.cos(phi[:-1]) * np.cos(phi[1:]) * np.sin(dlam / 2) ** 2
    return 2 * _EARTH_R * np.arcsin(np.sqrt(a))


def _rate_summary(label, intervals_s):
    iv = intervals_s[np.isfinite(intervals_s) & (intervals_s > 0)]
    if len(iv) == 0:
        print(f"    {label}: no data")
        return
    print(f"    {label}: mean={1/iv.mean():.2f} Hz  median={1/np.median(iv):.2f} Hz  "
          f"(interval mean={1e3*iv.mean():.1f} ms, std={1e3*iv.std():.1f} ms)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", help="run dir, gnss.csv, or run-name substring (default: latest)")
    ap.add_argument("--save", metavar="DIR", help="save plot to this directory/file")
    ap.add_argument("--no-show", action="store_true", help="don't open an interactive window")
    args = ap.parse_args()

    csv_path = resolve_stream(args.path, "gnss")
    df = load_stream(csv_path).dropna(subset=["lat", "lon", "speed"]).reset_index(drop=True)
    print(f"\nGNSS telemetry: {csv_path}  ({len(df)} fixes)\n")
    if len(df) < 2:
        print("  Not enough fixes to analyze.\n")
        return

    configured = df["rate_hz"].dropna()
    if not configured.empty:
        print(f"Configured rate: {configured.iloc[0]:.0f} Hz")

    print("Actual update rate:")
    _rate_summary("receiver clock (gnss_ts_ms)", np.diff(df["gnss_ts_ms"].to_numpy()) / 1e3)
    _rate_summary("arrival (t_mono)          ", np.diff(df["t_mono"].to_numpy()))

    # --- Position spread (at rest => noise / accuracy) ----------------------
    lat0 = df["lat"].mean()
    spread_lat_m = df["lat"].std() * _DEG_TO_M
    spread_lon_m = df["lon"].std() * _DEG_TO_M * np.cos(np.radians(lat0))
    print("\nPosition spread (std; at rest this is position noise):")
    print(f"    lat={spread_lat_m:.3f} m   lon={spread_lon_m:.3f} m   "
          f"horizontal={np.hypot(spread_lat_m, spread_lon_m):.3f} m")
    for col in ("eph", "epv"):
        if col in df.columns and df[col].notna().any():
            print(f"    {col}: mean={df[col].mean():.3f} m  max={df[col].max():.3f} m")

    # --- Displacement: Doppler integration vs position differentiation ------
    # Integrate over the GNSS receiver clock (gnss_ts_ms), which is the authoritative time
    # base for the Doppler speed and is immune to logging/arrival jitter in t_mono.
    elapsed = (df["gnss_ts_ms"].to_numpy() - df["gnss_ts_ms"].iloc[0]) / 1e3  # seconds
    dt = np.diff(elapsed)
    speed = df["speed"].to_numpy()
    # trapezoidal integration of speed over time
    doppler_steps = 0.5 * (speed[1:] + speed[:-1]) * dt
    doppler_cum = np.concatenate([[0.0], np.cumsum(doppler_steps)])
    pos_cum = np.concatenate([[0.0], np.cumsum(_haversine_m(df["lat"].to_numpy(), df["lon"].to_numpy()))])
    print("\nDisplacement over run:")
    print(f"    Doppler-integrated (speed*dt):     {doppler_cum[-1]:.2f} m")
    print(f"    Position-differentiated (haversine): {pos_cum[-1]:.2f} m")
    if pos_cum[-1] > 0:
        print(f"    difference: {doppler_cum[-1] - pos_cum[-1]:+.2f} m "
              f"({100*(doppler_cum[-1] - pos_cum[-1])/pos_cum[-1]:+.1f}%)")

    # --- Plots --------------------------------------------------------------
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"GNSS analysis  -  {csv_path.parent.name}")

    axs[0, 0].plot(elapsed, speed, lw=0.8)
    axs[0, 0].set(title="Doppler speed", xlabel="time [s]", ylabel="speed [m/s]")
    axs[0, 0].grid(True, alpha=0.3)

    axs[0, 1].scatter(df["lon"], df["lat"], s=6, alpha=0.5)
    axs[0, 1].set(title="Track (lon/lat)", xlabel="longitude [°]", ylabel="latitude [°]")
    axs[0, 1].ticklabel_format(useOffset=False)
    axs[0, 1].grid(True, alpha=0.3)

    axs[1, 0].plot(elapsed, doppler_cum, label="Doppler ∫speed·dt")
    axs[1, 0].plot(elapsed, pos_cum, label="haversine path", ls="--")
    axs[1, 0].set(title="Cumulative displacement", xlabel="time [s]", ylabel="distance [m]")
    axs[1, 0].legend()
    axs[1, 0].grid(True, alpha=0.3)

    axs[1, 1].hist(np.diff(elapsed) * 1e3, bins=40)
    axs[1, 1].set(title="Inter-fix interval (receiver clock)", xlabel="interval [ms]", ylabel="count")
    fig.tight_layout()
    finish_plot(fig, args.save, not args.no_show, f"{csv_path.parent.name}_gnss.png")
    print()


if __name__ == "__main__":
    main()
