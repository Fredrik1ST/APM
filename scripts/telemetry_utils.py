"""Shared helpers for the offline telemetry analysis scripts.

Telemetry is written by apm/telemetry/logger.py as one CSV per stream under
logs/telemetry/<timestamp>_<run>/<stream>.csv. Every row starts with t_mono (monotonic
seconds at log time) and t_iso (wall clock); the remaining columns are stream-specific.

These helpers locate the run/stream to analyze and load it into a pandas DataFrame.
"""

from pathlib import Path

import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TELEMETRY_DIR = PROJECT_ROOT / "logs" / "telemetry"


def load_stream(path: Path) -> pd.DataFrame:
    """Load one telemetry CSV into a DataFrame, with t_rel (seconds since first sample)
    added for convenience. Blank cells become NaN (pandas default)."""
    df = pd.read_csv(path)
    if "t_mono" in df.columns and len(df):
        df["t_rel"] = df["t_mono"] - df["t_mono"].iloc[0]
    return df


def list_runs() -> list[Path]:
    """All run directories under logs/telemetry, oldest first."""
    if not TELEMETRY_DIR.is_dir():
        return []
    return sorted(p for p in TELEMETRY_DIR.iterdir() if p.is_dir())


def resolve_stream(arg: str | None, stream: str) -> Path:
    """Find the CSV for `stream` (e.g. 'gnss', 'camera_front').

    `arg` may be:
      - None          -> latest run directory that contains <stream>.csv
      - a .csv file   -> used directly
      - a run dir     -> <dir>/<stream>.csv
      - a substring   -> latest run directory whose name contains it and has <stream>.csv
    """
    if arg is not None:
        p = Path(arg)
        if p.is_file():
            return p
        if p.is_dir():
            candidate = p / f"{stream}.csv"
            if candidate.is_file():
                return candidate
            raise FileNotFoundError(f"{candidate} not found")
        matches = [r for r in list_runs() if arg in r.name and (r / f"{stream}.csv").is_file()]
        if matches:
            return matches[-1] / f"{stream}.csv"
        raise FileNotFoundError(f"No run matching '{arg}' contains {stream}.csv")

    for run in reversed(list_runs()):
        candidate = run / f"{stream}.csv"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        f"No telemetry run under {TELEMETRY_DIR} contains {stream}.csv. "
        f"Run the corresponding mode first, or pass an explicit path."
    )


def finish_plot(fig, save: str | None, show: bool, default_name: str) -> None:
    """Save and/or show a figure, or close it if neither (e.g. headless batch runs)."""
    if save:
        out = Path(save)
        if out.is_dir() or save.endswith("/"):
            out = out / default_name
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=150)
        print(f"  plot saved -> {out}")
    if show:
        import matplotlib.pyplot as plt
        plt.show()
    if not show:
        import matplotlib.pyplot as plt
        plt.close(fig)
