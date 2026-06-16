'''
Pacing profile: the target-speed schedule the cruise controller follows.

Loaded from a CSV of ``(speed [m/s], duration [s])`` rows, e.g.::

    speed (m/s), time (s)
    0.5, 1
    1.0, 30
    3.2, 120

A header row is optional and skipped automatically, as are blank lines, ``#``
comments and trailing commas. The profile is a *piecewise-constant* (step)
function of time: ``speed_at(t)`` returns the active segment's speed. Transitions
between segments are intentionally not interpolated here -- the cruise
controller's velocity ramp (see ``velocity_profiles.py``) smooths the steps so
the setpoint never jumps.
'''

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


PROFILES_DIR = Path(__file__).resolve().parents[2] / 'pacing_profiles'


def available_profiles(directory: str | Path = PROFILES_DIR) -> list[Path]:
    '''List the pacing-profile CSV files in `directory`, sorted by name.'''
    directory = Path(directory)
    if not directory.is_dir():
        return []
    return sorted(directory.glob('*.csv'))


@dataclass(frozen=True)
class Segment:
    speed: float        # target speed [m/s]
    duration: float     # how long to hold it [s]


class PacingProfile:
    '''An ordered list of constant-speed segments, indexed by elapsed time.'''

    def __init__(self, segments: Iterable[Segment]):
        self.segments = list(segments)
        if not self.segments:
            raise ValueError('Pacing profile has no segments.')
        self.total_duration = sum(s.duration for s in self.segments)

    @classmethod
    def from_rows(cls, rows: Iterable[Iterable[str]]) -> 'PacingProfile':
        '''Build a profile from raw CSV cells (header/blank/comment lines skipped).'''
        segments: list[Segment] = []
        for line_no, raw in enumerate(rows, start=1):
            cells = [c.strip() for c in raw if c.strip() != '']  # drop trailing-comma blanks
            if not cells or cells[0].startswith('#'):
                continue  # blank line or comment
            try:
                speed, duration = float(cells[0]), float(cells[1])
            except (ValueError, IndexError):
                if not segments:
                    continue  # header / preamble before any data -> skip
                raise ValueError(f'Malformed pacing-profile row {line_no}: {raw!r}')
            if speed < 0:
                raise ValueError(f'Negative speed on row {line_no}: {speed} m/s (no reverse).')
            if duration <= 0:
                raise ValueError(f'Non-positive duration on row {line_no}: {duration} s.')
            segments.append(Segment(speed, duration))
        return cls(segments)

    @classmethod
    def from_csv(cls, path: str | Path) -> 'PacingProfile':
        '''Load a profile from a CSV file.'''
        path = Path(path)
        with path.open(newline='', encoding='utf-8') as f:
            try:
                return cls.from_rows(csv.reader(f))
            except ValueError as e:
                raise ValueError(f'{path}: {e}') from e

    def speed_at(self, t: float) -> float:
        '''Target speed [m/s] at elapsed time ``t`` [s].

        Clamps to the first segment for t <= 0 and holds the last segment's speed
        once the profile is finished; use ``is_finished`` to detect the end.
        '''
        if t <= 0:
            return self.segments[0].speed
        elapsed = 0.0
        for s in self.segments:
            elapsed += s.duration
            if t < elapsed:
                return s.speed
        return self.segments[-1].speed

    def is_finished(self, t: float) -> bool:
        '''True once elapsed time has reached the end of the schedule.'''
        return t >= self.total_duration

    @property
    def total_distance(self) -> float:
        '''Total distance covered over the whole profile [m].'''
        return sum(s.speed * s.duration for s in self.segments)

    def distance_curve(self) -> tuple[list[float], list[float]]:
        '''Cumulative ``(times, distances)`` vertices for plotting distance over time.

        Distance is the integral of a piecewise-constant speed, so it is piecewise
        linear: the polyline through these segment-boundary vertices is exact.
        Starts at the origin ``(0, 0)``.
        '''
        times, distances = [0.0], [0.0]
        t = d = 0.0
        for s in self.segments:
            t += s.duration
            d += s.speed * s.duration
            times.append(t)
            distances.append(d)
        return times, distances

    def speed_curve(self) -> tuple[list[float], list[float]]:
        '''``(times, speeds)`` points describing the speed step function.

        Each segment contributes its start time; a final point at the end time
        closes the last step (use a step-style line when plotting).
        '''
        times, speeds = [], []
        t = 0.0
        for s in self.segments:
            times.append(t)
            speeds.append(s.speed)
            t += s.duration
        times.append(t)
        speeds.append(self.segments[-1].speed)
        return times, speeds
