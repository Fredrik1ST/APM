'''
Structured, per-run telemetry capture for the APM.

Where the standard `logging` module produces human-readable lines on a ~1 Hz cadence,
this module captures machine-analyzable samples at full loop / frame rate so tests can be
quantified offline (round-trip latency, dropped frames, GNSS scatter, tracking error, ...).

Design mirrors the SpeedModelCalibrator's CSV approach (apm/speed_models/calibrator.py):

    - One TelemetryLogger per run. It owns a timestamped directory under logs/telemetry/.
    - Each named "stream" (e.g. "arduino", "gnss", "control") becomes one CSV file in that
      directory. Streams share the run directory so they can be joined offline on t_mono.
    - Columns are fixed from the first row written to a stream. Every row gets t_mono
      (monotonic seconds) and t_iso (wall clock) prepended automatically; callers should
      additionally include the source timestamp from the relevant snapshot/message so
      samples can be aligned to when the data was actually produced, not just logged.

Usage::

    tlm = TelemetryLogger('arduino_test')
    tlm.log('arduino', {'sent_nr': msg.message_nr, 'fb_nr': fb.message_nr, 'speed_pwm': pwm})
    ...
    tlm.close()   # flushes and closes all stream files

    # Offline:
    import pandas as pd
    df = pd.read_csv('logs/telemetry/2026-06-16_19-30-00_arduino_test/arduino.csv')
'''

import csv
import time
import logging
import threading
from datetime import datetime
from pathlib import Path

log = logging.getLogger(__name__)

TELEMETRY_DIR = Path(__file__).resolve().parent.parent.parent / 'logs' / 'telemetry'

# Auto-added to every row so independent streams can be aligned offline.
_T_MONO = 't_mono'   # monotonic seconds at log() time - use for intervals / alignment
_T_ISO  = 't_iso'    # wall-clock ISO 8601 at log() time - use for human-readable reference


def write_run_note(text: str) -> Path | None:
    '''Write a free-text note into the most recent telemetry run directory.

    Run directories are timestamp-prefixed, so the newest one is the active/just-finished
    run. Returns the note path, or None if the text is empty or no run directory exists.
    Used by the web app to attach an optional note to a run when it is stopped.
    '''
    text = text.strip()
    if not text:
        return None
    try:
        dirs = [d for d in TELEMETRY_DIR.iterdir() if d.is_dir()]
    except FileNotFoundError:
        return None
    if not dirs:
        return None
    latest = max(dirs, key=lambda d: d.name)
    note_path = latest / 'note.txt'
    note_path.write_text(text.rstrip('\n') + '\n')
    log.info('Run note written to %s', note_path)
    return note_path


class _Stream:
    '''A single CSV file. Header is fixed on the first row; later rows are written in that
    column order, missing keys left blank and unknown keys dropped (with a one-time warning).'''

    def __init__(self, path: Path):
        self._path = path
        self._file = open(path, 'w', newline='')
        self._writer: csv.DictWriter | None = None
        self._warned_extra = False

    def write(self, row: dict) -> None:
        if self._writer is None:
            # Row already carries _T_MONO / _T_ISO first (inserted by TelemetryLogger.log).
            self._writer = csv.DictWriter(self._file, fieldnames=list(row.keys()), extrasaction='ignore')
            self._writer.writeheader()
        elif not self._warned_extra:
            extra = set(row) - set(self._writer.fieldnames)
            if extra:
                log.warning('Telemetry stream %s: dropping columns not in header: %s',
                            self._path.name, sorted(extra))
                self._warned_extra = True
        self._writer.writerow(row)

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()


class TelemetryLogger:
    '''Owns one run directory and lazily creates a CSV per stream.

    Thread-safe: drivers may log from their background threads while a mode logs from the
    main loop. The directory is created on construction so its path is available immediately.

    Args:
        run_name: Short label for the run (typically the mode name). Used in the directory name.
        flush_interval: Seconds between automatic flushes to disk (0 to flush every row).
    '''

    def __init__(self, run_name: str, flush_interval: float = 1.0):
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.run_dir = TELEMETRY_DIR / f'{timestamp}_{run_name}'
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self._streams: dict[str, _Stream] = {}
        self._lock = threading.Lock()
        self._flush_interval = flush_interval
        self._last_flush = time.monotonic()
        self._closed = False
        log.info('Telemetry -> %s', self.run_dir)

    def log(self, stream: str, row: dict) -> None:
        '''Append one sample to the named stream. t_mono / t_iso are added automatically.'''
        now = time.monotonic()
        full = {_T_MONO: now, _T_ISO: datetime.now().isoformat(timespec='milliseconds')}
        full.update(row)

        with self._lock:
            if self._closed:
                return
            s = self._streams.get(stream)
            if s is None:
                s = _Stream(self.run_dir / f'{stream}.csv')
                self._streams[stream] = s
            s.write(full)

            if self._flush_interval == 0 or now - self._last_flush >= self._flush_interval:
                for st in self._streams.values():
                    st.flush()
                self._last_flush = now

    def close(self) -> None:
        '''Flush and close all stream files. Safe to call more than once.'''
        with self._lock:
            if self._closed:
                return
            for s in self._streams.values():
                s.close()
            self._closed = True
        log.info('Telemetry closed (%d stream(s) in %s)', len(self._streams), self.run_dir)

    def __enter__(self) -> 'TelemetryLogger':
        return self

    def __exit__(self, *exc) -> None:
        self.close()
