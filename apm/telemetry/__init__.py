'''Structured per-run telemetry capture for the APM.

See apm/telemetry/logger.py for the design and usage.'''

from .logger import TelemetryLogger, write_run_note

__all__ = ['TelemetryLogger', 'write_run_note']
