'''
Main loop of the GNSS Test mode.

Quite simple: verify that the GNSS receiver is alive and well by logging position and speed for a while.
'''

import time
import threading
import logging

import tomlkit

from apm.drivers.gnss import GNSSDriver
from apm.telemetry import TelemetryLogger

log = logging.getLogger(__name__)

_LOG_INTERVAL = 1.0  # How often to print a position/speed reading [s]


def gnss_test(gnss: GNSSDriver, stop_event: threading.Event,
                   cfg: tomlkit.TOMLDocument) -> None:

    log.info('GNSS fix acquired. Logging position and speed. Press Stop to exit.')

    last_log_time = 0.0
    max_speed = 0.0
    lon_lo = lon_hi = None  # Position range, seeded from the first valid fix
    lat_lo = lat_hi = None

    # Attach a telemetry sink so the driver records every fix (speed + lat/lon) at the
    # true GNSS rate for the whole run. The 1 Hz log below stays for live monitoring.
    with TelemetryLogger('gnss_test') as tlm:
        gnss.set_telemetry(tlm)
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                if now - last_log_time >= _LOG_INTERVAL:
                    snap = gnss.get_snapshot()
                    if snap is None:
                        log.warning('No GNSS data yet.')
                    else:
                        moving = snap.speed >= gnss.speed_threshold
                        log.info(
                            f'lat={snap.lat:.7f}  lon={snap.lon:.7f}  '
                            f'speed={snap.speed:.2f} m/s  '
                            f'{"(moving)" if moving else "(stationary)"}'
                        )

                        # Track the range observed during the test (useful for spotting
                        # position scatter at rest). Seeded on first fix so 0,0 isn't included.
                        max_speed = max(max_speed, snap.speed)
                        lon_lo = snap.lon if lon_lo is None else min(lon_lo, snap.lon)
                        lon_hi = snap.lon if lon_hi is None else max(lon_hi, snap.lon)
                        lat_lo = snap.lat if lat_lo is None else min(lat_lo, snap.lat)
                        lat_hi = snap.lat if lat_hi is None else max(lat_hi, snap.lat)

                    last_log_time = now
                stop_event.wait(0.1)
        finally:
            gnss.set_telemetry(None)

    log.info(f'Max speed during test: {max_speed:.2f} m/s')
    if lon_lo is not None:
        log.info(f'Longitude range during test: {lon_hi - lon_lo:.7f}°')
        log.info(f'Latitude range during test: {lat_hi - lat_lo:.7f}°')
    log.info('GNSS test complete.')