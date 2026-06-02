'''
Main loop of the GNSS Test mode.

Quite simple: verify that the GNSS receiver is alive and well by logging position and speed for a while.
'''

import time
import threading
import logging

import tomlkit
from apm.drivers.gnss import GNSSWrapper

log = logging.getLogger(__name__)

_LOG_INTERVAL = 1.0  # How often to print a position/speed reading [s]


def gnss_test_mode(gnss: GNSSWrapper, stop_event: threading.Event,
                   cfg: tomlkit.TOMLDocument) -> None:
    speed_threshold = cfg["gnss"]["speed_threshold"]

    log.info('GNSS fix acquired. Logging position and speed. Press Stop to exit.')

    last_log_time = 0.0
    max_speed = 0.0
    max_lon = 0.0
    min_lon = 0.0
    max_lat = 0.0
    min_lat = 0.0

    while not stop_event.is_set():
        now = time.monotonic()
        if now - last_log_time >= _LOG_INTERVAL:
            snap = gnss.get_snapshot()
            if snap is None:
                log.warning('No GNSS data yet.')
            else:
                moving = snap.speed >= speed_threshold
                log.info(
                    f'lat={snap.lat:.7f}  lon={snap.lon:.7f}  '
                    f'speed={snap.speed:.2f} m/s  '
                    f'{"(moving)" if moving else "(stationary)"}'
                )

                # Track the max values observed during the test, useful for detecting outliers at rest
                max_speed = max(max_speed, snap.speed)
                max_lon   = max(max_lon, snap.lon)
                min_lon   = min(min_lon, snap.lon)
                max_lat   = max(max_lat, snap.lat)
                min_lat   = min(min_lat, snap.lat)

            last_log_time = now
        stop_event.wait(0.1)

    
    log.info(f'Max speed during test: {max_speed:.2f} m/s')
    log.info(f'Max longitude difference during test: {max_lon - min_lon:.7f}°')
    log.info(f'Max latitude difference during test: {max_lat - min_lat:.7f}°')
    log.info('GNSS test complete.')