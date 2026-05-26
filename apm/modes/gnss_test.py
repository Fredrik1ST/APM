'''
Main loop of the GNSS Test mode.

Quite simple: verify that the GNSS receiver is alive and well by logging position and speed for a while.
'''

import time
import threading
import logging

import tomlkit
from gnss import GNSSWrapper

log = logging.getLogger(__name__)

_LOG_INTERVAL = 1.0  # How often to print a position/speed reading [s]


def gnss_test_mode(gnss: GNSSWrapper, stop_event: threading.Event,
                   cfg: tomlkit.TOMLDocument) -> None:
    speed_threshold = cfg["gnss"]["speed_threshold"]

    log.info('GNSS fix acquired. Logging position and speed. Press Stop to exit.')

    last_log = 0.0

    while not stop_event.is_set():
        now = time.monotonic()
        if now - last_log >= _LOG_INTERVAL:
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
            last_log = now
        stop_event.wait(0.1)

    log.info('GNSS test complete.')