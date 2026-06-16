'''
An autonomous pacemaker (AKA a "rabbit") for helping
runners maintain a set pace profile on a running track.

Hardware:
    - ZED Box Orin NX 16GB used for:
        - Control logic (Python), 
        - Speed control and calibration (GNSS)
        - Vision processing (ZED SDK)

    - 2 ZED X stereo cameras:
        - In front for lane detection and steering
        - In back for tracking the runner and maintaining distance

    - Arrma Infraction 6S RC car with:
        - Electronic Speed Controller (ESC) for main motor
        - Steering Servo
        - Brake Servo

    - Arduino for controlling the ESC and servos via PWM, controlled by ZED Box via Ethernet

Main control loop:
    1. Wait for start signal from user (e.g. via button press or local web app)
       Allow user to configure parameters during this phase.

    2. Initialize hardware and software components needed for the selected mode

    3. Run main control loop:
        a. Capture images from front and back cameras (free-running camera threads)
        b. Process images to detect lane and runner position
        c. Calculate desired speed and steering angle based on lane and runner position
        d. Send control signals to ESC and servos
        e. Follow pace profile as long as runner is within certain distance or...
        f. Maintain distance if lagging behind until runner is back within pace profile

    4. Stop when user sends stop signal or if an error occurs
       If stopped due to error, wait for user to reset 

'''

import argparse
import logging
import socket
import sys
from datetime import datetime
from pathlib import Path

from apm import config_handler
from apm.orchestrator import Orchestrator
from apm.webapp.app import start as start_webapp

_LOG_DIR = Path(__file__).resolve().parent.parent / 'logs'

def _setup_logging(config) -> None:
    """Write logs to file and console, applying per-module levels from config."""

    # Each run gets its own timestamped log file so runs don't overwrite each other
    _LOG_DIR.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_file = _LOG_DIR / f'{timestamp}.log'

    # Read logging config; fall back to DEBUG so nothing is silenced by default
    log_cfg = config.get('logging', {})
    root_level = getattr(logging, log_cfg.get('level', 'DEBUG').upper(), logging.DEBUG)

    # basicConfig wires up the root logger once. All loggers in the process inherit this.
    logging.basicConfig(
        level=root_level,
        format='%(asctime)s  %(levelname)-8s  %(name)s  %(message)s',
        handlers=[
            logging.FileHandler(log_file),   # persists to disk
            logging.StreamHandler(),         # echoes to terminal
        ],
    )

    # Override verbosity for specific modules (e.g. silence a chatty library).
    # config["logging"]["levels"] is a dict like {"some.module": "WARNING"}
    for module, level_str in log_cfg.get('levels', {}).items():
        level = getattr(logging, level_str.upper(), None)
        if level is not None:
            logging.getLogger(module).setLevel(level)

    logging.info(f'Logging to {log_file}')


def _parse_args() -> argparse.Namespace:
    """Parse CLI args. argparse handles --help by printing usage and exiting,
    so `python -m apm --help` no longer falls through and boots the app."""
    parser = argparse.ArgumentParser(
        prog='apm',
        description='Autonomous Pacemaker - paces a runner on a track via an RC car '
                    'and a web UI on http://<host>:<port>.',
    )
    parser.add_argument('--host', default=None,
                        help='Web UI bind address (overrides [webapp].host in config).')
    parser.add_argument('--port', type=int, default=None,
                        help='Web UI port (overrides [webapp].port in config).')
    return parser.parse_args()


def _check_port_free(host: str, port: int) -> bool:
    """Return True if (host, port) can be bound, so we can fail fast with a clear
    message instead of a raw bind traceback from the web server thread."""
    probe_host = '0.0.0.0' if host in ('', '0.0.0.0') else host
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind((probe_host, port))
            return True
        except OSError:
            return False


def main() -> None:
    args = _parse_args()
    config_handler.initialize()
    cfg = config_handler.load()
    _setup_logging(cfg)

    host = args.host if args.host is not None else cfg['webapp']['host']
    port = args.port if args.port is not None else int(cfg['webapp']['port'])

    if not _check_port_free(host, port):
        logging.error(f'Web UI port {host}:{port} is already in use - is another APM instance running? '
                      f'Stop it (or pass --port) and try again.')
        sys.exit(1)

    # The Orchestrator manages the main control loop, modes, and hardware communication
    orchestrator = Orchestrator()

    # The web app runs in a separate thread and allows the user to start/stop, select modes, and adjust config params
    start_webapp(orchestrator, host=host, port=port)

    # Point of no return. Time to start the main loop and run the APM!
    orchestrator.run()


if __name__ == '__main__':
    main()