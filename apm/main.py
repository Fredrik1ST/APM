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

import logging
from datetime import datetime
from pathlib import Path
from orchestrator import Orchestrator

_LOG_DIR = Path(__file__).resolve().parent.parent / 'logs'

def _setup_logging() -> None:
    """Write logs to file and console"""
    _LOG_DIR.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_file = _LOG_DIR / f'{timestamp}.log'

    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s  %(levelname)-8s  %(name)s  %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(),   # keep console output
        ],
    )
    logging.info(f'Logging to {log_file}')

if __name__ == '__main__':
    _setup_logging()
    Orchestrator().run()