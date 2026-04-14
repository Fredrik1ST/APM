'''
The brain that carries out the main logic of the Autonomous Pacemaker.

Main states:
- Idle: waiting for user input to start or configure parameters
- Config: allowing user to set parameters (e.g. pace profile, distance tolerance)
- Starting: triggered by user, initializing hardware and software components
- Running: main program loop started
    - Pace: following pace profile
    - Dist: maintaining distance to runner
    - Const: maintaining constant speed
- Stopping: triggered by user or error, stopping all components
- Stopped: same as idle, but indicates a successful run has completed
- Error: if stopped due to error, waiting for user to reset

'''

from enum import IntEnum

class State(IntEnum):
    IDLE = 0
    CONFIG = 10
    STARTING = 20
    RUNNING = 21
    RUNNING_PACE = 22
    RUNNING_DIST = 23
    RUNNING_CONST = 24
    STOPPING = 30
    FINISHED = 31
    ERROR = 60

class Mode(IntEnum):
    NONE = 0
    NORMAL = 1
    PACE_ONLY = 2
    DISTANCE_ONLY = 3
    CONSTANT_SPEED = 4
    CAMERA_TEST = 100
    CAMERA_TEST_FRONT = 101
    CAMERA_TEST_BACK = 102
    ETHERNET_TEST = 103