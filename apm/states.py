'''
State enums for the Orchestrator.

Kept in their own module so modes can import `State` to report which controller they are running. 
Not part of the orchestrator to avoid circular imports.
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
    CONSTANT_SPEED_CLOSED = 5
    CAMERA_TEST = 100
    CAMERA_TEST_FRONT = 101
    CAMERA_TEST_BACK = 102
    ARDUINO_TEST = 200
    GNSS_TEST = 300
    LANE_KEEPER_TEST = 400
