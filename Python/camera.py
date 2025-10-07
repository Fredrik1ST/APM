#!/usr/bin/env python3
 
import pyzed.sl as sl
import time
import numpy as np
import cv2
 
def camera_init(fps=30, resolution=sl.RESOLUTION.SVGA):
    init = sl.InitParameters()
    camera = sl.Camera()
    if not camera.is_opened():
        print("Opening ZED Camera...")
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit(0)
 
    mat = sl.Mat()
 
    runtime = sl.RuntimeParameters()
 
    init = sl.InitParameters()
    init.camera_resolution = resolution
    init.camera_fps = fps  # The framerate is lowered to avoid any USB3 bandwidth issues
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE #[X, Y, Z] Z FWD
    camera = sl.Camera()
    cam_status = camera.open(init)
 
    return mat, camera, runtime, cam_status
 
def get_camera_image(mat, camera, runtime):    # Obtain camera image and get timestamps
    err = camera.grab(runtime)
    # If grabbing image successfull, save to buffer
    if err == sl.ERROR_CODE.SUCCESS:
        # camera.retrieve_image(mat, sl.VIEW.SIDE_BY_SIDE)
        camera.retrieve_image(mat, sl.VIEW.LEFT)
        img = mat.get_data()
        return img
    else:
        print("ERROR")
        return None