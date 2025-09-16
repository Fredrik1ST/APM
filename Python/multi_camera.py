########################################################################
#
# Copyright (c) 2020, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    Multi cameras sample showing how to open multiple ZED in one program
"""

import pyzed.sl as sl
import cv2
import numpy as np
import threading
import time
import signal

zed_list = []
mat_list = []
depth_list = []
timestamp_list = []
thread_list = []
runtime_list = []
cam_status_list = []
stop_signal = False

def signal_handler(signal, frame):
    global stop_signal
    stop_signal=True
    time.sleep(0.5)
    exit()

def grab_run(index):
    global stop_signal
    global zed_list
    global timestamp_list
    global mat_list
    global depth_list

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed_list[index].grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed_list[index].retrieve_image(mat_list[index], sl.VIEW.LEFT)
            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH)
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
        time.sleep(0.001) #1ms
    zed_list[index].close()
	
def multi_camera_init(fps=30, resolution=sl.RESOLUTION.SVGA):
    global stop_signal
    global zed_list
    global mat_list
    global depth_list
    global timestamp_list
    global thread_list
    global runtime_list
    global cam_status_list
    signal.signal(signal.SIGINT, signal_handler)

    print("Running...")
    init = sl.InitParameters()
    init.camera_resolution = resolution
    init.camera_fps = fps  # The framerate is lowered to avoid any USB3 bandwidth issues
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE #(0,0) in top left corner. [X, Y, Z] Z forward.

    # init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

    #List and open cameras
    name_list = []
    last_ts_list = []
    cameras = sl.Camera.get_device_list()
    print("Length camera list: ", len(cameras))
    index = 0
    for cam in cameras:
        init.set_from_serial_number(cam.serial_number)
        name_list.append("ZED {}".format(cam.serial_number))
        print("Opening {}".format(name_list[index]))
        
        zed_list.append(sl.Camera())
        mat_list.append(sl.Mat())
        depth_list.append(sl.Mat())

        timestamp_list.append(0)
        last_ts_list.append(0)

        status = zed_list[index].open(init)
        cam_status_list.append(status)
        print("Opened {}".format(name_list[index]))

        runtime_list.append(sl.RuntimeParameters())

        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed_list[index].close()
        index = index +1

    return mat_list, zed_list, runtime_list, cam_status_list, name_list

def single_camera(serial_number, mat_list, zed_list, runtime_list, cam_status_list, name_list):

    for index in range(0, len(zed_list)):
        print("Checking name_list[{}] = {}".format(index, name_list[index]))
        print("Looking for: {}".format("ZED {}".format(serial_number)))
        if "ZED {}".format(serial_number) == name_list[index]:
            print("single camera found")
            mat = mat_list[index]
            camera = zed_list[index]
            runtime = runtime_list[index]
            cam_status = cam_status_list[index]

            return mat, camera, runtime, cam_status


   #Start camera threads after initializing cameras

def start_camera_threads(zed_list, thread_list):
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            thread_list.append(threading.Thread(target=grab_run, args=(index,)))
            #thread_list[index].daemon = True
            thread_list[index].start()
    
def stop_camera_threads(thread_list):
    #Stop the threads
    global stop_signal
    stop_signal = True
    for index in range(0, len(thread_list)):
        thread_list[index].join()

    print("\nFINISH")