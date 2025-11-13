# -----------------------------------------------------------
# Run two ZED cameras in parallel threads
# One for image capture (front), one for body tracking (back)
#
# Main loop can be extended with new functionality as needed
#
# -----------------------------------------------------------

import pyzed.sl as sl           # ZED SDK Python wrapper
import ogl_viewer.viewer as gl  # For displaying images
import pandas as pd
import threading
import time
import os

######################### THREADING PARAMETERS #########################
stop_signal = False                 
lock_front = threading.Lock()       # Thread lock for front camera
lock_back = threading.Lock()        # Thread lock for back camera                  

######################### CAMERA PARAMETERS ############################
show_image = True                   # Show camera images in GL Viewer
serial_number_front = 42146143      # 2.2mm big lens
serial_number_back = 40329509       # 4mm small lens
fps = 15
resolution = sl.RESOLUTION.SVGA     # 960x600
coordinate_units = sl.UNIT.METER    # Default: millimeter
coordinate_system = sl.COORDINATE_SYSTEM.IMAGE #(0,0) in top left corner. [X, Y, Z] Z forward.


######################### SHARED VARIABLES ###########################
image_front = sl.Mat()
timestamp_front = sl.Timestamp()
image_back = sl.Mat()
bodies_back = sl.Bodies()
timestamp_back = sl.Timestamp()

######################### DISTANCE CONTROL ########################
d_min = 2
d_max = 6
d_init = (d_max + d_min) / 2

######################### LOG COLUMNS #############################
# Will show up in /log/YYYYMMDD/HHMMSS.csv after program ends
ls_timestamp_front = []
ls_timestamp_back = []
ls_num_bodies_back = []
ls_runner_distance = []


def run_front_cam(serial, fps, resolution, coord_units, coord_system):
    """Front camera setup with continuous image capture.

    Run on separate thread to avoid locking when grabbing images.

    Args:
        serial (int): Serial number of the camera
        fps (int): Frames per second
        resolution (pyzed.sl.RESOLUTION): Camera resolution
        coord_units (pyzed.sl.UNIT): Coordinate units
        coord_system (pyzed.sl.COORDINATE_SYSTEM): Coordinate system
    """ 
    global image_front, timestamp_front, stop_signal, show_image
    image = sl.Mat()

    # --- Camera setup ---
    init_params = sl.InitParameters()
    init_params.set_from_serial_number(serial)
    init_params.camera_resolution = resolution
    init_params.camera_fps = fps
    init_params.coordinate_units = coord_units
    init_params.coordinate_system = coord_system
    camera = sl.Camera()

    if camera.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open front camera {serial}")
        return

    runtime = sl.RuntimeParameters()
    image = sl.Mat() # Left image
    viewer = None
    if show_image:
        viewer = gl.GLViewer()
        camera_info = camera.get_camera_information()
        viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, False)

    print(f"Started capture on front camera {serial}")

    while not stop_signal:
        if camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            camera.retrieve_image(image, sl.VIEW.LEFT)
            # Update shared variables
            with lock_front:
                image_front = image.get_data().copy()
                timestamp_front = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        time.sleep(0.001)

    camera.close()


def run_back_cam(serial, fps, resolution, coord_units, coord_system):
    """Back camera setup and continuous image capture + body tracking.

    Run on separate thread to avoid locking when grabbing images.

    Args:
        serial (int): Serial number of the camera
        fps (int): Frames per second
        resolution (pyzed.sl.RESOLUTION): Camera resolution
        coord_units (pyzed.sl.UNIT): Coordinate units
        coord_system (pyzed.sl.COORDINATE_SYSTEM): Coordinate system
    """ 
    global image_back, bodies_back, timestamp_back, stop_signal, show_image

    # --- Camera setup ---
    init_params = sl.InitParameters()
    init_params.set_from_serial_number(serial)
    init_params.camera_resolution = resolution
    init_params.camera_fps = fps
    init_params.coordinate_units = coord_units
    init_params.coordinate_system = coord_system
    camera = sl.Camera()

    if camera.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open back camera {serial}")
        return

    # --- Body Tracking setup ---
    body_params = sl.BodyTrackingParameters()
    body_params.enable_tracking = True
    body_params.enable_body_fitting = False                                # Optimize joint positions
    # body_params.body_format = sl.BODY_FORMAT.BODY_18                     # Default basic body model
    body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST

    # --- Positional tracking setup (necessary for body tracking) ---
    if body_params.enable_tracking:
        positional_tracking_param = sl.PositionalTrackingParameters()
        positional_tracking_param.set_floor_as_origin = True
        camera.enable_positional_tracking(positional_tracking_param)

    print("Body tracking: Loading Module...")

    err = camera.enable_body_tracking(body_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Enable Body Tracking : "+repr(err)+". Exit program.")
        camera.close()
        exit()

    body_runtime_param = sl.BodyTrackingRuntimeParameters()
    # For outdoor scene or long range, the confidence should be lowered to avoid missing detections (~20-30)
    # For indoor scene or closer range, a higher confidence limits the risk of false positives and increase the precision (~50+)
    body_runtime_param.detection_confidence_threshold = 40

    runtime = sl.RuntimeParameters()
    bodies = sl.Bodies()
    image = sl.Mat()
    viewer = None
    if show_image:
        viewer = gl.GLViewer()
        camera_info = camera.get_camera_information()
        viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, body_params.enable_tracking)

    
    print(f"Started back camera {serial}")

    while not stop_signal:
        if camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            camera.retrieve_image(image, sl.VIEW.LEFT)
            camera.retrieve_bodies(bodies, body_runtime_param)
            if show_image:
                viewer.update_view(image, bodies)

            # Update shared variables
            with lock_back:
                image_back = image.get_data().copy()
                bodies_back = bodies
                timestamp_back = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE)

    camera.disable_body_tracking()
    camera.close()


def main():
    """Run front and back cameras for image capture and body tracking
    
    """
    global stop_signal
    fpsCounter = 0
    last_timestamp_front = 0
    last_timestamp_back = 0
    front_updated = False
    back_updated = False

    # --- Discover both cameras ---
    devices = sl.Camera.get_device_list()
    if len(devices) < 2:
        print(f"Error: {len(devices)} / 2 cameras detected")
        return
    
    # --- START THREADS ---
    t0 = threading.Thread(target=run_front_cam, args=(0, serial_number_front, fps, resolution))
    t1 = threading.Thread(target=run_back_cam, args=(1, serial_number_back, fps, resolution))
    t0.start()
    t1.start()

    # --- MAIN LOOP ---
    try:
        while True:

            # # --- Check for camera updates ---
            with lock_front:
                if timestamp_front.get_milliseconds() != last_timestamp_front:
                    front_updated = True
                    last_timestamp_front = timestamp_front.get_milliseconds()
                    
            with lock_back:
                if timestamp_back.get_milliseconds() != last_timestamp_back:
                    back_updated = True
                    last_timestamp_back = timestamp_back.get_milliseconds()


            # --- FRONT CONTROL ---
            if front_updated:
                front_updated = False

                # Log front system
                ls_timestamp_front.append(last_timestamp_front) # Log timestamp


            # --- BACK CONTROL ---
            if back_updated:
                back_updated = False

                # Detect Runner Distance (meters)
                if len(bodies_back.body_list) > 0:
                    runner_distance = bodies_back.body_list[0].position[2]    # Assumed to be the first detected person, with distance in meters (Z axis)             
                    if runner_distance <= 1:
                        runner_distance = 1
                else:
                    runner_distance = (d_min + d_max) / 2 # Assume mid distance if no bodies detected (=cruise control speed)

                # Log back system
                ls_timestamp_back.append(last_timestamp_back)                   # Log timestamp
                ls_num_bodies_back.append(len(bodies_back.body_list))           # Log number of bodies
                ls_runner_distance.append(bodies_back.body_list[0].position[2]) # Log runner distance (DO NOT log runner_distance variable here!)
                    

            time.sleep(0.0005) # Sleep main thread to avoid loop using 100% CPU due to active polling


    except KeyboardInterrupt:
        stop_signal = True
        print("Stopping...")

    t0.join()
    t1.join()
    print("All threads closed.")

# Log data to CSV
df = pd.DataFrame({
    "Front Timestamp (ms)": ls_timestamp_front,
    "Back Timestamp (ms)": ls_timestamp_back,
    "Number of Bodies (Back)": ls_num_bodies_back
})
# Save log in /log/YYYYMMDD/HHMMSS.csv format
root_dir = "log"
day_str = time.strftime("%Y%m%d")
time_str = time.strftime("%H%M%S")
os.makedirs(os.path.join(root_dir, day_str), exist_ok=True)
file_name = os.path.join(root_dir, day_str, f"{time_str}.csv")
df.to_csv(file_name)


if __name__ == "__main__":
    main()