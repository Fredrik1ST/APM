# This is the code for stopping the APM during or after a finished run!

from speed_reference import controller_timer as timer
from multi_camera import stop_camera_threads
import pyzed.sl as sl

def stop_if_runner_stands_still(speed_APM, gps_reader, front_cam_mat, front_cam, back_cam_mat, back_cam, thread_list, stop_signal, viewer_front,viewer_back):
    if ( speed_APM < 0.05):
        if timer.start_time is None:
            timer.start_timer()
        # If three second has passed Stop APM kill all threads and close cameras
        elif timer.check_timer(3):
            print("Runner has stopped running, closing all threads")
            speed_ref = 0
            gps_reader.stop_thread()

            front_cam_mat.free(memory_type=sl.MEM.CPU)
            front_cam.disable_object_detection()
            front_cam.disable_positional_tracking()
            front_cam.disable_body_tracking()

            back_cam_mat.free(memory_type=sl.MEM.CPU)
            back_cam.disable_object_detection()
            back_cam.disable_positional_tracking()
            back_cam.disable_body_tracking()
            stop_camera_threads(thread_list, stop_signal)

            viewer_front.exit()
            viewer_back.exit()
            front_cam.close()
            back_cam.close()

            timer.start_time = None
            return speed_ref
    return 
    
 # Import the speed the Runner has from cameras, as well as the APMs speed  
def run_is_finished(speed_APM, gps_reader, front_cam_mat, front_cam, back_cam_mat, back_cam, thread_list, stop_signal):
    # If the run is finised switch to DC, stop APM if runner stands still for over 3 seconds. 
    # Start timer if not yet started
    control_strategy = 'distance_control'
    speed_ref = stop_if_runner_stands_still(speed_APM, gps_reader, front_cam_mat, front_cam, back_cam_mat, back_cam, thread_list, stop_signal)

    return control_strategy, speed_ref
    