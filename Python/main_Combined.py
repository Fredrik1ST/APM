# ZED ID 4mm (SMALL LENS): 40329509
# ZED ID 2.2mm (BIG LENS): 42146143

'''
#######################################################################
#
# Function gpsProcess() is copied from GeeksForGeeks
# Written by Aarti_Rathi, 13 Feb, 2023 
# Accsessed 18.04.2024
# Url: https://www.geeksforgeeks.org/program-distance-two-points-earth/  

######################################################################
# Update GPS rate

sudo pkill gpsd


echo -e -n "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xC2\x01\x00\x07\x00\x03\x00\x00\x00\x00\x00\xC0\x7E" > /dev/ttyACM0


echo -e "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\x7A\x12" > /dev/ttyACM0


gpsd -nG ntrip://admin:admin@159.162.103.14:2101/CPOSGLONASS -s 115200 /dev/ttyACM0

# Copied from : https://ozzmaker.com/faq/how-do-i-change-the-update-rate/ by ozzmaker. Accessed 29.04.2024
# Copied from : https://www.stereolabs.com/docs/get-started-with-zed-x/troubleshooting by StereoLabs. Accessed 29.04.2024

#######################################################################
'''

from lane_keeping.PIDController import PIDController
from tracking.body_tracking import init_body_tracking
from emergency_braking import emergency_braking
from multi_camera import signal_handler, grab_run, multi_camera_init, single_camera, start_camera_threads, stop_camera_threads, zed_list, mat_list, depth_list, thread_list, runtime_list, timestamp_list, cam_status_list, stop_signal
from speed_reference.GB_Controller import GBController as gbc


from camera import camera_init, get_camera_image
import matplotlib.pyplot as plt
import pyzed.sl as sl
import pandas as pd
import time
import cv2
import csv

######################### Import GPS functions ####################
from gpsd_reader import GPSDReader
import gps_processor as GPSProsessor 
import sys
import socket
import struct
import subprocess

######################### Import stop gps #########################
import stop_APM 

######################### Import backcontroller ###################
from speed_reference import cruise_controller as cc
from speed_reference import distance_controller as dc
from speed_reference import decide_controller as ctrl
from speed_reference import PID_to_motor as PID_motor

######################### Import Front Controller #################
from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection


#subprocess.run(["sudo", "service", "zed_x_daemon", "restart"])
subprocess.run(["./gpsd_init.sh"], shell=True)


######################### CAMERA SETTINGS #########################
fps = 15
resolution=sl.RESOLUTION.SVGA
show_image = False

######################### DETECTION SETTINGS ######################
detection_confidence = 40
detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST


######################### DISTANCE CONTROL ########################
d_min = 2
d_max = 6
d_init = (d_max + d_min) / 2


######################### LANE KEEPING ############################
cut_top = 4/7
angle_threshold = 50
show_lines = False
set_width = 640      


######################### MULTI CAMERA ############################
serial_number_front = 42146143
serial_number_back = 40329509
mat_list, zed_list, runtime_list, cam_status_list, name_list = multi_camera_init(fps, resolution)

front_cam_mat, front_cam, front_cam_runtime, front_cam_status = single_camera(serial_number_front, mat_list, zed_list, runtime_list, cam_status_list, name_list)
back_cam_mat, back_cam, back_cam_runtime, back_cam_status = single_camera(serial_number_back, mat_list, zed_list, runtime_list, cam_status_list, name_list)

start_camera_threads(zed_list, thread_list)


######################### OBJECT DETECTION ########################
viewer_front, bodies_front, body_runtime_param_front = init_body_tracking(front_cam, detection_confidence, show_image, detection_model)
print("Front camera Status: ", front_cam_status)

viewer_back, bodies_back, body_runtime_param_back = init_body_tracking(back_cam, detection_confidence, show_image, detection_model)
print("Back camera Status: ", back_cam_status)


######################### MOTOR CONTROLLER ########################
Kp_motor = 80
Ki_motor = 1251
Kd_motor = 0

motor_controller = gbc(kp=Kp_motor, ki=Ki_motor, kd=Kd_motor, setpoint=0, gamma=1, beta=1)


######################### DISTANCE CONTROLLER #####################
Kp_DC = 0.3
Ki_DC = 0
Kd_DC = 0.06
gamma_DC = 0.1
beta_DC = 1

dc_pid = gbc(kp=Kp_DC, ki=Ki_DC, kd=Kd_DC, setpoint=0, gamma=gamma_DC, beta=beta_DC)


######################### LANE KEEPING CONTROLLERS ################
angle_setpoint = -90        # The desired angle you want to maintain. -90 is straight ahead due to image coordinates
position_setpoint = 363     # The desired position you want to reach
steering_angle = 90 #Starting value set to 90 deg, straight

angle_pid = PIDController(kp=0.9, ki=0, kd=0.06, setpoint=angle_setpoint)
position_pid = PIDController(kp=0.03, ki=0.0, kd=0.03, setpoint=position_setpoint)
debug_pid = True

#____Low pass filter____
alpha = 1 # 1 Means off
last_steering_angle = 0
fpsCounter = 0


######################### GPS #####################################
fist_read_gnss = True
total_pos_travelled = 0
lat_prev = 0
lon_prev = 0
speed_gnss_prev = 0
speed_APM = 0
gps_reader = GPSDReader()

if gps_reader.initialize() != 0:
    print("Failed to initialize GPSDReader. Exiting...")
    sys.exit(1)


######################### CONNECT TO ARDUINO ######################
try:
    # IP and port for the computer
    HOST = '192.168.56.1'  # MÅ ENDRES TIL ZED BOX
    PORT = 49152

    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # makes it possible to reconnect without resetting arduino

    # Bind the socket to the host and port
    server_socket.bind((HOST, PORT))

    # Listen for incoming connections
    server_socket.listen(1)

    print('Waiting for connection...')

    # Accept a connection
    client_socket, addr = server_socket.accept()
    print('Connected by', addr)
    connectionStatus = True
except:
    print("Socket failed")
    connectionStatus = False


######################### LOG FILE ################################
file = input("Input name of output csv file: ")
#if file[-4:] != ".csv":
#    file += ".csv"
#print(file)

data_df = []


######################### TIMER ###################################
start = time.time()


######################### RUNNING SCHEDULE ########################
# Modules for generating speed reference from elite runners are made in speed_reference and pacing_schedule
Vd_list = [0, 1, 1, 1, 1, 0]
p_segment_list = [0, 10, 20, 30, 40, 50]


######################### MAIN LOOP ############################### 
totalTimePassed  = 0
start_time= time.time()
last_speed = 0
speed_input = 0
while True:
    try:
        fpsCounter += 1
        if show_image:
            if not (viewer_front.is_available() and viewer_back.is_available()):
                break
        print()
        print("---------------")

        #____Retrieve images from cameras____
        image_front = get_camera_image(front_cam_mat, front_cam, front_cam_runtime)
        assert image_front is not None, "file could not be read, check with os.path.exists()"

        image_back = get_camera_image(back_cam_mat, back_cam, back_cam_runtime)
        assert image_back is not None, "file could not be read, check with os.path.exists()"

        #____Retrieve detected objects from cameras____
        front_cam.retrieve_bodies(bodies_front, body_runtime_param_front)
        back_cam.retrieve_bodies(bodies_back, body_runtime_param_back)

        #_____Detect Runner______
        if len(bodies_back.body_list) > 0:

            runner_object = bodies_back.body_list[0] #Runner is the first element of the detected bodies. Assumed initialized with only runner visible for back camera
            time.sleep(0.05)

            #____Back Control System____
            runner_pos = runner_object.position[2] 
            if runner_pos <= 1:
                runner_pos =1
        else:
            print("No runner detected")
            runner_pos = (d_min + d_max) / 2    # Use CC    

        cc.updateCCparameters(Vd_list , p_segment_list, total_pos_travelled, speed_APM)
        ctrl.updatePosError(total_pos_travelled, Vd = cc.Vd_APM, d_init = d_init)
        ctrl.check_within_bounds(runner_pos, d_min, d_max, d_init)
        ctrl.decide_control_method(runner_pos, d_init=d_init)

        print("Controller",  ctrl.control_strategy)
        match ctrl.control_strategy:
            case 'cruise_control':
                v_ref = round(cc.cruise_controller(),2)
                speed_input = v_ref
            case 'distance_control':
                v_ref = round(dc.distance_controller(speed_APM, runner_pos, set_point = ctrl.d_ref, dc_pid = dc_pid),2)
                speed_input = v_ref

       # Find speed to motor
        motor_controller.setpoint = v_ref
        if last_speed != speed_APM:
            speed_input = round(PID_motor.input_to_motor(speed_APM, v_ref, motor_controller),2)
            print("Vd:", cc.Vd_APM)
            print("v_ref: ", v_ref)
            print("PID OUTPUT", speed_input)
            print("AMP speed: ", speed_APM)
            print("Time: ", totalTimePassed)
        
        if speed_input <= 0: # Negative values are not valid
            speed_input = 0

        pwm_input = int(20.23* speed_input + 1540) #Only used for logging data, speed_input gets converted to pwm on arduino


        # ____Visualization_____
        if show_image:
            viewer_front.update_view(front_cam_mat, bodies_front)
            viewer_back.update_view(back_cam_mat, bodies_back)

        #_____Front Control System________
        height, width, _ = image_front.shape
        image_front = image_front[int(height*cut_top):, :]
        image_front = cv2.resize(image_front, (set_width, int(height/width*set_width)), interpolation = cv2.INTER_AREA)
        lane_lines = line_detection(image_front, set_width, cut_top, angle_threshold, show_lines)



        if lane_lines != None:
            steering_angle,a, p, a_c, p_c = steering_angle_controller(image_front, angle_pid, 
                                                        position_pid, lane_lines, debug_pid)
            print("steering angle to motor: ", steering_angle)

            if last_steering_angle:
                steering_angle = last_steering_angle + alpha*(steering_angle - last_steering_angle)
            last_steering_angle = steering_angle
            print("Low pass steering angle: ", steering_angle)

        #____Emergency braking_____
        #emergency_brake, speed_reduction = emergency_braking(bodies_front)
        emergency_brake = False
        #if emergency_brake:
        #    print("EMERGENCY BRAKE")
        #    emergency_brake=False   # TODO remove
        #    speed_input *= (1-speed_reduction)
        #    steering_angle = 90

        bool_light_green = True # Endre til lyslogikk
        message = data = struct.pack('<3?2f', True, False, bool_light_green, steering_angle, speed_input)
        client_socket.sendall(message)

        # Plot
        timestamp = front_cam.get_timestamp(sl.TIME_REFERENCE.IMAGE) # Get the timestamp of the image
        new_row = {'Timestamp' : timestamp.get_milliseconds(), 
                    'Steering angle' : steering_angle, 
                    'Angle' : a,                # Setpoint 90
                    'Position' : p,             # Setpoint 320
                    'Angle control' : a_c,     
                    'Position control' : p_c,
                    'PWM pulse width': pwm_input,
                    'Distance travelled (m)': total_pos_travelled,
                    'Desired speed (m/s)': cc.Vd_APM,
                    'Reference speed (m/s)': v_ref,
                    'Measured speed (m/s)': speed_APM, 
                    'Distance runner/APM (m)': runner_pos
                    }  # Creating a new row data
        data_df.append(new_row)
        
    except KeyboardInterrupt: #Clc + c
        print("KeyboardInterrupt detected. Stopping rabbit")
        break 




df = pd.DataFrame(data_df)
df.to_csv(file)

print("FPS", fpsCounter/(time.time() - start))

######################### STOP CAR AND CLOSE SOCKET ############
if connectionStatus:
    data = struct.pack('<3?2f', False, True, False, 0.0, 0.0)
    client_socket.sendall(data)
    client_socket.close()
    server_socket.close()


######################### STOP THREADS #########################
gps_reader.stop_thread()
stop_camera_threads(thread_list, stop_signal)


######################### DISABLE MODULES AND CLOSE CAMERAS ####
front_cam_mat.free(memory_type=sl.MEM.CPU)
front_cam.disable_object_detection()
front_cam.disable_positional_tracking()
front_cam.disable_body_tracking()
front_cam.close()

back_cam_mat.free(memory_type=sl.MEM.CPU)
back_cam.disable_object_detection()
back_cam.disable_positional_tracking()
back_cam.disable_body_tracking()
back_cam.close()