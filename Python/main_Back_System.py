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
import time
import cv2
import csv

# Import GPS functions
from gpsd_reader import GPSDReader
import gps_processor as GPSProsessor 
import sys
import socket
import struct
import subprocess

# Import stop gps
import stop_APM 

# Import backcontroller
from speed_reference import cruise_controller as cc
from speed_reference import distance_controller as dc
from speed_reference import decide_controller as ctrl
from speed_reference import PID_to_motor as PID_motor
#Import light logic
#from speed_reference import lights_logic as light

#subprocess.run(["sudo", "service", "zed_x_daemon", "restart"])
subprocess.run(["~/Documents/kristoffer/enya-Autonomous-Rabbit/gpsd_init.sh"], shell=True)


d_min = 3
d_max = 6
d_init = (d_max + d_min) / 2

# Tunable parameters:
set_width = 640      
show_image = False #True
fps = 30
resolution=sl.RESOLUTION.SVGA
detection_confidence = 40
detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST

############################### Initialization of two cameras from serial number. Starting camera threads ###########################################
serial_number_back = 40329509
print(1)
back_cam_mat, back_cam, back_cam_runtime, back_cam_status = camera_init(fps, resolution)
print(2)

############################### Initialization of object detection for two cameras. Body tracking ##################################################

viewer_back, bodies_back, body_runtime_param_back = init_body_tracking(back_cam, detection_confidence, show_image, detection_model)
print("Back camera Status: ", back_cam_status)
print(3)

############################### Initialization of GPS variables #############################################################
fist_read_gnss = True
total_pos_travelled = 0
lat_prev = 0
lon_prev = 0
speed_gnss_prev = 0
speed_APM = 0
gps_reader = GPSDReader()
print(4)

if gps_reader.initialize() != 0:
    print("Failed to initialize GPSDReader. Exiting...")
    sys.exit(1)


################################# Input user MAKE MODULE input from APP################################################################################

Vd_list = [0, 1, 1, 1, 1, 0]
p_segment_list = [0, 10, 20, 30, 40, 50]

################################  CONNECT TO ARDUINO ##################################################################################################

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


file = input("Input name of output csv file: ")
if file[-4:] != ".csv":
    file += ".csv"
print(file)

Kp_motor = 3
Ki_motor = 0.2
Kd_motor = 0

Kp_DC = input("Kp_DC: ")
if Kp_DC == "":
    Kp_DC = 0.15
else:
    Kp_DC = float(Kp_DC)

Ki_DC = input("Ki_DC: ")
if Ki_DC == "":
    Ki_DC = 0
else:
    Ki_DC = float(Ki_DC)

Kd_DC = input("Kd_DC: ")
if Kd_DC == "":
    Kd_DC = 0.005
else:
    Kd_DC = float(Kd_DC)

gamma_DC = input("gamma_DC:")
if gamma_DC == "":
    gamma_DC = 0.1
else:
    gamma_DC = float(gamma_DC)
    
beta_DC = input("beta_DC:")
if beta_DC == "":
    beta_DC = 1
else:
    beta_DC = float(beta_DC)


dc_pid = gbc(kp = Kp_DC, ki = Ki_DC, kd = Kd_DC, setpoint = 0, gamma = gamma_DC, beta = beta_DC)
motor_controller = gbc(kp=Kp_motor, ki=Ki_motor, kd=Kd_motor, setpoint=0, gamma=0.1, beta=1)


csvfile = open(file, 'w', newline='')
fieldnames = ['PWM pulse width', 'Distance travelled (m)', 'Reference speed (m/s)', 'PID OUTPUT (m/s)','Measured speed (m/s)','Kp','Kd', 'Control', 'D_ref','Distance runner/APM (m)', 'Time (s)']
writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

## Startup light
# light.lampStarup(bool_readyToStart) #Input should be the signal to say that all parameters are found, turn off when the runner starts to run


################################# while-loop ######################################################################################
totalTimePassed  = 0
start_time= time.time()
last_speed = 0
while True:
    try:
        if show_image:
            if not viewer_back.is_available():
                break
        print("---------------")

        image_back = get_camera_image(back_cam_mat, back_cam, back_cam_runtime)
        assert image_back is not None, "file could not be read, check with os.path.exists()"

        back_cam.retrieve_bodies(bodies_back, body_runtime_param_back) # Retrieve the detected objects from back camera
        if show_image:
                viewer_back.update_view(back_cam_mat, bodies_back)

        # Skip rest if no bodies found:
        if len(bodies_back.body_list) == 0:
            print("No runner found")
            continue

        runner_object = bodies_back.body_list[0] #Runner is the first element of the detected bodies. Assumed initialized with only runner visible for back camera
            
        time.sleep(0.05)
    ######################################################### GPS
        fist_read_gnss, total_pos_travelled, lat_prev, lon_prev, speed_gnss_prev, speed_APM = GPSProsessor.gpsProcess(gps_reader, fist_read_gnss, total_pos_travelled, lat_prev, lon_prev, speed_gnss_prev)
        print("New Distance moved:", total_pos_travelled)
        print("New Speed:", speed_APM)
        time.sleep(0.05)


        ###################### Controller testing ############################################
        runner_pos = runner_object.position[2] 
        if runner_pos <= 1:
            runner_pos =1

        cc.updateCCparameters(Vd_list , p_segment_list, total_pos_travelled, speed_APM)
        ctrl.updatePosError(total_pos_travelled, Vd = cc.Vd_APM, d_init = d_init)
        ctrl.check_within_bounds(runner_pos, d_min, d_max, d_init)
        ctrl.decide_control_method(runner_pos, d_init=d_init)

        print("Controller",  ctrl.control_strategy)
        match ctrl.control_strategy:
            case 'cruise_control':
                v_ref = round(cc.cruise_controller(),2)
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

        message = data = struct.pack('<3?2f', True, False, False, 90, speed_input) # Drive straight
        if connectionStatus:
            client_socket.sendall(message)

        print("Distance to runner: {}".format(runner_pos) )

        totalTimePassed = time.time()  - start_time
        last_speed = speed_APM

        writer.writerow({'PWM pulse width': pwm_input, 'Distance travelled (m)': total_pos_travelled, 'Reference speed (m/s)': v_ref,'PID OUTPUT (m/s)': speed_input, 'Measured speed (m/s)': speed_APM, 'Kp': Kp_DC,'Kd':Kd_DC,'Control': ctrl.control_strategy, 'D_ref': ctrl.d_ref, 'Distance runner/APM (m)': runner_pos, 'Time (s)': totalTimePassed})

    except KeyboardInterrupt: #Clc + c
        print("KeyboardInterrupt detected. Stopping...")
        gps_reader.stop_thread()
        if connectionStatus:
            # Stop car and close connection to arduino
            data = struct.pack('<3?2f', False, False, True, 0.0, 0.0)
            client_socket.sendall(data)
            client_socket.close()
            server_socket.close()
            connectionStatus = False

        # Disable modules and close camera
        back_cam_mat.free(memory_type=sl.MEM.CPU)
        back_cam.disable_object_detection()
        back_cam.disable_positional_tracking()
        back_cam.disable_body_tracking()
        back_cam.close()

        break

        

if connectionStatus:
    # Stop car and close connection to arduino
    data = struct.pack('<2?2f', False, True, 0.0, 0.0)
    client_socket.sendall(data)
    client_socket.close()
    server_socket.close()

gps_reader.stop_thread()

# Disable modules and close camera
back_cam_mat.free(memory_type=sl.MEM.CPU)
back_cam.disable_object_detection()
back_cam.disable_positional_tracking()
back_cam.disable_body_tracking()
back_cam.close()
