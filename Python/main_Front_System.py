# FRONT MAIN FINISHED
from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection
from lane_keeping.PIDController import PIDController
from tracking.body_tracking import init_body_tracking
from emergency_braking import emergency_braking

from camera import camera_init, get_camera_image
import matplotlib.pyplot as plt
import pyzed.sl as sl
import time
import cv2
import socket
import struct
import pandas as pd



#sl.Camera.reboot()?
# sl.rebootCamera()

file_name = input('Filename: ')
# Tunable parameters:

# Camera settings
set_width = 640      
show_image = False
fps = 15
resolution=sl.RESOLUTION.SVGA

# Lane keeping
cut_top = 4/7
angle_threshold = 50
show_lines = True

# Detection:
detection_confidence = 40
detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST

# PIDs:
angle_setpoint = -90  # The desired angle you want to maintain
position_setpoint = 363  # The desired position you want to reach
# angle_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, setpoint=angle_setpoint)
# position_pid = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=position_setpoint)
#angle_pid = PIDController(kp=0, ki=0, kd=0.0, setpoint=angle_setpoint)
#position_pid = PIDController(kp=0.01, ki=0, kd=0.01, setpoint=position_setpoint)
angle_pid = PIDController(kp=0.3, ki=0, kd=0.02, setpoint=angle_setpoint)
position_pid = PIDController(kp=0.01, ki=0.0, kd=0.01, setpoint=position_setpoint)
debug_pid = True


# Camera init:
mat, camera, runtime, cam_status = camera_init(fps, resolution)

viewer, bodies, body_runtime_param = init_body_tracking(camera, detection_confidence, show_image, detection_model)

test_image = get_camera_image(mat, camera, runtime)
print("Camera Status: ", cam_status)

# Low pass filter:
alpha = 1 # TODO: tune 1 is no
last_steering_angle = 0


#CONNECT TO ARDUINO

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

speed_input = float(input("Speed input: "))


start = time.time()
fpsCounter = 0


data_df = []
    

speedref = speed_input
while True:
    try:
        fpsCounter += 1
        if show_image:
            if not viewer.is_available():
                break
        print()
        print("---------------")
        image = get_camera_image(mat, camera, runtime)
        assert image is not None, "file could not be read, check with os.path.exists()"


        camera.retrieve_bodies(bodies, body_runtime_param) # Retrieve the detected objects
        emergency_brake, speed = emergency_braking(bodies)

        speed_input = speedref
        if emergency_brake:
            print("EMERGENCY BRAKE")
            emergency_brake=False   # TODO remove
            speed_input *= speed

        # visualization
        if show_image:
            viewer.update_view(mat, bodies)

        
        if not emergency_brake:
            """cv2.imshow("Original",image)
            cv2.waitKey(0)"""
            height, width, _ = image.shape
            image = image[int(height*cut_top):, :]
            image = cv2.resize(image, (set_width, int(height/width*set_width)), interpolation = cv2.INTER_AREA)
            lane_lines = line_detection(image, set_width, cut_top, angle_threshold, show_lines)

            if lane_lines != None:
                steering_angle,a, p, a_c, p_c = steering_angle_controller(image, angle_pid, 
                                                        position_pid, lane_lines, debug_pid)
                print("steering angle to motor: ", steering_angle)

                if last_steering_angle:
                    steering_angle = last_steering_angle + alpha*(steering_angle - last_steering_angle)
                last_steering_angle = steering_angle
                print("Low pass steering angle: ", steering_angle)

                message = data = struct.pack('<2?2f', True, False, steering_angle, speed_input)
                client_socket.sendall(message)

                # Plot
                timestamp = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE) # Get the timestamp of the image
                new_row = {'Timestamp' : timestamp.get_milliseconds(), 
                            'Steering angle' : steering_angle, 
                            'Angle' : a,                # Setpoint 90
                            'Position' : p,             # Setpoint 320
                            'Angle control' : a_c,     
                            'Position control' : p_c
                            }  # Creating a new row data
                data_df.append(new_row)

            else: #??
                message = data = struct.pack('<2?2f', True, False, 90, speed_input) # Drive straight
                client_socket.sendall(message)

    except KeyboardInterrupt: #Clc + c
        print("KeyboardInterrupt detected. Stopping rabbit")
        break

# TODO
# ctrl-c
# timer

df = pd.DataFrame(data_df)
df.to_csv(file_name+'.csv')



print("FPS", fpsCounter/(time.time() - start))


# Stop car and close connection to arduino
data = struct.pack('<2?2f', False, True, 0.0, 0.0)
client_socket.sendall(data)
client_socket.close()
server_socket.close()

# Disable modules and close camera
mat.free(memory_type=sl.MEM.CPU)
camera.disable_object_detection()
camera.disable_positional_tracking()
camera.disable_body_tracking()
camera.close()