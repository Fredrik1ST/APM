import cv2
import numpy as np
import math
import statistics 
from matplotlib import pyplot as plt


def steering_angle_controller(image, angle_pid, position_pid, lane_lines, debug=False, show_image=False):
    a1_optimal, b1_optimal, a2_optimal, b2_optimal = lane_lines
    def find_intersection(a1, b1, a2, b2):
        if a1 == a2:
            # The lines are parallel and will never intersect
            return None
        x = (b2 - b1) / (a1 - a2)
        y = a1 * x + b1
        return round(x), round(y)
    def find_intersection_with_y_equals_constant(a1, b1, a2, b2, y_constant):
        # Solve for x when y = y_constant in both line equations
        x1 = (y_constant - b1) / a1 if a1 != 0 else None
        x2 = (y_constant - b2) / a2 if a2 != 0 else None

        return (x1+x2) / 2

    x_intersect, y_intersect = find_intersection(a1_optimal, b1_optimal, a2_optimal, b2_optimal)
    #print("intersect", x_intersect, y_intersect)
    
    height, width, _ = image.shape  
    y_robot = height
    x_robot = width/2
    
    #print(x_robot, y_robot)
    x_center_of_lane_at_bottom = int(find_intersection_with_y_equals_constant(a1_optimal, b1_optimal, a2_optimal, b2_optimal, y_robot))
   
    # Angle between robot and aiming point(Vanishing point)
    angle_radians = math.atan2(y_intersect - y_robot, x_intersect - x_robot)
    angle = math.degrees(angle_radians)

    # Simulate controllers
    angle_control_signal = -angle_pid.compute(angle)
    position_control_signal = position_pid.compute(x_center_of_lane_at_bottom)
    steering_angle = -angle_control_signal + position_control_signal


    if debug:
        print("angle and pos:")
        print(angle, x_center_of_lane_at_bottom)
        #print("Angle control, position control and steering angle:")
        #print(angle_control_signal," + ", position_control_signal," = ", steering_angle)

    if show_image:
        image = cv2.circle(image, (int(x_intersect),int(y_intersect)), radius=5, color=(255, 0, 0), thickness=-1)   
        image = cv2.circle(image, (x_center_of_lane_at_bottom,y_robot), radius=100, color=(255, 0, 0), thickness=-1)   
        # Center line for lane
        cv2.line(image,(x_center_of_lane_at_bottom,y_robot),(int(x_intersect),int(y_intersect)),(0,0,255),2)

        cv2.imshow("Image lines", image)
        cv2.imwrite("lines.jpg", image)
        cv2.waitKey(0) 

        cv2.destroyAllWindows()


    return steering_angle + 90, angle, x_center_of_lane_at_bottom, angle_control_signal, position_control_signal
