



import pyzed.sl as sl
import ogl_viewer.viewer as gl
import numpy as np
from math import isnan



def emergency_braking(bodies):
    emergency_brake = False
    speed_reduction = 0     # in % 
    for person in bodies.body_list:
        
        # Assuming r and v are numpy arrays representing position and velocity vectors
        pos = person.position
        vel = person.velocity
        if not isnan(vel[0]):
            # given Image coordinate frame, first is depth, then side:
            r = np.array([pos[2], pos[0]])  # Position vector 2d plane
            v = np.array([vel[2], vel[0]])  # Velocity vector 2d plane

            # Calculate speed towards origin
            speed_towards_origin = -np.dot(r / np.linalg.norm(r), v)

            # Calculate distance to origin
            distance_to_origin = np.linalg.norm(r)

            closest_point = abs(np.cross(r, v)) / np.linalg.norm(v)

            if speed_towards_origin*2 > distance_to_origin and closest_point < 0.5:
                print("Emergency brake")
                emergency_brake = True
                # Get a speed reduction between 0.25 and 1 based on speed and distance due to if.
                speed_reduction = min(1, speed_towards_origin/(distance_to_origin*2))
        
    return emergency_brake, speed_reduction
