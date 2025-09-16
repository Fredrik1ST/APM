#######################################################################
#
# Function gpsProcess() is copied from GeeksForGeeks
# Written by Aarti_Rathi, 13 Feb, 2023 
# Accsessed 18.04.2024
# Url: https://www.geeksforgeeks.org/program-distance-two-points-earth/  

##################################################################
'''
How to change samplingrate (Hz) on GPS
1: Kill gpsd if it is already running
    sudo pkill gpsd

2: To change the baud rate to 115200;
    echo -e -n "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\xC2\x01\x00\x07\x00\x03\x00\x00\x00\x00\x00\xC0\x7E" > /dev/ttyACM0

3: To increase update rate to 10Hz
    echo -e "\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\x7A\x12" > /dev/ttyACM0

4: Enabling RTK on your GNSS module (Makes the position more accurate)
    gpsd -nG ntrip://admin:admin@159.162.103.14:2101/CPOSGLONASS -s 115200 /dev/ttyACM0

from: https://ozzmaker.com/faq/how-do-i-change-the-update-rate/
and: https://www.stereolabs.com/docs/get-started-with-zed-box-orin-nx/gnss

'''

##################################################################

import pyzed.sl as sl
from gpsd_reader import GPSDReader
from math import radians, cos, sin, asin, sqrt
from speed_reference import controller_timer as timer


def DistanceFromCoordinates(lat1, lat2, lon1, lon2):
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
      
    dlon = lon2 - lon1 
    dlat = lat2 - lat1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
 
    c = 2 * asin(sqrt(a)) 
    r = 6371
    return(c * r * 1609.344)

def gpsProcess(gps_reader, fist_read_gnss, total_pos, lat_prev, lon_prev, speed_gnss_prev):

    status, lat_gnss, lon_gnss, speed_gnss = gps_reader.grab() # Get information from GPS 
    if status == sl.ERROR_CODE.SUCCESS:
        print("Received GNSS data: Latitude:", lat_gnss, ", Longitude:", lon_gnss, ", Speed:", speed_gnss)
        if fist_read_gnss: # Initilize first position
            total_pos = 0
            lon_prev = lon_gnss
            lat_prev = lat_gnss
            fist_read_gnss = False
        else:   
            # If speed of the APM is larger than 0.07 m/s calculate distance moved. 
            # Prevents the distance moved to be updated if the car is standing still, The GPS is has an 0.05 m/s accuracy. 
            if speed_gnss >= 0.07: 
                 #Start timer if not yet started
                if timer.start_time is None:
                    timer.start_timer()
                # If one second has passed update position
                elif timer.check_timer(1):
                    total_pos += DistanceFromCoordinates(lat_gnss, lat_prev, lon_gnss, lon_prev) # Update position
                    timer.start_time = None
            # Update previos position and speed     
            lat_prev = lat_gnss
            lon_prev = lon_gnss
            speed_gnss_prev = speed_gnss 
    else:
        print("Failed to grab GNSS data.") # Returns None for all parameters
        speed_gnss = speed_gnss_prev
        lat_gnss = lat_prev
        lon_gnss = lon_prev
    return fist_read_gnss, total_pos, lat_prev, lon_prev, speed_gnss_prev, speed_gnss


def speedLogger(gps_reader, fist_read_gnss, total_pos, lat_prev, lon_prev, speed_input, speed_gnss_prev): 
    status, lat_gnss, lon_gnss, speed_gnss = gps_reader.grab() # Get information from GPS 
    if status == sl.ERROR_CODE.SUCCESS:
        print("Received GNSS data: Latitude:", lat_gnss, ", Longitude:", lon_gnss, ", Speed:", speed_gnss)
        if fist_read_gnss: # Initilize first position
            total_pos = 0
            lon_prev = lon_gnss
            lat_prev = lat_gnss
            fist_read_gnss = False
        else:   
            # If speed of the APM is larger than 0.07 m/s calculate distance moved. 
            # Prevents the distance moved to be updated if the car is standing still, The GPS is has an 0.05 m/s accuracy. 
            if speed_gnss >= 0.07: 
                 #Start timer if not yet started
                if timer.start_time is None:
                    timer.start_timer()
                # If one second has passed update position
                elif timer.check_timer(1):
                    total_pos += DistanceFromCoordinates(lat_gnss, lat_prev, lon_gnss, lon_prev) # Update position
                    timer.start_time = None
            # Update previos position and speed   
            lat_prev = lat_gnss
            lon_prev = lon_gnss
            speed_gnss_prev = speed_gnss
    else:
        print("Failed to grab GNSS data.") # Returns None for all parameters
        speed_gnss = speed_gnss_prev
        lat_gnss = lat_prev
        lon_gnss = lon_prev
    return fist_read_gnss, total_pos, lat_prev, lon_prev, speed_gnss_prev, speed_gnss
    