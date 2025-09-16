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
import time
import sys
from gpsd_reader import GPSDReader
import gps_processor as GPSProsessor 


def main():
    #insialisering
    fist_read_gnss = True
    total_pos = 0
    lat_prev = 0
    lon_prev = 0
    speed_APM = 0

    gps_reader = GPSDReader()
    
    if gps_reader.initialize() != 0:
        print("Failed to initialize GPSDReader. Exiting...")
        sys.exit(1)

    try:
        while True:
            fist_read_gnss, total_pos, lat_prev, lon_prev, speed_APM = GPSProsessor.gpsProcess(gps_reader, fist_read_gnss, total_pos, lat_prev, lon_prev)

            print("New Distance moved:", total_pos)
            print("New Speed:", speed_APM)
            time.sleep(0.1)
            
    except KeyboardInterrupt: #Clc + c
        print("KeyboardInterrupt detected. Stopping GPSDReader...")
        gps_reader.stop_thread()

if __name__ == "__main__":
    main()