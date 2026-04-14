'''
Handling of GNSS data retrieval and processing from the ZED BOX's ZED-F9P module.

Used for speed control, measuring distance traveled, and calibration.

How it works:

1. GNSS data is read from the ZED-F9P module via serial interface (/dev/ttyACM0).
   This data is formatted as NMEA 0183 sentences.

2. NMEA sentences are parsed by gpsd, which is running as a daemon on the ZED Box.

3. gpsd outputs the parsed data in JSON format to TCP port 2947.
   This can

Notes: 

- The default polling rate of the ZED-F9P is 1Hz, 
  but it can be configured to output at higher rates using ubxtool or 
  via directly sending serial UBX configuration messages to the module.

- 
'''