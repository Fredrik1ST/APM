'''This code is edited from the original from the GitHub code Found 31.01.204'''
########################################################################
#
# Copycopyright c
# right (c) 2023, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
# DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################


import threading
import time
import subprocess
import pyzed.sl as sl
from gpsdclient import GPSDClient
import random
import datetime
import socket
import json

class GPSDReader:

    def __init__(self):
        self.continue_to_grab = True
        self.new_data = False
        self.is_initialized = False
        self.current_gnss_lat = None
        self.current_gnss_long = None
        self.current_gnss_speed = None
        self.current_gnss_speed = None
        self.is_initialized_mtx = threading.Lock()
        self.client = None
        self.gnss_getter = None

    def connect_to_ntrip():
        # Kill gpsd if it is already running
        subprocess.run(["sudo", "pkill", "gpsd"])

        # Start gpsd with NTRIP connection
        subprocess.run(["gpsd", "-nG", "ntrip://admin:admin@159.162.103.14:2101/CPOSGPS", "-s", "115200", "/dev/ttyACM0"])
        subprocess.call(['gpsctl', '-c', '0.1'])  # Digits are the time in seconds.

    def initialize(self):
        #self.connect_to_ntrip()
        try : 
            self.client = GPSDClient(host="127.0.0.1")
        except : 
            print("No GPSD running .. exit")
            return -1 
        
        self.grab_gnss_data = threading.Thread(target=self.grabGNSSData)
        self.grab_gnss_data.daemon = True
        self.grab_gnss_data.start()
        print("Successfully connected to GPSD")
        print("Waiting for GNSS fix")
        received_fix = False
        while not received_fix:
            self.gnss_getter = self.client.dict_stream(convert_datetime=True, filter=["TPV"])
            gpsd_data = next(self.gnss_getter)
            if "class" in gpsd_data and gpsd_data["class"] == "TPV" and "mode" in gpsd_data and gpsd_data["mode"] >=2:
                received_fix = True
        print("Fix found !!!")
        with self.is_initialized_mtx:
            self.is_initialized = True
        return 0 

    def getNextGNSSValue(self):
        while self.continue_to_grab:
            gpsd_data = None 
            while gpsd_data is None:
                gpsd_data = next(self.gnss_getter)
            if "class" in gpsd_data and gpsd_data["class"] == "TPV" and "mode" in gpsd_data and gpsd_data["mode"] >= 2:                    
                current_gnss_long = gpsd_data.get("lat", None) 
                current_gnss_lat = gpsd_data.get("lon", None) 
                current_gnss_speed = gpsd_data.get("speed", None) 
                return current_gnss_lat, current_gnss_long, current_gnss_speed # Return latitude, longitude and speed
            else:  
                print("Fix perdu : réinitialisation du GNSS")
                self.initialize()
        


    def grab(self):
        if self.new_data:
            self.new_data = False
            return sl.ERROR_CODE.SUCCESS, self.current_gnss_lat, self.current_gnss_long, self.current_gnss_speed
        return sl.ERROR_CODE.FAILURE, None,None, None

    def grabGNSSData(self):
        while self.continue_to_grab:
            with self.is_initialized_mtx:
                if self.is_initialized:
                    break
            time.sleep(0.001)

        while self.continue_to_grab :
            self.current_gnss_lat, self.current_gnss_long, self.current_gnss_speed = self.getNextGNSSValue()
            self.new_data = True

    def stop_thread(self):
        self.continue_to_grab = False
        self.grab_gnss_data.join()


