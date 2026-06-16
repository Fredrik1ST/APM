'''
Modified version of Stereolabs' GNSS reader example:
https://github.com/stereolabs/zed-sdk/tree/master/global%20localization/live/python/gnss_reader

Changes:
    - Refactored into a "Driver" class with a start/stop interface for the main program.
    - Fixed thread handling and shutdown to prevent hanging threads.
    - Added thread-safe snapshot function to read the most recent GNSS data.
    - Added existence checks to keys in TPV messages to prevent silent crashes if a field is missing.

How it works on a low level:

1. GNSS data is read from the ZED-F9P module via serial interface (/dev/ttyACM0).
   This data is formatted as NMEA 0183 sentences.

2. NMEA sentences are parsed by gpsd, which is running as a daemon on the ZED Box.

3. gpsd outputs the parsed data in JSON format to TCP port 2947.

4. Our program parses the data to get what we need.

Notes: 
- The default polling rate of the ZED-F9P is 1Hz. 
  It can be configured to output at higher rates using ubxtool or 
  via directly sending serial UBX configuration messages to the module.
  This is implemented via the function 'set_measurement_rate'.

'''

import time
import threading
import pyzed.sl as sl
from gpsdclient import GPSDClient
from dataclasses import dataclass, field
import subprocess
import logging

log = logging.getLogger(__name__)


@dataclass
class GNSSSnapshot: # Simple dataclass to hold the most relevant GNSS data for our use case
    timestamp_monotonic: float = field(default_factory=time.monotonic)  # Monotonic clock at object creation time (seconds)
    timestamp_ms: int = 0   # Timestamp from GNSS receiver in milliseconds
    speed: float = 0.0      # [m/s]
    lon: float = 0.0,       # [degrees]
    lat: float = 0.0,       # [degrees]
    


class GNSSDriver:
    """Driver for ZED-F9P GNSS module via gpsd. 
    Runs a background thread that continuously reads at a specified rate.
    Stores thread-safe snapshots of data for the main program.
    
    Attributes:
        measurement_rate_hz (int): The rate at which to poll the GNSS module in Hz. (Max 20 Hz)
        speed_threshold (float): Minimum speed (m/s) to consider valid (to filter out noise when stationary)
    """
    def __init__(self):

        self.is_initialized = False
        self.ok = False # True if GNSS data is being successfully received
        self.measurement_rate_hz: int
        self.speed_threshold: float
        self.snapshot: GNSSSnapshot | None = None
        
        self._client = None
        self._new_data = False
        self._gnss_getter = None
        self._grabber_thread = None
        self._current_gnss_data = None
        self._stop_event = threading.Event()
        self._snapshot_lock = threading.Lock()
        self._is_initialized_mtx = threading.Lock()
        

    def start(self, measurement_rate_hz: int = 10, speed_threshold: float = 0.07):
        try:
            self._client = GPSDClient(host="127.0.0.1")
        except Exception:
            log.error("No GPSD running .. exit")
            return False
        
        try:
            self.set_measurement_rate(measurement_rate_hz)
        except Exception:
            return False
        
        self._gnss_getter = self._client.dict_stream(convert_datetime=True, filter=["TPV", "SKY"])

        # (Re)start thread for continuously grabbing GNSS data
        if self._grabber_thread is None or not self._grabber_thread.is_alive():
            self._stop_event.clear()
            self._grabber_thread = threading.Thread(target=self.grabGNSSData, daemon=True)
            self._grabber_thread.start()

        log.info("Successfully connected to GPSD")
        log.info("Waiting for GNSS fix")
        for gpsd_data in self._gnss_getter:
            if gpsd_data.get("class") == "TPV" and gpsd_data.get("mode", 0) >= 2:
                break
        log.info("GNSS fix found")
        with self._is_initialized_mtx:
            self.is_initialized = True
        return True


    def set_measurement_rate(self, rate_hz: int = 10):
        """Set measurement rate of the ZED-F9P GNSS module. Requires ubxtool to be installed. Max 20 Hz with RTK, """
        if rate_hz < 1:
            rate_hz = 1
        elif rate_hz > 20:
            rate_hz = 20
        rate_ms = int(1000 / rate_hz)
        self.measurement_rate_hz = rate_hz

        log.info(f'Setting GNSS polling rate to {rate_hz} Hz')
        try:
            result = subprocess.run(['ubxtool', '-p', 'CFG-RATE,', str(rate_ms)], capture_output=True, text=True, timeout=5)
            log.debug(f'ubxtool response: {result.stdout}')
        except subprocess.TimeoutExpired:
            log.error("Timeout while setting GNSS polling rate")
            raise
        except Exception as e:
            log.error(f"Could not set GNSS rate: {e}")
            raise


    def getNextGNSSValue(self):
        while not self._stop_event.is_set():
            gpsd_data = None 
            while gpsd_data is None:
                gpsd_data = next(self._gnss_getter)
            if "class" in gpsd_data and gpsd_data["class"] == "TPV" and "mode" in gpsd_data and gpsd_data["mode"] >=2:                    
                _current_gnss_data = sl.GNSSData()

                if not ("lat" in gpsd_data and "lon" in gpsd_data and "altMSL" in gpsd_data):
                    log.warning("Incomplete TPV message received: missing lat/lon/altMSL")
                _current_gnss_data.set_coordinates(gpsd_data.get("lat", 0.0), gpsd_data.get("lon", 0.0), gpsd_data.get("altMSL", 0.0), False)
                _current_gnss_data.longitude_std =  0.001
                _current_gnss_data.latitude_std = 0.001
                _current_gnss_data.altitude_std = 1.0

                gpsd_mode = gpsd_data["mode"]
                sl_mode = sl.GNSS_MODE.UNKNOWN
                if gpsd_mode == 0:  # MODE_NOT_SEEN
                    sl_mode = sl.GNSS_MODE.UNKNOWN
                elif gpsd_mode == 1:  # MODE_NO_FIX
                    sl_mode = sl.GNSS_MODE.NO_FIX
                elif gpsd_mode == 2:  # MODE_2D
                    sl_mode = sl.GNSS_MODE.FIX_2D
                elif gpsd_mode == 3:  # MODE_3D
                    sl_mode = sl.GNSS_MODE.FIX_3D

                sl_status = sl.GNSS_STATUS.UNKNOWN
                if 'status' in gpsd_data:
                    gpsd_status = gpsd_data["status"]
                    if gpsd_status == 0:  # STATUS_UNK
                        sl_status = sl.GNSS_STATUS.UNKNOWN
                    elif gpsd_status == 1:  # STATUS_GPS
                        sl_status = sl.GNSS_STATUS.SINGLE
                    elif gpsd_status == 2:  # STATUS_DGPS
                        sl_status = sl.GNSS_STATUS.DGNSS
                    elif gpsd_status == 3:  # STATUS_RTK_FIX
                        sl_status = sl.GNSS_STATUS.RTK_FIX
                    elif gpsd_status == 4:  # STATUS_RTK_FLT
                        sl_status = sl.GNSS_STATUS.RTK_FLOAT
                    elif gpsd_status == 5:  # STATUS_DR
                        sl_status = sl.GNSS_STATUS.SINGLE
                    elif gpsd_status == 6:  # STATUS_GNSSDR
                        sl_status = sl.GNSS_STATUS.DGNSS
                    elif gpsd_status == 7:  # STATUS_TIME
                        sl_status = sl.GNSS_STATUS.UNKNOWN
                    elif gpsd_status == 8:  # STATUS_SIM
                        sl_status = sl.GNSS_STATUS.UNKNOWN
                    elif gpsd_status == 9:  # STATUS_PPS_FIX
                        sl_status = sl.GNSS_STATUS.SINGLE

                _current_gnss_data.gnss_mode = sl_mode.value
                _current_gnss_data.gnss_status = sl_status.value
                
                position_covariance = [
                    gpsd_data["eph"] * gpsd_data["eph"],
                    0.0,
                    0.0,
                    0.0,
                    gpsd_data["eph"] * gpsd_data["eph"],
                    0.0,
                    0.0,
                    0.0,
                    gpsd_data["epv"] * gpsd_data["epv"]
                ]
                _current_gnss_data.position_covariances = position_covariance

                # Timestamp is a ISO 8601 time string converted to a datetime object by gpsdclient
                if not ("time" in gpsd_data):
                    log.warning("TPV message missing timestamp")
                else:
                    timestamp_seconds = float(gpsd_data["time"].timestamp())
                    timestamp_microseconds = int(timestamp_seconds * 1000000)
                    ts = sl.Timestamp()
                    ts.set_microseconds(timestamp_microseconds)
                    _current_gnss_data.ts = ts

                    # Update snapshot for main thread
                    if "speed" in gpsd_data and "lon" in gpsd_data and "lat" in gpsd_data:
                        with self._snapshot_lock:
                            self.snapshot = GNSSSnapshot(
                                timestamp = timestamp_microseconds,
                                speed = gpsd_data["speed"] if "speed" in gpsd_data else 0.0,
                                lon = gpsd_data["lon"] if "lon" in gpsd_data else 0.0,
                                lat = gpsd_data["lat"] if "lat" in gpsd_data else 0.0
                            )
                return _current_gnss_data

            elif gpsd_data.get("class") == "SKY": # Informational message about satellite status
                nb_low_snr = sum(
                    1 for s in gpsd_data.get('satellites', [])
                    if s.get('used') and s.get('ss', 99) < 16
                )
                if nb_low_snr > 0:
                    log.warning("[Warning] Low SNR (<16) on %d satellite(s) (using %s out of %s visible)",
                                nb_low_snr, gpsd_data.get('uSat', '?'), gpsd_data.get('nSat', '?'))
            else:
                log.warning("GNSS fix lost: reconnecting stream")
                try:
                    self._gnss_getter = self._client.dict_stream(convert_datetime=True, filter=["TPV", "SKY"])
                except Exception:
                    log.error("Failed to reconnect GNSS stream")


    def grab(self):
        if self._new_data:
            self._new_data = False
            return sl.ERROR_CODE.SUCCESS, self._current_gnss_data
        return sl.ERROR_CODE.FAILURE, None 


    def grabGNSSData(self):
        while not self._stop_event.is_set():
            with self._is_initialized_mtx:
                if self.is_initialized:
                    break
            time.sleep(0.001)

        while not self._stop_event.is_set():
            self._current_gnss_data = self.getNextGNSSValue()
            self._new_data = True


    def get_snapshot(self) -> GNSSSnapshot | None:
        """Get the most recent thread-safe GNSS snapshot for use in the main program"""
        with self._snapshot_lock:
            return self.snapshot


    def stop(self):
        self._stop_event.set()
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
        if self._grabber_thread and self._grabber_thread.is_alive():
            self._grabber_thread.join(timeout=2.0)
            if self._grabber_thread.is_alive():
                log.warning("GNSS thread did not stop cleanly within timeout")
        self.is_initialized = False
        