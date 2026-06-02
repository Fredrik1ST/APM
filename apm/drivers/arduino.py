'''
Handling of communication between ZED Box (TCP Server) and Arduino (TCP Client) via Ethernet

Message formats:
 * ZED Box -> Arduino - 32 bytes:
 *     - Byte 0.0:    Run signal          (bool)
 *     - Byte 1.0:    Emergency brake     (bool)
 *     - Byte 2.0:    Green LED           (bool)
 *     - Byte 3.0:    Red LED             (bool)
 *     - Byte 4-15:   Spare
 *     - Byte 16-19:  Message nr          (uint32_t)    (Increments per message. Can be used as watchdog / lost message detector.)
 *     - Byte 20-23:  Steer angle         (float)       
 *     - Byte 24-27:  Desired speed PWM   (float)       (Pulse width in microseconds)
 *     - Byte 28-31:  Spare
 *
 * Arduino Feedback -> ZED Box - 32 bytes:
 *     - Byte 0.0:    Run signal          (bool)        (Mirror incoming message)
 *     - Byte 1.0:    Emergency brake     (bool)        (Mirror incoming message)
 *     - Byte 2.0:    Green LED           (bool)        (Read pin value)
 *     - Byte 3.0:    Red LED             (bool)        (Read pin value)
 *     - Byte 6-15:   Spare
 *     - Byte 16-19:  Message nr          (uint32_t)    (Increments per message)
 *     - Byte 20-23:  ESC min PWM         (float)       (ESC calibrated min PWM limit - zero point)
 *     - Byte 24-27:  ESC max PWM         (float)       (ESC calibrated max PWM limit)
 *     - Byte 28-31:  PWM max speed limit (float)       (Hardcoded PWM limit for safety)
 
Notes:

    - Runs a background thread that continuously sends commands and receives feedback.
      Simply create an instance of ArduinoDriver and call start() to begin. Call stop() to shut down.

    - Use get_msg() and write_msg() to read/write messages from the main program in a thread-safe way.

'''

import socket
import struct
import logging
import threading
import time

log = logging.getLogger(__name__)

MESSAGE_SIZE = 32 # bytes
_COMMANDS_FORMAT = '<4?12xIff4x'
_FEEDBACK_FORMAT = '<4?12xIfff'


def _now_us():
    """Microseconds since epoch."""
    return int(time.time_ns() / 1000)


def blink(period=2, duty_cycle=0.5, offset = 0):
    """Blinking function that flips between returning True and False based on parameters. Can be used for e.g. LEDs.

    Args:
        period: Total time of one on/off cycle in seconds.
        duty_cycle: The ratio of the period for which the function returns True.
        offset: Time offset in seconds to shift the blinking pattern.
    """
    t = time.monotonic() + offset

    if period == 0: 
        period = 0.5 # Nuh-uh! 

    return (t % period) < (period * duty_cycle)


def mps_to_pwm(mps = 0.0, factor=20.23, offset_pwm=1540):

    """Convert speed in m/s to PWM signal for the ESC.
    Args:
        mps: Desired speed in meters per second.
        factor: Conversion factor from m/s to PWM microseconds. Found by linear regression of measured speed vs output.
        offset_pwm: The PWM value corresponding to 0 m/s (neutral point). Depends on ESC calibration.
    """
    return (mps * factor) + offset_pwm


class MessageCommands:
    """Formatted message of commands sent from ZED Box to Arduino"""
    def __init__(self):
        self.timestamp = 0.0  # Internal - Microseconds since epoch
        self.run = False
        self.emergency_brake = False
        self.green_led = False
        self.red_led = False
        self.message_nr = 0
        self.steer_angle = 90.0
        self.speed_pwm = 0.0

    @property
    def packed(self):
        """Pack the message into bytes for sending to Arduino"""
        return struct.pack(_COMMANDS_FORMAT,
                           self.run,
                           self.emergency_brake,
                           self.green_led, self.red_led,
                           self.message_nr,
                           self.steer_angle,
                           self.speed_pwm)


class MessageFeedback:
    """Formatted message of feedback sent from Arduino to ZED Box"""
    def __init__(self):
        self.timestamp = 0.0  # Internal - Microseconds since epoch
        self.running = False
        self.emergency_brake = False
        self.green_led = False
        self.red_led = False
        self.message_nr = 0
        self.esc_min_pwm = 0.0
        self.esc_max_pwm = 0.0
        self.pwm_speed_limit = 0.0

    @property
    def packed(self):
        """Pack the message into bytes for sending to ZED Box"""
        return struct.pack(_FEEDBACK_FORMAT,
                           self.running,
                           self.emergency_brake,
                           self.green_led, self.red_led,
                           self.message_nr,
                           self.esc_min_pwm,
                           self.esc_max_pwm,
                           self.pwm_speed_limit)

    @classmethod
    def from_bytes(cls, data):
        """Parse 32 bytes received from the Arduino into a MessageFeedback object."""
        msg = cls()
        (msg.running,
         msg.emergency_brake,
         msg.green_led,
         msg.red_led,
         msg.message_nr,
         msg.esc_min_pwm,
         msg.esc_max_pwm,
         msg.pwm_speed_limit) = struct.unpack(_FEEDBACK_FORMAT, data)
        msg.timestamp = _now_us()
        return msg


class ArduinoDriver:
    '''
    A simple TCP server for interfacing with an Arduino via Ethernet.
    Only one client can connect at a time. Runs a background thread that continuously sends the
    latest commands and receives feedback, automatically re-accepting if the client disconnects.
    '''

    def __init__(self):
        self.host: str | None = None
        self.port: int | None = None
        self.refresh_rate: float = 0.0
        self.server_socket = None
        self.client_socket = None
        self._SOCKET_TIMEOUT = 1.0
        self.client_address = None
        self.is_connected = False
        self._running = False
        self.msg_commands = MessageCommands()
        self.msg_feedback = MessageFeedback()
        self._thread = None
        self._send_lock = threading.Lock()  # Guards self.msg_commands
        self._recv_lock = threading.Lock()  # Guards self.msg_feedback


    def start(self, host='192.168.56.1', port=49152, refresh_rate=50.0,
              ):
        """Start the server socket and background thread for continuous communication. Returns True on success."""
        try:
            # Read config
            self.host = host
            self.port = port
            self.refresh_rate = refresh_rate

            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            log.info(f'Started TCP Server {self.host}:{self.port}. Waiting for connection...')
        except socket.error as e:
            log.error(f'TCP socket error: {e}')
            return False

        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return True


    def _serve(self):
        """Accept a client, run the send/receive loop, re-accept on disconnect."""
        period = 1.0 / self.refresh_rate if self.refresh_rate > 0 else 0.0

        while self._running:
            if not self._accept_client():
                break

            while self._running and self.is_connected:
                loop_start = time.monotonic()

                if not self._send_commands():
                    break
                if not self._receive_feedback():
                    break

                if period:
                    elapsed = time.monotonic() - loop_start
                    if elapsed < period:
                        time.sleep(period - elapsed)

            self._close_client()


    def _accept_client(self):
        """Block until a client connects. Returns False if the server is stopping."""
        self.server_socket.settimeout(self._SOCKET_TIMEOUT)
        while self._running:
            try:
                self.client_socket, self.client_address = self.server_socket.accept()
                self.client_socket.settimeout(self._SOCKET_TIMEOUT)
                self.is_connected = True
                log.info(f'Client {self.client_address} connected')
                return True
            except socket.timeout:
                # Timeout allows for checking if the user has requested a stop (by turning off _running).
                continue
            except socket.error as e:
                if self._running:
                    log.error(f'Error accepting client: {e}')
        return False


    def _send_commands(self):
        """Send the latest commands. Returns False on socket failure."""
        with self._send_lock:
            self.msg_commands.timestamp = _now_us()
            self.msg_commands.message_nr += 1
            data = self.msg_commands.packed
        try:
            self.client_socket.sendall(data)
            log.debug(f'Bytes sent to {self.client_address}: {data.hex()}')
            return True
        except socket.error as e:
            log.error(f'Error sending data: {e}')
            return False


    def _receive_feedback(self):
        """Receive and store one feedback message. Returns False on disconnect/failure."""
        data = self._receive_exact(MESSAGE_SIZE)
        if data is None:
            return False
        try:
            msg = MessageFeedback.from_bytes(data)
        except struct.error as e:
            log.error(f'Error unpacking feedback: {e}')
            return False
        with self._recv_lock:
            self.msg_feedback = msg
        log.debug(f'Bytes received from {self.client_address}: {data.hex()}')
        return True


    def _receive_exact(self, n):
        """Read exactly n bytes from the client. Returns None on disconnect/error."""
        chunks = bytearray()
        while len(chunks) < n:
            if not self._running:
                return None # Stop if server is shutting down
            try:
                chunk = self.client_socket.recv(n - len(chunks))
            except socket.timeout:
                log.error(f'Socket timed out while waiting for feedback from {self.client_address}')
                return None # Feedback is expected from Arduino every cycle, so treat timeout as disconnect
            except socket.error as e:
                log.error(f'Error receiving data from {self.client_address}: {e}')
                return None
            if not chunk:
                log.error(f'No data received from {self.client_address} - disconnected?')
                return None
            chunks.extend(chunk)
        return bytes(chunks)


    def _close_client(self):
        """Close the current client connection (server socket stays open)."""
        self.is_connected = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except socket.error:
                pass
            self.client_socket = None


    def stop(self):
        """Stop the server and close all sockets."""
        
        # Send an "empty" command to force stop ASAP
        self.msg_commands = MessageCommands() 
        self._send_commands()

        self._running = False # Signals the background thread to stop
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=self._SOCKET_TIMEOUT + 1.0)
        # Now that the thread is stopped, we can safely close sockets
        self._close_client()
        if self.server_socket:
            try:
                self.server_socket.close()
            except socket.error:
                pass
            self.server_socket = None
        log.info('TCP Server stopped')


    # -----------------------------------------------------------------------
    # Thread-safe helper functions to read/write messages from main program
    # -----------------------------------------------------------------------

    def write_msg(self, msg: MessageCommands):
        """Thread safe helper to set the commands sent to the Arduino."""
        with self._send_lock:
            self.msg_commands = msg
            return self.msg_commands


    def read_msg(self):
        """Thread safe helper to read the latest feedback from the Arduino (or None)."""
        with self._recv_lock:
            return self.msg_feedback