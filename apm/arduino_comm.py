'''
Handling of communication between ZED Box and Arduino via Ethernet (TCP Socket)

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
 * Arduino -> ZED Box - 32 bytes:
 *     - Byte 0.0:    Run signal          (bool)        (Mirror incoming message)
 *     - Byte 1.0:    Emergency brake     (bool)        (Mirror incoming message)
 *     - Byte 2.0:    Green LED           (bool)        (Read pin value)
 *     - Byte 3.0:    Red LED             (bool)        (Read pin value)
 *     - Byte 6-15:   Spare
 *     - Byte 16-19:  Message nr          (uint32_t)    (Increments per message)
 *     - Byte 20-23:  ESC min PWM         (float)       (ESC calibrated min PWM limit - zero point)
 *     - Byte 24-27:  ESC max PWM         (float)       (ESC calibrated max PWM limit)
 *     - Byte 28-31:  PWM max speed limit (float)       (Hardcoded PWM limit for safety)

'''

import socket
import struct
import logging

log = logging.getLogger(__name__)

class message_commands:
    """Formatted message of commands sent from ZED Box to Arduino"""
    def __init__(self):
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
        return struct.pack('<4?12xIff4x', 
                           self.run, 
                           self.emergency_brake, 
                           self.green_led, self.red_led, 
                           self.message_nr, 
                           self.steer_angle, 
                           self.speed_pwm)
    
class message_feedback:
    """Formatted message of feedback sent from Arduino to ZED Box"""
    def __init__(self):
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
        return struct.pack('<4?12xIfff', 
                           self.running, 
                           self.emergency_brake, 
                           self.green_led, self.red_led, 
                           self.message_nr, 
                           self.esc_min_pwm, 
                           self.esc_max_pwm, 
                           self.pwm_speed_limit)


class TCPServer:
    '''
    A simple TCP server for communicating with an Arduino (or another TCP client) via Ethernet.
    Only one client can connect at a time.
    '''

    def __init__(self, host='192.168.56.1', port=49152):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_connected = False
    

    def start(self):
        """Start TCP server and wait for a client to connect."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow immediate reconnections

            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1) # Wait for a client to connect

            log.info(f'Started TCP Server {self.host}:{self.port}. Waiting for connection...')

            self.client_socket, self.client_address = self.server_socket.accept()
            self.is_connected = True
            log.info(f'Client {self.client_address} connected')

            return True
        
        except socket.error as e:
            log.error(f'TCP socket error: {e}')
            return False
        

    def send(self, data):
        """Send a byte message to the connected client."""
        if self.is_connected and self.client_socket:
            try:
                self.client_socket.sendall(data)
                log.debug(f'Bytes sent to {self.client_address}: {data.hex()}')
                return True
            except socket.error as e:
                log.error(f'Error sending data: {e}')
                return False
    
    def receive(self, data_length=32):
        """Receive a byte message from the connected client."""
        if self.is_connected and self.client_socket:
            try:
                data = self.client_socket.recv(data_length)
                log.debug(f'Bytes received from {self.client_address}: {data.hex()}')
            except socket.error as e:
                log.error(f'Error receiving data: {e}')
                return None
    
    def stop(self):
        """Stop the server and close all sockets."""
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        self.is_connected = False
        log.info('TCP Server stopped')