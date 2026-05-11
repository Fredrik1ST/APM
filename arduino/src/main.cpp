/* Based on code from the 2024 master's thesis "Development of an Autonomous Rabbit for Running on a Track"
 * by Kvamme et al. at NTNU, Norway.
 *
 * Communication with the ZED Box over Ethernet.
 * Connects to the ZED Box as a TCP client and receives setpoints and commands for the car's:
 *     - Electronic Speed Controller (ESC)
 *     - Steering servo
 *     - Braking servo
 *     - Rear LED lights (green and red)
 * 
 * Incoming message structure - 32 bytes:
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
 * Outgoing message structure - 32 bytes:
 *     - Byte 0.0:    Run signal          (bool)        (Mirror incoming message)
 *     - Byte 1.0:    Emergency brake     (bool)        (Mirror incoming message)
 *     - Byte 2.0:    Green LED           (bool)        (Read pin value)
 *     - Byte 3.0:    Red LED             (bool)        (Read pin value)
 *     - Byte 6-15:   Spare
 *     - Byte 16-19:  Message nr          (uint32_t)    (Increments per message)
 *     - Byte 20-23:  ESC min PWM         (float)       (ESC calibrated min PWM limit - zero point)
 *     - Byte 24-27:  ESC max PWM         (float)       (ESC calibrated max PWM limit)
 *     - Byte 28-31:  PWM max speed limit (float)       (Hardcoded PWM limit for safety)
 * 
 * Notes:
 *     - Programmed for Arduino Uno (8-bit) with Arduino Ethernet Shield 2 (W5500 chip).
 *         - Shield supports 8 independent sockets. A socket's internal RX and TX memory buffer is 2KB each.
 *         - Float data type is truncated and not rounded when assigned to an int. (Angle of 90.9 becomes 90, not 91.)
 *         - Not tested on 32-bit boards, but data types are chosen to be compatible.
 *     - Servo library interprets values below 200 as angles, and values above 200 as microsecond pulse widths (PWM).
 *     - PWM values are gained from testing. A new ESC may need to be calibrated anew by teaching max/min values.
*/
// ============================================
// PREAMBLE (Includes and definitions)
// ============================================

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Servo.h>

#define RED_LED_PIN 2
#define GREEN_LED_PIN 7
#define STEER_PWM_PIN 5
#define BRAKE_PWM_PIN 6
#define ESC_PWM_PIN 3
#define REGULATOR_PIN 8

// Incoming TCP message offsets
#define OFFSET_RUN_SIGNAL 0
#define OFFSET_EMERGENCY_BRAKE 1
#define OFFSET_GREEN_LED 2
#define OFFSET_RED_LED 3
#define OFFSET_MESSAGE_NR 16
#define OFFSET_STEER_ANGLE 20
#define OFFSET_DESIRED_SPEED_PWM 24

// Outgoing TCP message offsets
#define OFFSET_OUT_RUN_SIGNAL 0
#define OFFSET_OUT_EMERGENCY_BRAKE 1
#define OFFSET_OUT_GREEN_LED 2
#define OFFSET_OUT_RED_LED 3
#define OFFSET_OUT_MESSAGE_NR 16
#define OFFSET_OUT_ESC_MIN_PWM 20
#define OFFSET_OUT_ESC_MAX_PWM 24
#define OFFSET_OUT_PWM_SPEED_LIMIT 28

// ESC PWM pulse widths in microseconds
#define SPEED_PWM_STOP 1500   // Zero point PWM when tuning ESC - Stands still around 1500-1599
#define SPEED_PWM_MIN 1540    
#define SPEED_PWM_MAX 2000    // Max PWM when tuning ESC - Roughly 22.7 m/s or 80 km/h.
#define SPEED_PWM_LIMIT 1650  // Artificial limit for safety - Roughly 5.4 m/s or 19.6 km/h

// Steering servo angles (note: values below 200 are treated as angles by servo library)
#define STEER_ANGLE_NEUTRAL 90
#define STEER_ANGLE_MAX_RIGHT 45
#define STEER_ANGLE_MAX_LEFT 135
#define STEER_PWM_MIN 863
#define STEER_PWM_MAX 2063

// Braking servo angles (note: values below 200 are treated as angles by servo library)
#define BRAKE_ANGLE_ON 120
#define BRAKE_ANGLE_OFF 87
#define BRAKE_PWM_MIN 650
#define BRAKE_PWM_MAX 2250

// Network parameters
#define INPUT_BUFFER_SIZE 32        // Byte size of TCP message from ZED Box
#define OUTPUT_BUFFER_SIZE 32       // Byte size of TCP message to ZED Box
#define TIMEOUT 1000                // If incoming message takes more time than this (ms) the car will deactivate
#define ZEDBOX_PORT 49152           // TCP port on ZED Box - 49152 and up are unused
#define ZEDBOX_IP 192, 168, 56, 1
#define ARDUINO_IP 192, 168, 56, 2
#define ARDUINO_MAC 0xA8, 0x61, 0x0A, 0xAF, 0x19, 0x43 // Mac address of arduino shield, read from sticker on shield


// ============================================
// VARIABLE AND OBJECT DEFINITIONS
// ============================================

// Network setup
EthernetClient client;
IPAddress ip(ARDUINO_IP);
byte mac[] = {ARDUINO_MAC};
IPAddress server_ip(ZEDBOX_IP);
uint16_t server_port = ZEDBOX_PORT;

// ESC and servos
Servo Steer; // 1199 = max right, 1200-1499 = right, 1500-1599 = 0 degrees, 1600-1899 = left, 1900 = max left
Servo Brake;
Servo ESC;  // 1500-1599: stationary, 1600 = min speed, 2000 = max speed
int16_t ESCpwm = SPEED_PWM_STOP;


// Variables for received data
byte buffer_in[INPUT_BUFFER_SIZE];
bool runSignal;
bool emergencyBrake;
bool greenLED;
bool redLED;
uint32_t msgNr_in = 0;
// float desiredSpeed; // Legacy code: ESCpwm = desiredSpeed * 20.23 + MIN_SPEED
float desiredSpeed_pwm = SPEED_PWM_STOP;
float steerAngle = STEER_ANGLE_NEUTRAL;

// Variables for outgoing data
byte buffer_out[OUTPUT_BUFFER_SIZE];
uint32_t msgNr_out = 0;
float escMinPWM = SPEED_PWM_MIN;
float escMaxPWM = SPEED_PWM_MAX;
float pwmSpeedLimit = SPEED_PWM_LIMIT;

// Internal logic

bool idle = true;
unsigned long watchdogTimer;


// ============================================
// FORWARD FUNCTION DECLARATIONS - defined after loop())
// ============================================

void recvData(byte buffer[INPUT_BUFFER_SIZE]);      // Read incoming message buffer and store data in variables
void setOutputs();                                  // Set outputs according to variables updated by rcvData()
void sendData();                                    // Send feedback message to ZED Box


// ============================================
// SETUP AND LOOP
// ============================================

void setup() {
  ESC.attach(ESC_PWM_PIN);
  ESC.writeMicroseconds(ESCpwm);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  
  Brake.attach(BRAKE_PWM_PIN, BRAKE_PWM_MIN, BRAKE_PWM_MAX);
  Brake.write(BRAKE_ANGLE_OFF);

  Steer.attach(STEER_PWM_PIN,  STEER_PWM_MIN, STEER_PWM_MAX);
  Steer.write(STEER_ANGLE_NEUTRAL);
  
  // Disable SD Card (Not sure if necessary, 
  // but read in an arduino forum that it could cause issues with ethernet comms)
  // arduino docs also say that pins 4, 11, 12 and 13 cannot be used for digital output because of ethernet
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // Turn the ZED Box on by activating the isolated voltage regulator
  pinMode(REGULATOR_PIN, OUTPUT);
  digitalWrite(REGULATOR_PIN, HIGH);

  // Start Ethernet connection
  Ethernet.begin(mac, ip); 
  delay(5000);
  
  // Connect to server
  client.connect(server_ip, server_port);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);

}


void loop() {
  // put your main code here, to run repeatedly:

  // Stop if no message is received for a certain time
  if ( ((millis() - watchdogTimer) > TIMEOUT) && !idle){
      idle = true;
      ESCpwm = SPEED_PWM_STOP;
      ESC.writeMicroseconds(ESCpwm);
      Brake.write(BRAKE_ANGLE_ON);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, LOW);
  }

  // Normal operation: Receive data -> set outputs accordingly -> send feedback message
  if (client.connected()) {
    if (client.available() >= INPUT_BUFFER_SIZE) {
      watchdogTimer = millis(); // Reset timer
      idle = false;

      recvData(buffer_in); // Read message from ZED Box and updates variables

      setOutputs(); // Set outputs based on the updated variables from ZED Box

      sendData(); // Send feedback message back to ZED Box
    }
  } else {
      client.connect(server_ip, server_port); // Auto-reconnect
  }

}


// ============================================
// HELPER FUNCTION DEFINITIONS
// ============================================

// Read incoming message from ZED Box and store in buffer
void recvData(byte buffer[INPUT_BUFFER_SIZE]) {
  client.read(buffer, INPUT_BUFFER_SIZE);
  // Extract data from buffer to respective variables
  runSignal = buffer[OFFSET_RUN_SIGNAL];
  emergencyBrake = buffer[OFFSET_EMERGENCY_BRAKE];
  greenLED = buffer[OFFSET_GREEN_LED];
  redLED = buffer[OFFSET_RED_LED];
  memcpy(&msgNr_in, buffer + OFFSET_MESSAGE_NR, sizeof(msgNr_in));
  memcpy(&steerAngle, buffer + OFFSET_STEER_ANGLE, sizeof(steerAngle));
  memcpy(&desiredSpeed_pwm, buffer + OFFSET_DESIRED_SPEED_PWM, sizeof(desiredSpeed_pwm));
}


// Set outputs based on received data from ZED Box
void setOutputs() {
  // LEDs
  digitalWrite(GREEN_LED_PIN, greenLED);
  digitalWrite(RED_LED_PIN, redLED);

  if (!runSignal || emergencyBrake) { // Brake
      ESCpwm = SPEED_PWM_STOP;
      ESC.writeMicroseconds(ESCpwm);
      Brake.write(BRAKE_ANGLE_ON);

  } else{ // Normal operation: let ZED Box control the car
      steerAngle = constrain(steerAngle, STEER_ANGLE_MAX_RIGHT, STEER_ANGLE_MAX_LEFT);
      Steer.write(steerAngle);

      ESCpwm = static_cast<int>(desiredSpeed_pwm);
      if (ESCpwm <= SPEED_PWM_MIN) {ESCpwm = SPEED_PWM_STOP;}
      else if (ESCpwm > SPEED_PWM_LIMIT) {ESCpwm = SPEED_PWM_LIMIT;}
      ESC.writeMicroseconds(ESCpwm);
  }
}


// Send feedback to ZED Box
void sendData() {
  buffer_out[OFFSET_OUT_RUN_SIGNAL] = runSignal;
  buffer_out[OFFSET_OUT_EMERGENCY_BRAKE] = emergencyBrake;
  buffer_out[OFFSET_OUT_GREEN_LED] = digitalRead(GREEN_LED_PIN);
  buffer_out[OFFSET_OUT_RED_LED] = digitalRead(RED_LED_PIN);
  memcpy(buffer_out + OFFSET_OUT_MESSAGE_NR, &msgNr_out, sizeof(msgNr_out));
  memcpy(buffer_out + OFFSET_OUT_ESC_MIN_PWM, &escMinPWM, sizeof(escMinPWM));
  memcpy(buffer_out + OFFSET_OUT_ESC_MAX_PWM, &escMaxPWM, sizeof(escMaxPWM));
  memcpy(buffer_out + OFFSET_OUT_PWM_SPEED_LIMIT, &pwmSpeedLimit, sizeof(pwmSpeedLimit));

  client.write(buffer_out, OUTPUT_BUFFER_SIZE);
  msgNr_out++;
}