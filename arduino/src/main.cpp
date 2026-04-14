#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Servo.h>

#define BRAKE_ANGLE 120
#define NO_BRAKE_ANGLE 87
#define RED_LED_PIN 2
#define GREEN_LED_PIN 7
#define STEER_PWM_PIN 5
#define BRAKE_PWM_PIN 6
#define ESC_PWM_PIN 3
#define REGULATOR_PIN 8
#define BUFFER_SIZE 11
#define NO_SPEED 1500
#define MIN_SPEED 1540
#define MAX_SPEED 2000
#define SPEED_LIMIT 1650 // Artificial limit for safety
#define TIMEOUT 3000     // If incoming message takes more time than this (ms) the car will deactivate


// The MAC-address of the arduino Shield (Written on a sticker on the shield)
byte mac[] = {0xA8, 0x61, 0x0A, 0xAF, 0x19, 0x43};

// The IP-address of the computer (MÅ ENDRES TIL ZED BOX)
IPAddress server_ip(192, 168, 56, 1);

// Set the IP-address of the arduino (The 3 first sections must be identical to the IP-address of the computer)
IPAddress ip(192, 168, 56, 2);

// Port numbers from 49152 and up are guaranteed to be unused
unsigned int server_port = 49152;

EthernetClient client;

// Variables to store received data
Servo Steer; // 1199 = max right, 1200-1499 = right, 1500-1599 = 0 degrees, 1600-1899 = left, 1900 = max left
Servo Brake;
Servo ESC;  // 1500-1599: stationary, 1600 = min speed, 2000 = max speed

float desiredSpeed;

bool greenBool;
float steerAngle = 90; // 90 = straight, 45 = max right, 135 = max left
int ESCpwm = NO_SPEED;


bool idle = true;
bool runFinished = false;

unsigned long timeStamp; 
unsigned long watchdogTimer;


void setOutputs(byte buffer[BUFFER_SIZE]);     // Set vehicle parameters
//void printBuffer(byte buffer[11]);          // Print received values

void setup() {
  //pinMode(ESC_PWM_PIN, OUTPUT);
  //digitalWrite(ESC_PWM_PIN, LOW);
  ESC.attach(ESC_PWM_PIN);
  ESC.writeMicroseconds(ESCpwm);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  
  Brake.attach(BRAKE_PWM_PIN, 650, 2250);
  Brake.write(NO_BRAKE_ANGLE);

  Steer.attach(STEER_PWM_PIN,  863, 2063);
  Steer.write(steerAngle);
  

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

  if ( ((millis() - watchdogTimer) > TIMEOUT) && !idle){ //In case communication is broken
      idle = true;
      ESCpwm = NO_SPEED;
      ESC.writeMicroseconds(ESCpwm);
      Brake.write(BRAKE_ANGLE);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GREEN_LED_PIN, LOW);
      runFinished = true;
    }


  //Receive angle
  if (client.connected()) {
    

    if (client.available() >= BUFFER_SIZE) {
      watchdogTimer = millis(); // Reset watchdog timer

      // Read data into buffer
      byte buffer[BUFFER_SIZE];
      client.read(buffer, BUFFER_SIZE);

      if (!runFinished) setOutputs(buffer); // Set vehicle parameters
      //printBuffer(buffer); // Print values, comment out in practical application
    }
  }
  else {
    client.connect(server_ip, server_port);
  }

}

void setOutputs(byte buffer[BUFFER_SIZE]) {


//sig,brake,angle,speed
  
  memcpy(&steerAngle, buffer + 3, sizeof(steerAngle));
  memcpy(&desiredSpeed, buffer + 7, sizeof(desiredSpeed));
  
  //Serial.println(desiredSpeed);
  


  if (buffer[0] == true) { // On signal

    if (buffer[1] == true) { //Emergency brake
      ESCpwm = NO_SPEED;
      ESC.writeMicroseconds(ESCpwm);
      Brake.write(BRAKE_ANGLE);
    }

    else{
      //constrain angle
      steerAngle = constrain(steerAngle, 45, 135);
      Steer.write(steerAngle);

      digitalWrite(RED_LED_PIN, LOW);

      ESCpwm = static_cast<int>(desiredSpeed * 20.23 + MIN_SPEED); // found through testing, PID output desiredSpeed correspons roughly to m/s

      if (ESCpwm <= MIN_SPEED) { // Shouldn't be a possible case
        ESCpwm = NO_SPEED;
      }

      else if (ESCpwm > SPEED_LIMIT) { // Desired speed is higher than allowed
        ESCpwm = SPEED_LIMIT;
        digitalWrite(RED_LED_PIN, HIGH);

      }

      else { // Valid speed
        digitalWrite(RED_LED_PIN, LOW);
      }

      greenBool = buffer[2];
      digitalWrite(GREEN_LED_PIN, greenBool);
      
      timeStamp = millis(); // Start timer
      //startRun = true;
      idle = false;
      //startRun = false;
      timeStamp = millis();
      ESC.writeMicroseconds(ESCpwm);
      
    }


  }
  else { // ZED Box has sent deactivation message
    
    ESCpwm = NO_SPEED;
    ESC.writeMicroseconds(ESCpwm);
    Brake.write(BRAKE_ANGLE);
    idle = true;
    runFinished = true;
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
  }

}