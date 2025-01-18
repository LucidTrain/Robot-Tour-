
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <WebServer.h>
#include <Wire.h>
#include "OTA.h"
#include "WifiModule.h"
#include "SDMMC.h"
#include "udpModule.h"
#include "globals.h"
#include "esp32-hal-timer.h"

const char* ap_ssid = "JenksScienceOlympia";
const char* ap_password = "12345678";

// Motor Pins
#define ENA 5 // Enable pin for Motor A
#define IN1 6 // Motor A input 1
#define IN2 7 // Motor A input 2
#define ENB 8 // Enable pin for Motor B
#define IN3 9 // Motor B input 1
#define IN4 10 // Motor B input 2
#define debug1 1
#define debug2 2

// Opto sensor pins
#define SENSOR_A 3 // Opto sensor for Motor A
#define SENSOR_B 4 // Opto sensor for Motor B


// Variables
// Variables for motor speed and distance
volatile unsigned long pulseCountA = 0;
volatile unsigned long pulseCountB = 0;
float wheelCircumference = 20.42035; // Example: in centimeters
int pulsesPerRevolution = 37;    // Number of pulses per wheel revolution
float targetDistance = 0;
float kP = 8.0; // Proportional gain
float kD = 0.5; // Derivative gain
float lastError = 0;
volatile unsigned long lastPulseTimeA = 0;
volatile unsigned long lastPulseTimeB = 0;
unsigned long debounceInterval = 200; // Microseconds

//----[UDP Log]-----
WiFiUDP Udp;
const char* remoteIP = "192.168.4.32"; // Replace with the IP of the computer you want to send data to
unsigned int remotePort = 4411;     // Port on the remote device to send to

void udpLog(String msg);
// Function Declarations
void pulseHandlerA();
void pulseHandlerB();
//void setup();
void countPulses();
void setMotorSpeed(int motor, int speed);
void driveForwardPID(float distance, int baseSpeed);
void turnPID(float angle, int baseSpeed);
void driveBackward(int speed);
void stopMotors();
//void loop();
void initMotorSpeed(int speed);



//*******************************************************************************
//*******************************************************************************
//*******************************************************************************
// ISR for wheel A
void pulseHandlerA() 
{
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeA > debounceInterval) 
    {
        lastPulseTimeA = currentTime;
        pulseCountA++;
    }  
  digitalWrite(debug1, HIGH);
  digitalWrite(debug1, LOW);  
}

// ISR for wheel B
void pulseHandlerB() 
{
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeB > debounceInterval) 
    {
        lastPulseTimeB = currentTime;
        pulseCountB++;
    }
  digitalWrite(debug2, HIGH);
  digitalWrite(debug2, LOW);  
}

//*******************************************************************************
//*******************************************************************************
//*******************************************************************************
void setup()
{
  Serial.begin(115200);
  delay(2000);
  
  // Set up the Access Point
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("Access Point started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  wifiModule = new WiFiModule("PerkinsN", "123456789a");
  wifiModule->begin();

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start OTA task
  setupOTA("esp32-s3-zero");
  startOTATask();
  

  //----- Setup Robot Motor ----------------
  Serial.begin(115200);
  udpLog("Starting program...");
    // Set motor pins as outputs
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(debug1, OUTPUT);
    pinMode(debug2, OUTPUT);

    // Set opto sensor pins as inputs with internal pull-ups
    pinMode(SENSOR_A, INPUT_PULLUP);
    pinMode(SENSOR_B, INPUT_PULLUP);

    // Configure PWM channels
    ledcSetup(0, 5000, 8); // Channel 0, 5 kHz, 8-bit resolution for Motor A
    ledcAttachPin(ENA, 0);
    ledcSetup(1, 5000, 8); // Channel 1, 5 kHz, 8-bit resolution for Motor B
    ledcAttachPin(ENB, 1);

    // Attach interrupt to opto sensors
    attachInterrupt(digitalPinToInterrupt(SENSOR_A), pulseHandlerA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SENSOR_B), pulseHandlerB, CHANGE);

}

//*******************************************************************************
//*******************************************************************************
//*******************************************************************************
void loop() 
{
  int speed = 200;
  int angle = -90.0;

  while (true)
  {
    pulseCountA = 0;
    pulseCountB = 0;
//    driveForwardPID(61.26, 210);
    String tstr = String("Angle Setting: ")+ String(angle);
    udpLog(tstr);
    turnPID(angle, 200);
//    driveBackward(speed);
//    delay(2000);
//    stopMotors();
#if(false)
    initMotorSpeed(200);
    delay(200);
    driveBackward(speed);
    delay(1000);
//    speed+=10;
#endif      
//    angle+=10;
    if (angle>=100)
      angle = 10;
    tstr = String("countA:")+String(pulseCountA)+String("  countB:")+String(pulseCountB);
//    udpLog(tstr);
    delay(5000);
  }

}

//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void udpLog(String msg)
{
//  return;
  String logData = String(millis()) + " ms ->"+msg;
  
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write((const uint8_t*)logData.c_str(), logData.length());
  Udp.endPacket();

}

//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void countPulses() 
{
}

//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void setMotorSpeed(int motor, int speed) 
{
    // Constrain speed to 0-255
    speed = constrain(speed, 0, 255);

    if (motor == 1) { // Motor A
        ledcWrite(0, speed);
    } else if (motor == 2) { // Motor B
        ledcWrite(1, speed);
    }
}
void initMotorSpeed(int speed)
{
  setMotorSpeed(1, speed);
  setMotorSpeed(2, speed);
}
//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void driveForwardPID(float distance, int baseSpeed) 
{
    float lastDistanceA = 0.0;
    float lastDistanceB = 0.0;
    targetDistance = distance;
    pulseCountA = 0;
    pulseCountB = 0;

    // Set both motors to drive forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setMotorSpeed(1, baseSpeed);
    setMotorSpeed(2, baseSpeed);
//    delay(50);
    String tstr = "============================";
    udpLog(tstr);
    tstr = "Starting PID forward...";
    udpLog(tstr);
    tstr = "============================";
    udpLog(tstr);

    lastError = 0;
    while (true) 
    {
        countPulses(); // Update pulse counts from queues
//        udpLog("------------");
    


        float distanceA = (pulseCountA / pulsesPerRevolution) * wheelCircumference;
        distanceA = pulseCountA;
        distanceA /= pulsesPerRevolution;
        distanceA *= wheelCircumference;

        float distanceB = (pulseCountB / pulsesPerRevolution) * wheelCircumference;
        distanceB = pulseCountB;
        distanceB /= pulsesPerRevolution;
        distanceB *= wheelCircumference;



        // Calculate error
        float error = distanceA - distanceB;

        // Proportional term
        float P = kP * error;

        // Derivative term
        float D = kD * (error - lastError);

        // Total correction
        int correction = P + D;

        // Adjust motor speeds
        int speedA = baseSpeed - correction;
        int speedB = baseSpeed + correction;

        if ((distanceB!=lastDistanceB) || (distanceA!=lastDistanceA))
        {
          tstr = String("countA:")+String(pulseCountA)+String("  countB:")+String(pulseCountB);
          udpLog(tstr);

          tstr = String("distanceA:")+String(distanceA,3)+String("  distanceB:")+String(distanceB,3);
          udpLog(tstr);

          tstr = String("error:")+String(error)+String("  speedA:")+String(speedA)+String("  speedB:")+String(speedB);
          udpLog(tstr);
        }

        setMotorSpeed(1, speedB);
        setMotorSpeed(2, speedA);

        lastError = error;
        lastDistanceA = distanceA;
        lastDistanceB = distanceB;

        // Check if target distance reached
        if (distanceA >= (targetDistance) && distanceB >= (targetDistance)) 
        {
          stopMotors();
            break;
        }
//        delay(30);
    }
}

//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void turnPID(float angle, int baseSpeed) 
{
    float lastDistanceA = 11.0;
    float lastDistanceB = 11.0;
    float calibrationFactor = 0.85; // Adjust this value to fine-tune turns (reduce over-rotation)

    float turnCircumference = wheelCircumference * 3.14159; // Half the circumference for a 180-degree turn
    float targetDistance = ((abs(angle) / 360.0) * turnCircumference) / 2;
    targetDistance*=calibrationFactor;

    String tstr = "============================";
    udpLog(tstr);
    tstr = "Turn PID ... ANGLE:" + String(angle);
    udpLog(tstr);
    tstr = "============================";
    udpLog(tstr);

    pulseCountA = 0;
    pulseCountB = 0;

    while (true) 
    {
        countPulses(); // Update pulse counts from queues

        float distanceA = (pulseCountA / pulsesPerRevolution) * wheelCircumference;
        distanceA = pulseCountA;
        distanceA /= pulsesPerRevolution;
        distanceA *= wheelCircumference;

        float distanceB = (pulseCountB / pulsesPerRevolution) * wheelCircumference;
        distanceB = pulseCountB;
        distanceB /= pulsesPerRevolution;
        distanceB *= wheelCircumference;

        // Calculate error
        float error = (angle > 0) ? distanceA - distanceB : distanceB - distanceA; // Adjust error direction based on turn

        // Proportional term
        float P = kP * error;

        // Derivative term
        float D = kD * (error - lastError);

        // Total correction
        int correction = P + D;

        // Adjust motor speeds
        int speedA = baseSpeed - correction;
        int speedB = baseSpeed + correction;

        if ((distanceB!=lastDistanceB) || (distanceA!=lastDistanceA))
        {
          tstr = String("countA:")+String(pulseCountA)+String("  countB:")+String(pulseCountB);
          udpLog(tstr);

          tstr = String("distanceA:")+String(distanceA,3)+String("  distanceB:")+String(distanceB,3);
          udpLog(tstr);

          tstr = String("error:")+String(error)+String("  speedA:")+String(speedA)+String("  speedB:")+String(speedB);
          udpLog(tstr);
        }

        if (angle > 0) { // Right turn
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            setMotorSpeed(1, speedB);
            setMotorSpeed(2, speedA);
        } else { // Left turn
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            setMotorSpeed(1, speedA);
            setMotorSpeed(2, speedB);
        }

        lastError = error;
        lastDistanceA = distanceA;
        lastDistanceB = distanceB;

        // Check if target distance reached
        if (distanceA >= (targetDistance) && distanceB >= (targetDistance)) 
        {
            stopMotors();
            break;
        }
    }
}

//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void driveBackward(int speed) 
{
    // Set both motors to drive backward
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, HIGH);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, HIGH);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);


    setMotorSpeed(1, speed);
    setMotorSpeed(2, speed);
}

//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void stopMotors() 
{
    // Stop both motors
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}


//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
