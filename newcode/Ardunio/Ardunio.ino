
#include <Arduino.h>
#include <Wire.h>
// Motor Pins
#define ENA 6 // Enable pin for Motor A
#define IN1 11 // Motor A input 1
#define IN2 12 // Motor A input 2
#define ENB 5 // Enable pin for Motor B
#define IN3 13 // Motor B input 1
#define IN4 10 // Motor B input 2
#define debug1 1
#define debug2 9
// Opto sensor pins
#define SENSOR_A  3// Opto sensor for Motor A
#define SENSOR_B 2 // Opto sensor for Motor B


// Variables
// Variables for motor speed and distance
volatile unsigned long pulseCountA = 0;
volatile unsigned long pulseCountB = 0;
float wheelCircumference = 20.42035; // Example: in centimeters old: 20.42035
int pulsesPerRevolution = 6;    // Number of pulses per wheel revolution
float targetDistance = 0;
float kP = 13.20; // Proportional gain
float kD = 2.5; // Derivative gai
float lastError = 0;
volatile unsigned long lastPulseTimeA = 0;
volatile unsigned long lastPulseTimeB = 0;
unsigned long debounceInterval = 1200; // Microseconds
unsigned long startTime; 
bool timerRunning = false;
//----[UDP Log]-----

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
    Serial.println("Pulse A detected!");
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeA > debounceInterval) 
    {
        lastPulseTimeA = currentTime;
        pulseCountA++;
    }  
    digitalWrite(debug1, HIGH);
    delayMicroseconds(15);
    digitalWrite(debug1, LOW);  
}

void pulseHandlerB() 
{
    Serial.println("Pulse B detected!");
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeB > debounceInterval) 
    {
        lastPulseTimeB = currentTime;
        pulseCountB++;
    }
    digitalWrite(debug2, HIGH);
    delayMicroseconds(15);
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
 
  

  //----- Setup Robot Motor ----------------
  Serial.begin(115200);

    // Set motor pins as outputs
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);
    pinMode(debug1, OUTPUT);
    pinMode(debug2, OUTPUT);

    // Set opto sensor pins as inputs with internal pull-ups
    pinMode(SENSOR_A, INPUT_PULLUP);
    pinMode(SENSOR_B, INPUT_PULLUP);

    // Configure PWM channels



    // Attach interrupt to opto sensors
    attachInterrupt(digitalPinToInterrupt(SENSOR_A), pulseHandlerA, RISING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_B), pulseHandlerB, RISING);

}

//*******************************************************************************
//*******************************************************************************
//*******************************************************************************
void loop() 
{
//  int speed = 200;
 // int angle = -90.0;
 //   pulseCountA = 0;
 //   pulseCountB = 0;
   //  Serial.print("SENSOR_A: ");
   // Serial.println(digitalRead(SENSOR_A));
  //  Serial.print("SENSOR_B: ");
   // Serial.println(digitalRead(SENSOR_B));
   // delay(500);
        //turnPID(90, 100);
   // driveForwardPID(50, 210);

    turnPID(95, 210);
    delay(5000);
    turnPID(-95, 200);
    delay(50000);

}
  

        //driveForwardPID(61.26, 210);




//*********************************************************** */
//*********************************************************** */
//*********************************************************** */


//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void countPulses() 
{
}
void startTimer() {
    startTime = millis();
    timerRunning = true;
    Serial.println("Timer started!");
}

// Check if 5 seconds have passed
void checkTimer() {
    if (timerRunning && millis() - startTime >= 5000) {
        digitalWrite(8, LOW);
        stopMotors();  // Stop the robot when the timer expires
        timerRunning = false;
    }
}
//*********************************************************** */
//*********************************************************** */
//*********************************************************** */
void setMotorSpeed(int motor, int speed) 
{
    // Constrain speed to 0-255
    speed = constrain(speed, 0, 255);

    if (motor == 1) { // Motor A
        analogWrite(ENA, speed);
    } else if (motor == 2) { // Motor B
        analogWrite(ENB, speed);
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
   startTimer();
  pulsesPerRevolution = 18;
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


    lastError = 0;
    while (true) 
    {
        countPulses(); // Update pulse counts from queues
//        udpLog("------------");
    
          checkTimer();

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
 
  //pulsesPerRevolution = 6;  
 //if (angle > 0 ){
    pulsesPerRevolution = 5;
  //}
    float wheelBase = 16.51; // Distance between wheels in cm (adjust for your robot)

    //float calibrationFactor = 0.87; // Fine-tune for accuracy
  float calibrationFactor = 0.9;
if (angle > 0 ){
  wheelBase = 16.51;
    pulsesPerRevolution = 5;
  //calibrationFactor = 1.95;
}
    float turnCircumference = wheelBase * 3.14159; // Correct turn calculation
    float targetDistance = (abs(angle) / 360.0) * turnCircumference;
    targetDistance *= calibrationFactor;

    pulseCountA = 0;
    pulseCountB = 0;

    while (true) 
    {
        countPulses();

        float distanceA = (pulseCountA / pulsesPerRevolution) * wheelCircumference;
        float distanceB = (pulseCountB / pulsesPerRevolution) * wheelCircumference;

        Serial.print("pulseCountA: "); Serial.println(pulseCountA);
        Serial.print("pulseCountB: "); Serial.println(pulseCountB);
        Serial.print("distanceA: "); Serial.println(distanceA);
        Serial.print("distanceB: "); Serial.println(distanceB);
        Serial.print("targetDistance: "); Serial.println(targetDistance);

        if (distanceA >= (targetDistance - 0.5) && distanceB >= (targetDistance - 0.5)) 
        {
            stopMotors();
            Serial.println("Stopping motors: target reached.");
            break;
        }

        float error = (angle > 0) ? distanceA - distanceB : distanceB - distanceA;
        float P = kP * (error-1.55);
        if (angle < 0 ){
          P = kP * (error-0.10);
        }
        float D = kD * (error - lastError);
        int correction = P + D;



        int speedA = baseSpeed - correction;
        int speedB = baseSpeed + correction;

        if (angle > 0) { // LEft Turn

            digitalWrite(IN1, LOW);
        
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            setMotorSpeed(1, speedB+15);
            setMotorSpeed(2, speedA+15);
        } else { // Rgiht Turn
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            setMotorSpeed(1, speedA+15);
            setMotorSpeed(2, speedB+15);
        }

        lastError = error;
    
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
void threesixty(float angle, int baseSpeed){
  pulsesPerRevolution = 6*4; 
    float wheelBase = 15.0; // Distance between wheels in cm (adjust for your robot)
    float calibrationFactor = 0.87; // Fine-tune for accuracy

    float turnCircumference = wheelBase * 3.14159; // Correct turn calculation
    float targetDistance = (abs(angle) / 360.0) * turnCircumference;
    targetDistance *= calibrationFactor;

    pulseCountA = 0;
    pulseCountB = 0;

    while (true) 
    {
        countPulses();

        float distanceA = (pulseCountA / pulsesPerRevolution) * wheelCircumference;
        float distanceB = (pulseCountB / pulsesPerRevolution) * wheelCircumference;

        Serial.print("pulseCountA: "); Serial.println(pulseCountA);
        Serial.print("pulseCountB: "); Serial.println(pulseCountB);
        Serial.print("distanceA: "); Serial.println(distanceA);
        Serial.print("distanceB: "); Serial.println(distanceB);
        Serial.print("targetDistance: "); Serial.println(targetDistance);

        if (distanceA >= (targetDistance - 0.5) && distanceB >= (targetDistance - 0.5)) 
        {
            stopMotors();
            Serial.println("Stopping motors: target reached.");
            break;
        }

        float error = (angle > 0) ? distanceA - distanceB : distanceB - distanceA;
        float P = kP * error;
        float D = kD * (error - lastError);
        int correction = P + D;



        int speedA = baseSpeed - correction;
        int speedB = baseSpeed + correction;

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
        delay(100); // Small delay for stability
    }
}

//*********************************************************** */
//*********************************************************** */
//*********************************************************** */