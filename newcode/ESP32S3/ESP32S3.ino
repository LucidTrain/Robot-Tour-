
#include <Arduino.h>
#include <Wire.h>
// Motor Pins
#define ENA 6 // Enable pin for Motor A
#define IN1 7 // Motor A input 1
#define IN2 15 // Motor A input 2
#define ENB 18 // Enable pin for Motor B
#define IN3 16 // Motor B input 1
#define IN4 17 // Motor B input 2
#define debug1 1
#define debug2 9
// Opto sensor pins
#define SENSOR_A  4// Opto sensor for Motor A
#define SENSOR_B 5 // Opto sensor for Motor B

TaskHandle_t sensorTaskHandle;
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
volatile bool pulseA = false;
volatile bool pulseB = false;

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
pulseA = true; 
}

void pulseHandlerB() 
{
pulseB = true;
}
void handlePulseA() {
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeA > debounceInterval) 
    {
        lastPulseTimeA = currentTime;
        pulseCountA++;
    }  

}

void handlePulseB() {
    unsigned long currentTime = micros();
    if (currentTime - lastPulseTimeB > debounceInterval) 
    {
        lastPulseTimeB = currentTime;
        pulseCountB++;
    }
}

void sensorTask(void *pvParameters) {
    while (1) {
        if (pulseA) {
            handlePulseA();
            pulseA = false;
        }
        if (pulseB) {
            handlePulseB();
            pulseB = false;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Prevents task from blocking
    }
}
//*******************************************************************************
//*******************************************************************************
//*******************************************************************************
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Delay");
  delay(2000);
  Serial.println("RUnning progrm");
  // Set up the Access Point
 
  

  //----- Setup Robot Motor ----------------
  

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
    digitalWrite(ENA, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENB, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);


    // Attach interrupt to opto sensors
    attachInterrupt(digitalPinToInterrupt(SENSOR_A), pulseHandlerA, RISING);
    attachInterrupt(digitalPinToInterrupt(SENSOR_B), pulseHandlerB, RISING);
//start sensrr task
    xTaskCreatePinnedToCore(
        sensorTask,   // Function to run
        "SensorTask", // Name
        1000,         // Stack size
        NULL,         // Parameters
        1,            // Priority
        &sensorTaskHandle, // Task handle
        0             // Core (1 = APP core)
    );

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
        //turnPID(90, 220);
Serial.println("Starting 2 motor contorl test");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
delay(5000);
    digitalWrite(ENA, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENB, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(150);
    driveForwardPID(50, 210);
    turnPID(-95, 255);
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
  pulsesPerRevolution = 37;
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
    
        Serial.println("LoopDrive");

        float distanceA = (pulseCountA / pulsesPerRevolution) * wheelCircumference;
        float distanceB = (pulseCountB / pulsesPerRevolution) * wheelCircumference;
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
        Serial.println("Error calc done");
        if ((distanceB!=lastDistanceB) || (distanceA!=lastDistanceA))
        {
         
        }
        Serial.println("Setting motor speed");
        setMotorSpeed(1, speedB);
        setMotorSpeed(2, speedA);
        Serial.println("Set motor speed");
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
  pulsesPerRevolution = 6; 
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