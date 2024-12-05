#define ENA_PIN 9
#define IN1_PIN 7
#define IN2_PIN 5
#define IN3_PIN 4
#define IN4_PIN 8
#define ENB_PIN 6
#define SensorLeft 3
#define SensorRight 2
 
// Variables to count slots
volatile int leftCounter = 0;
volatile int rightCounter = 0;
const float WHEEL_CIRCUMFERENCE = 209.4; // Circumference in mm if wheel diameter is 66.5mm (2.62 inches)
const int SLOTS_PER_REVOLUTION = 30;
 
 
 
//*********************************************
//*********************************************
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
//  pinMode(6, OUTPUT);
//  pinMode(5, OUTPUT);
//  pinMode(4, OUTPUT);
//  pinMode(3, OUTPUT);
//  pinMode(2, OUTPUT);
//  pinMode(9, OUTPUT);
  analogWrite(ENA_PIN, 255);
  analogWrite(ENB_PIN, 255);
 
  // Add pin modes for optocoupler sensors (assuming you're using analog pins A0 and A1 for simplicity)
  pinMode(SensorLeft, INPUT);
  pinMode(SensorRight, INPUT);
 
  // Reset slot counters
  resetCounters();
 
 
  // Attach interrupts if your sensors can trigger on state change
  attachInterrupt(digitalPinToInterrupt(SensorLeft), leftSensorInterrupt, CHANGE); // Assuming A0 for left wheel
  attachInterrupt(digitalPinToInterrupt(SensorRight), rightSensorInterrupt, CHANGE); // Assuming A1 for right wheel
 delay(5000);
}
 
void loop()
{
  //use 28 for left/right turn 
  //use stop(); then delay(200); before executing any turn command
  // use 88 for going 50cm straight
  forward2(88,255);
  stop();
  delay(200);
 //left2(28, 255);

  //forward2(1,200);
  //forward2(1,200);
  //forward2(1,200);
  //forward2(1,200);
  //forward2(1,200);
  //forward2(1,200);
  //forward2(1,200);
  //forward2(1,100);
 
  
   
  //delay(500);
  //left3(1,200);
  //forward2(1,200);
  //forward2(1,200);
  //forward2(1,100);
  //delay(1000);
  //right3(1,200);
  //forward2(1,200);
  //forward2(1,200);
  //left3(1,200);
  //forward2(1,200);
  //forward2(1,200);
 
 
 
while(1)
{
 
}
  //to go 50cm do forward(1567, 255);
  left3(1567, 200);
  stop();
  Serial.print(leftCounter);
  Serial.print(" - ");
  Serial.println(rightCounter);
   delay(2000);
  forward2(1,200);
  forward2(1,200);
  forward2(1,200);
   delay(2000);
  right3(1,200);
   delay(2000);
  forward2(1,200);
  forward2(1,200);
  forward2(1,200);
   delay(2000);
}
 
 
// Reset counters
void resetCounters()
{
  leftCounter = rightCounter = 0;
}
 
// Interrupt service routines for counting slots
void leftSensorInterrupt()
{
  leftCounter++;
}
 
void rightSensorInterrupt()
{
  rightCounter++;
}
 
// Function to calculate distance traveled based on slot count
float getDistanceTraveled(int slots)
{
  return (slots / (float)SLOTS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE; // Convert slots to distance in mm
}
 
 
void forward2(int turns, int speed)
{
  resetCounters();
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
 
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
 
  // Wait for both wheels to complete one rotation
  while (leftCounter < turns || rightCounter < turns)
  {
    // If left wheel has completed its rotation, stop it
    if (leftCounter >= turns) {
      digitalWrite(IN1_PIN, LOW);
      analogWrite(ENA_PIN, 0);
    }
   
    // If right wheel has completed its rotation, stop it
    if (rightCounter >= turns) {
      digitalWrite(IN3_PIN, LOW);
      analogWrite(ENB_PIN, 0);
    }
  }
  stop();
 
}
 
 
void left3(int turns, int speed)
{
  resetCounters();
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
 
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
 
  // Wait for both wheels to complete one rotation
  while (leftCounter < 11 || rightCounter < 11)
  {
    // If left wheel has completed its rotation, stop it
    if (leftCounter >= 11)
    {
      digitalWrite(IN1_PIN, LOW);
      analogWrite(ENA_PIN, 0);
    }
   
    // If right wheel has completed its rotation, stop it
    if (rightCounter >= 11)
    {
      digitalWrite(IN3_PIN, LOW);
      analogWrite(ENB_PIN, 0);
    }
  }
  stop();
 
}
 
 
void left2(int turns, int speed)
{
  resetCounters();
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);    //Right Wheel Turns ONLY
 
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
 
// If right wheel has completed its rotation, stop it
  while (rightCounter < turns)
  {
    if (rightCounter >= turns)
    {
      digitalWrite(IN3_PIN, LOW);
      analogWrite(ENB_PIN, 0);
    }
  }
 
  stop();
 
}
 
 
void right3(int turns, int speed)
{
  resetCounters();
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
 
  digitalWrite(IN4_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
 
  // Wait for both wheels to complete one rotation
  while (leftCounter < 11 || rightCounter < 11)
  {
    // If left wheel has completed its rotation, stop it
    if (leftCounter >= 11)
    {
      digitalWrite(IN1_PIN, LOW);
      analogWrite(ENA_PIN, 0);
    }
   
    // If right wheel has completed its rotation, stop it
    if (rightCounter >= 11)
    {
      digitalWrite(IN3_PIN, LOW);
      analogWrite(ENB_PIN, 0);
    }
  }
  stop();
 
}
 
 
 
void right2(int turns, int speed)
{
  resetCounters();
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
 
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
 
// If left wheel has completed its rotation, stop it
  while (leftCounter < turns)
  {
    if (leftCounter >= turns)
    {
      digitalWrite(IN1_PIN, LOW);
      analogWrite(ENA_PIN, 0);
    }
  }
  stop();
 
}
 
 
 
void forward(int delay2, int speed)
{
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, HIGH);
 
  digitalWrite(IN3_PIN, HIGH);
 
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
 
  delay(delay2);
  analogReset();
}
void backward(int delay2, int speed)
{
 
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN2_PIN, HIGH);
 
  digitalWrite(IN4_PIN, HIGH);
  delay(delay2);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  analogReset();
 
}
void left(int delay2, int speed)
{
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, HIGH);
  delay(delay2);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogReset();
}
void right(int delay2, int speed)
{
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  delay(delay2);
  digitalWrite(IN4_PIN, LOW);
  analogReset();
}
 
void stopLeft()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}
void stopRight()
{
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}
void stop()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogReset();
 
}
void analogReset()
{
  analogWrite(ENB_PIN, 255);
  analogWrite(ENA_PIN, 255);
}
 
 
 
 
 
 
