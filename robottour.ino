
#define ENA_PIN 9
#define IN1_PIN 7
#define IN2_PIN 5
#define IN3_PIN 4
#define IN4_PIN 3
#define BNB_PIN 2


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(7, OUTPUT);
pinMode(6, OUTPUT);
pinMode(5, OUTPUT);
pinMode(4, OUTPUT);
pinMode(3, OUTPUT);
pinMode(2, OUTPUT);
pinMode(9, OUTPUT);
analogWrite(9, 255);
analogWrite(6, 255);
}

void loop() {
  // put your main code here, to run repeatedly:

forward(5000, 100);
delay(5000);
  left(150, 255);
  delay(5000);
  right(150, 200);
  delay(5000);
  backward(2000, 123);
  delay(5000);
  stop();
  //backward();
  //  Serial.println("Backward");
  //left();
    //Serial.println("Left");
  //right();
    //Serial.println("Right");
  //stop();
    //Serial.println("Stop");
}

void forward(int delay2, int speed){
   analogWrite(9, speed);
  analogWrite(6, speed);
  digitalWrite(IN1_PIN, HIGH);
 
  digitalWrite(IN3_PIN, HIGH);

  digitalWrite(IN4_PIN, LOW);
   digitalWrite(IN2_PIN, LOW); 

    delay(delay2);
    analogReset();
}
void backward(int delay2, int speed){

   analogWrite(9, speed);
  analogWrite(6, speed);
  digitalWrite(IN2_PIN, HIGH);

  digitalWrite(IN4_PIN, HIGH);
  delay(delay2);
    digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN3_PIN, LOW);
      analogReset();
  
}
void left(int delay2, int speed){
     analogWrite(9, speed);
  analogWrite(6, speed);
    digitalWrite(IN1_PIN, HIGH);
    delay(delay2);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogReset();
}
void right(int delay2, int speed){
     analogWrite(9, speed);
  analogWrite(6, speed);
      digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  delay(delay2);
  digitalWrite(IN4_PIN, LOW);
  analogReset();
}
void stop(){
      digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogReset();

}
void analogReset(){
  analogWrite(6, 255);
  analogWrite(9, 255);
}