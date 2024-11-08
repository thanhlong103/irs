#include <Wire.h>
#include <Servo.h>

int servoPin = 6;
Servo grabber;
int command;
int direction;

// Left motor
int ENA = 7;
int IN1 = 8;
int IN2 = 9;

// Right motor
int ENB = 12;
int IN3 = 10;
int IN4 = 11;

// For encoder
int ENCLA = 2;
int ENCLB = 3;

int ENCRA = 18;
int ENCRB = 19;

long previousMillis = 0;
long currentMillis = 0;
float rpm = 0;
float rpmL = 0;
float rpmR = 0;

float v = 0;
float w = 0;

float l = 0.18;
float r = 0.025;

float samplingrate = 20;
int maxrpm = 106;

float vl = 0;
float vr = 0;

float x = 0; 
float y = 0;
float theta = 0.0000001;

void setup() {
  Serial.setTimeout(1);
  Serial.begin(9600);

  grabber.attach(servoPin); 

  // Set all the motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void move(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void back(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop() {
  //  Turn off motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveWithInput(float v, float w){
  vl = (2*v - (l*w))/2;
  vr = (2*v + (l*w))/2;

  float ratio = vl/vr;

  int analogvl = analogConverter(vl, ratio);
  float analogvr = analogConverter(vr, 1);

  if (direction == 0){
    move(analogvl, analogvr);
  }
  else if (direction == 1){
    back(analogvl, analogvr);
  }
}

int analogConverter(float v, float ratio){
  float rpm = (v*60)/(2*r*3.1412);
  float analog = rpm/maxrpm * 255;

  if (analog >= 255){
    analog = 150*ratio;
  }
  analog = constrain(analog, 0, 255);
  return analog;
}

void testCom(){
  while (!Serial.available());
  command = Serial.readString().toInt();
  if (command == 1){
    v = 0.15;
    w = 0.0;
    direction = 0;
    Serial.print(command);
  }
  else if (command == 0){
    v = 0.0;
    w = 0.0;
    direction = 0;
    Serial.print(command);
  }
  else if (command == 2){
    v = 0.15;
    w = 0.5;
    direction = 0;
    Serial.print(command);
  }
  else if (command == 3){
    v = 0.15;
    w = -0.5;
    direction = 0;
    Serial.print(command);
  }
  else if (command == 4){
    v = 0.15;
    w = 0.0;
    direction = 1;
    Serial.print(command);
  }
  else if (command == 5){
    grabber.write(85);
  }
  else if (command == 6){
    grabber.write(0);
  }
  moveWithInput(v,w);
}

void  loop() {
  testCom();
}
