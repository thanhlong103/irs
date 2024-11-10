#include <Wire.h>
#include <Servo.h>

int servoPin = 6;
Servo grabber;
int command;
int direction;

volatile long leftEnCount = 0;
volatile long rightEnCount = 0;

float pulsePerRound = 2000;
// float pulsePerRoundR = 1500;

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

float samplingrate = 10;
int maxrpm = 106;

float vl = 0;
float vr = 0;

float x = 0; 
float y = 0;
float theta = 0.0000001;

int leftOrright = 0;

// Interrupt service routines for encoder pulses
void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}

void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}

void setup() {
  Serial.setTimeout(1);
  Serial.begin(9600);

  grabber.attach(servoPin); 

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCRB), rightEnISRB, RISING);


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

void rotateRight(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rotateLeft(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
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

void moveWithInput(float v, float w) {
  vl = (2 * v - (l * w)) / 2;
  vr = (2 * v + (l * w)) / 2;

  float ratio = vl / vr;
  int analogvl = analogConverter(vl, ratio);
  int analogvr = analogConverter(vr, 1);

  if (direction == 0) {
    if (v == 0 && w > 0){
      rotateLeft(255,255);
      leftOrright = 1;
    }
    else if (v == 0 && w < 0){
      rotateRight(255,255);
      leftOrright = 2;
    }
    else {
      move(analogvl, analogvr);
    }
    leftOrright = 0;
  } else if (direction == 1) {
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

void keyboard(){
  if (!Serial.available());
  command = Serial.readString().toInt();
  if (command == 1){
    move(255,255);
    leftOrright = 0;
  }
  else if (command == 0){
    stop();
  }
  else if (command == 2){
    rotateLeft(255,255);
    leftOrright = 1;
  }
  else if (command == 3){
    rotateRight(255,255);
    leftOrright = 2;
  }
  else if (command == 4){
    back(255,255);
    leftOrright = 0;
  }
  else if (command == 5){
    grabber.write(85);
  }
  else if (command == 6){
    grabber.write(0);
  }
}

float positionByEncoder() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= samplingrate){
    rpmL = (float)(leftEnCount * (1000/samplingrate) * 60 / pulsePerRound);
    rpmR = (float)(rightEnCount * (1000/samplingrate) * 60 / pulsePerRound);

    vl = rpmL/60 * (2*r*3.1412);
    vr = rpmR/60 * (2*r*3.1412);
    // Serial.print(vl);
    // Serial.print("===");
    // Serial.println(vr);
    if (leftOrright == 1) {
      vl = -vl;
    }
    else if (leftOrright == 2) {
      vr = -vr;
    }

    leftEnCount = 0;
    rightEnCount = 0;

    x = x + (vl + vr) / 2 * cos(theta) * (samplingrate/1000);
    y = y + (vl + vr) / 2 * sin(theta)* (samplingrate/1000);
    theta = theta + (vr - vl) / l * (samplingrate/1000);
    previousMillis = millis();
  }
  return x,y,theta;
}

void sendXYZ(float x, float y, float theta) {
  // Send x, y, z as a formatted string "<x,y,z>"
  Serial.print("<");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(theta);
  Serial.println(">");
}

void  loop() {
  keyboard();
  x,y,theta = positionByEncoder();
  sendXYZ(x,y,theta);
}
