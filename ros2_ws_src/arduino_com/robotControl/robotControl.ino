#include <Wire.h>
#include <Servo.h>

int testComKeyboard;

int servoPin = 6;
Servo grabber;
int direction;

float linear_velocity = 0.0;
float angular_velocity = 0.0;
String inputString = "";   // String to hold incoming data
bool newData = false;

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

volatile long leftEnCount = 0;
volatile long rightEnCount = 0;
const int pulsePerRound = 2000;

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

// Interrupt service routines (ISRs) for encoder pulses
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
  Serial.begin(9600);
  Serial.println("Ready to receive velocities.");

  grabber.attach(servoPin); 
  
  // Setup interrupt 
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

int analogConverter(float v, float ratio){
  float rpm = (v*60)/(2*r*3.1412);
  float analog = rpm/maxrpm * 255;

  if (analog >= 255){
    analog = 150*ratio;
  }
  analog = constrain(analog, 0, 255);
  return analog;
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

float calculateSpeed(){
  rpmL = (float)(leftEnCount * (1000/samplingrate) * 60 / pulsePerRound);
  rpmR = (float)(rightEnCount * (1000/samplingrate) * 60 / pulsePerRound);

  vl = rpmL/60 * (2*r*3.1412);
  vr = rpmR/60 * (2*r*3.1412);

  leftEnCount = 0;
  rightEnCount = 0;

  return vl, vr;
}

float positionByEncoder(){
  currentMillis = millis();
  if (currentMillis - previousMillis >= samplingrate){
    vl, vr = calculateSpeed();
    x = x + (vl+vr)/2*cos(theta)*(samplingrate/1000);
    y = y + (vl+vr)/2*sin(theta)*(samplingrate/1000);
    theta = theta + (vr - vl)/l*(samplingrate/1000);
    previousMillis = currentMillis;
  }
  return x, y, theta;
}

void loop() {
  receiveData();
  if (newData) {
    processVelocities();
    x, y, theta =positionByEncoder();
    sendXYZ(x,y,theta);
    newData = false;
  }
}

void receiveData() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '>') {  // End of message
      newData = true;
    } else if (receivedChar == '<') {  // Start of message
      inputString = "";  // Clear the string to start a new message
    } else {
      inputString += receivedChar;
    }
  }
}

void processVelocities() {
  int commaIndex = inputString.indexOf(',');  // Find the comma separating the two values
  if (commaIndex > 0) {
    // Extract and convert substrings to floats
    linear_velocity = inputString.substring(0, commaIndex).toFloat();
    angular_velocity = inputString.substring(commaIndex + 1).toFloat();

    // Display the received values for debugging
    Serial.print("Linear Velocity: ");
    Serial.println(linear_velocity);
    Serial.print("Angular Velocity: ");
    Serial.println(angular_velocity);
  }
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