// Mega2560
// external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
// pin                  2         3      21      20      19      18

#include <TimerOne.h>
#include <HCSR04.h>
#include <Wire.h>
#include <Servo.h>

HCSR04 hc(34,35);
int servoPin = 25;
Servo grabber;

#define PI 3.1415926535897932384626433832795
float d = 0.172; // wheel distance
float r = 0.0225; // wheel radius
float samplingTime = 0.1; //0.5;  // sampling time
const int ENCODER_RESOLUTION = 2100;
float M_PER_REV = 2*PI*r;

// Left Motor
int enL = 5;
int inL1 = 6;
int inL2 = 7;

// Right motor
int enR = 10;
int inR1 = 8;
int inR2 = 9;

// For encoder
int enLA = 2;
int enLB = 3;

int enRA = 20;
int enRB = 21;

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
float vR, vL;

// PID constants
float Kp = 500;  // Proportional gain
float Ki = 700;  // Integral gain
float Kd = 100;  // Derivative gain
float set_vL = 0, set_vR = 0;
float err_vL = 0, err_vR = 0, pre_err_vL = 0, pre_err_vR = 0;
float integralL = 0, integralR = 0, derivativeR = 0, derivativeL = 0;
float controlOutputL = 0, controlOutputR = 0;

float ultra = 0.0;

int grab = 0;
bool grabbed;
int status = 0;

void setup()
{
  Serial.begin(9600);

  // Setup interrupt 
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);

  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all the motor control pins to outputs
	pinMode(enR, OUTPUT);
	pinMode(enL, OUTPUT);
	pinMode(inR1, OUTPUT);
	pinMode(inR2, OUTPUT);
	pinMode(inL1, OUTPUT);
	pinMode(inL2, OUTPUT);

  grabber.attach(servoPin); 
	
	// Turn off motors - Initial state
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);

  // Initialize TimerOne to call the toggleLED function every 1 second (1000000 microseconds)
  // Note: This timer will use pin 12 so do not use it for other function 
  Timer1.initialize(1000000*samplingTime); // 1,000,000 microseconds = 1 second
  Timer1.attachInterrupt(VelCtrlTimer); // Attach the interrupt to the calculate velocity 
  grabber.write(160);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read data until newline
    data.trim(); // Remove any leading/trailing whitespace

    int firstSeparatorIndex = data.indexOf(' '); // Find the first space
    int secondSeparatorIndex = data.indexOf(' ', firstSeparatorIndex + 1); // Find the second space

    if (firstSeparatorIndex != -1 && secondSeparatorIndex != -1) {
      // Parse the three values from the input string
      set_vL = data.substring(0, firstSeparatorIndex).toFloat();
      set_vR = data.substring(firstSeparatorIndex + 1, secondSeparatorIndex).toFloat();
      grab = data.substring(secondSeparatorIndex + 1).toInt();
    }
  }
}

void VelCtrlTimer() {
  ultra = hc.dist();

  Serial.println(status);

  if (ultra > 5){
    status = 0;
    grabber.write(0);
  }
  else {
    if (grab == 0){
      if (grabbed == true){
        status = 0;
        grabber.write(0);
        delay(2000);
      }
    }
    else {
      if (grabbed == false){
        delay(400);
        grabber.write(70);
        status = 1;
        grabbed = true;
      }
      else {
        status = 1;
        grabber.write(70);
      }
    }
  }

  // Calculate wheel velocities in m/s
  // v = (leftEncoderCount/ENCODER_RESOLUTION)  * M_PER_REV / samplingTime
  vR = (float(rightEnCount)/(ENCODER_RESOLUTION+0))*M_PER_REV/samplingTime;
  vL = (float(leftEnCount)/(ENCODER_RESOLUTION+0))*M_PER_REV/samplingTime;

  // Reset encoder counts for the next calculation
  leftEnCount = 0;
  rightEnCount = 0;

  // PID calculations
  err_vL = set_vL - vL;
  err_vR = set_vR - vR;
  
  integralR += err_vR * samplingTime;
  derivativeR = (err_vR - pre_err_vR) / samplingTime;
  controlOutputR = Kp * err_vR + Ki * integralR + Kd * derivativeR;
  pre_err_vR = err_vR;

  integralL += err_vL * samplingTime;
  derivativeL = (err_vL - pre_err_vL) / samplingTime;
  controlOutputL = Kp * err_vL + Ki * integralL + Kd * derivativeL;
  pre_err_vL = err_vL;

  // Set the speed = 0 if the set value = 0
  if (set_vL == 0) {
    controlOutputL = 0;
  }

  if (set_vR == 0) {
    controlOutputR = 0;
  }

  setMotorSpeedR((int)controlOutputR);
  setMotorSpeedL((int)controlOutputL);

/*
  Serial.print("P: ");
  Serial.print(Kp * err_vL);
  Serial.print(" - I: ");
  Serial.print(Ki * integralL);
  Serial.print(" - D: ");
  Serial.print(Kd * derivativeL);
  Serial.print(" - All: ");
  Serial.print((int)controlOutputL);
  Serial.print(" - vL: ");
  Serial.println(vL);
*/

}

void setMotorSpeedL(int speed) {

  // Stop
  if (speed == 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);   
  }

  // set motor direction
  if (speed > 0) {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  }
  
  if (speed < 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
  }
  
  // Set motor speed
  speed = abs(speed);

  if (speed > 255) {
    speed = 255;
  }
  if (speed < 40) {
    speed = 40;
  }
  
  analogWrite(enL, speed);
}

void setMotorSpeedR(int speed) {

  // Stop
  if (speed == 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);   
  }

  // set motor direction
  if (speed > 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
  }
  
  if (speed < 0) {
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  }
  
  // Set motor speed
  speed = abs(speed);
  if (speed > 255) {
    speed = 255;
  }
  if (speed < 40) {
    speed = 40;
  }

  analogWrite(enR, speed);
}

void stop() {
	// Turn off motors 
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);
}

void leftEnISRA() {
  if (digitalRead(enLB) == HIGH) {
    leftEnCount++;
  } else {
    leftEnCount--;
  }
}

void leftEnISRB() {
  if (digitalRead(enLA) == HIGH) {
    leftEnCount--;
  } else {
    leftEnCount++;
  }
}

void rightEnISRA() {
  if (digitalRead(enRB) == HIGH) {
    rightEnCount--;
  } else {
    rightEnCount++;
  }
}

void rightEnISRB() {
  if (digitalRead(enRA) == HIGH) {
    rightEnCount++;
  } else {
    rightEnCount--;
  }
}