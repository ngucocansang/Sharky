#include <math.h>
#include "ArduPID.h"

// Define motor control pins
int enL = 4;
int inL1 = 9;
int inL2 = 8;
int enR = 5;
int inR1 = 11;
int inR2 = 10;

// Define encoder pins
int enLA = 2;
int enLB = 3;
int enRA = 18;
int enRB = 19;

// Variables to count encoder pulses
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;

// Sign to indicate robot moved forward (1) or backward (-1)
int signL = 1;
int signR = 1;

// Time variables
unsigned long prevTime = 0;
const unsigned int interval = 0.5; // 0.5 second
float dt = 0;
const float d = 45; // wheel diameter
const float l = 186; // distance between two wheels
const int PPR = 2100; // Define number of pulses per revolution
const int mintorqueL = 20;
const int mintorqueR = 20;
const int compensator = 15;

// Define line sensor pins (assuming 5 channels)
float previousError = 0;  // Store the previous error
bool isRight = 0, isLeft = 0;
int sensorLeft2 = 44; // Leftmost sensor
int sensorLeft1 = 46;
int sensorMiddle = 48; // Middle sensor
int sensorRight1 = 50;
int sensorRight2 = 52; // Rightmost sensor

// Ultrasonic sensor pins
int trigFront = 24;  // Trig pin for front ultrasonic sensor
int echoFront = 22;  // Echo pin for front ultrasonic sensor
int trigLeft = 28;   // Trig pin for left ultrasonic sensor
int echoLeft = 26;   // Echo pin for left ultrasonic sensor
int trigRight = 32;  // Trig pin for right ultrasonic sensor
int echoRight = 30;  // Echo pin for right ultrasonic sensor

// Safe distance
int safeDistance = 18;

// Speed constants
int normalSpeed = 50;   // Normal speed when following the line correctly
int slowSpeed = 10;     // Slow speed for corners or curves
int minSpeed = 10, maxSpeed = 50; // Minimum and maximum motor speeds

void setup() {
  Serial.begin(9600);
  // Setup interrupt for encoders
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set motor control pins as outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  // Set ultrasonic sensor pins as outputs and inputs
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // Set line sensor pins as inputs
  pinMode(sensorLeft2, INPUT);
  pinMode(sensorLeft1, INPUT);
  pinMode(sensorMiddle, INPUT);
  pinMode(sensorRight1, INPUT);
  pinMode(sensorRight2, INPUT);
}

void loop() {
  // Check for obstacles using ultrasonic sensors
  long distanceFront = getDistance(trigFront, echoFront);
  int left2 = digitalRead(sensorLeft2);
  int left1 = digitalRead(sensorLeft1);
  int middle = digitalRead(sensorMiddle);
  int right1 = digitalRead(sensorRight1);
  int right2 = digitalRead(sensorRight2);
  int sensorCount = left2 + left1 + middle + right1 + right2;
  if (distanceFront > safeDistance && sensorCount < 5) {
    line(); // Follow the line
  } else if (sensorCount == 5){

    move(0,0);
    delay(1000);       
    move(80,80);
    delay(2900);
    turn(50,50);
    delay(900);
    move(200,200);  
    delay(5500); 
    // PID(1810, 1, 1, 1, false, true);
    move(0,0);    
    delay(10000);
  } 
  else {
    avoidObstacle(); // Perform obstacle avoidance
  }
  

}

void avoidObstacle() {
  long distanceLeft = getDistance(trigLeft, echoLeft);
  long distanceRight = getDistance(trigRight, echoRight);

  // Choose to turn right if the left side is more obstructed
  if (distanceLeft < distanceRight || distanceLeft > distanceRight) {
    // PID(-PI / 2, 80, 50, 3.3, true, false);
    turnR(50,50);
    delay(800);
    move(0,0);
    delay(500);
    while (true) {
      distanceLeft = getDistance(trigLeft, echoLeft);
      isRight = 0;
      isLeft = 1;
      wall_distance(distanceLeft);

      // Check if the line is detected again
      if (lineDetected()) {
        move(0,0);
        delay(200); // Small delay to stabilize
        return; // Exit obstacle handling
      }
    }
  }
  // Otherwise, turn left
  // else {
  //   PID(PI / 2, 80, 50, 3.3, true, false);
  //   while (true) {
  //     distanceRight = getDistance(trigRight, echoRight);
  //     isRight = 1;
  //     isLeft = 0;
  //     wall_distance(distanceRight);

  //     // Check if the line is detected again
  //     if (lineDetected()) {
  //       move(0,0);
  //       delay(200); // Small delay to stabilize
  //       return; // Exit obstacle handling
  //     }
  //   }
  // }
}

void wall_distance(float distance) {
  // PID control to maintain a safe distance from the wall
  float Kp = 4.0;
  float setPoint = safeDistance;
  int in_min = 0;
  int in_max = setPoint * Kp;
  int out_min = 0;
  int out_max = maxSpeed - minSpeed;

  // Variables for PID control
  float error = 0;
  float lastError = 0;
  float integral = 0;
  float derivative = 0;
  float motorSpeedCorrection = 0;

  // Calculate PID control
  distance = constrain(distance, 0, setPoint * 2);
  error = setPoint - distance;       // Proportional term
  integral += error;                 // Integral term
  derivative = error - lastError;    // Derivative term
  motorSpeedCorrection = (Kp * error);

  // Ensure correction value stays within motor speed limits
  motorSpeedCorrection = map(motorSpeedCorrection, -in_max, in_max, -out_max, out_max);

  // Control the motors to maintain distance
  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;

  if (isRight == 1 && isLeft == 0) {
    leftMotorSpeed = slowSpeed - motorSpeedCorrection;   // Adjust left motor speed
    rightMotorSpeed = slowSpeed + motorSpeedCorrection;  // Adjust right motor speed
  } else if (isRight == 0 && isLeft == 1) {
    leftMotorSpeed = slowSpeed + motorSpeedCorrection;   // Adjust left motor speed
    rightMotorSpeed = slowSpeed - motorSpeedCorrection;  // Adjust right motor speed
  } else {
    return;
  }

  leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, normalSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, normalSpeed);

  move(leftMotorSpeed, rightMotorSpeed);

  // Update last error for the next cycle
  lastError = error;
}

bool lineDetected() {
  int left1 = digitalRead(sensorLeft1);
  int middle = digitalRead(sensorMiddle);
  int right1 = digitalRead(sensorRight1);

  // Return true if any of the middle line sensors detect the line
  return (middle == 1 || left1 == 1 || right1 == 1);
}

void line() {
  int left2 = digitalRead(sensorLeft2);
  int left1 = digitalRead(sensorLeft1);
  int middle = digitalRead(sensorMiddle);
  int right1 = digitalRead(sensorRight1);
  int right2 = digitalRead(sensorRight2);

  int sensorCount = left2 + left1 + middle + right1 + right2;

  if (sensorCount >= 5) {
    move(0, 0);
    Serial.println("Stopping: Line detected by 3 or more sensors.");
    return;
  }

  float error = calculateError(left2, left1, middle, right1, right2);
  float correction = 50 * error;

  previousError = error;

  int leftSpeed = normalSpeed + correction;
  int rightSpeed = normalSpeed - correction;

  if (left1 == 1 || left2 == 1 || right1 == 1 || right2 == 1) {
    leftSpeed = slowSpeed + correction;
    rightSpeed = slowSpeed - correction;
  }

  leftSpeed = constrain(leftSpeed, minSpeed, normalSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, normalSpeed);

  move(leftSpeed, rightSpeed);
}

float calculateError(int left2, int left1, int middle, int right1, int right2) {
  int weights[5] = {-2, -1, 0, 1, 2};
  int sensorValues[5] = {left2, left1, middle, right1, right2};
  float weightedSum = 0;
  int sum = 0;

  for (int i = 0; i < 5; i++) {
    weightedSum += sensorValues[i] * weights[i];
    sum += sensorValues[i];
  }

  if (sum == 0) {
    return previousError;
  }

  return weightedSum / sum;
}

void PID(double setvar, double Kp, double Ki, double Kd, bool angle, bool distance) {
  ArduPID controller;
  double var = 0;
  double outvar = 0;
  controller.begin(&var, &outvar, &setvar, Kp, Ki, Kd);
  controller.setOutputLimits(-200, 200);
  controller.setBias(0);

  while (abs(100 - (100 * var) / setvar) > 10) {
    unsigned long currentTime = millis();
    dt = currentTime - prevTime;

    if (dt >= interval) {
      noInterrupts();
      int leftPulseCount = leftEnCount;
      int rightPulseCount = rightEnCount;
      leftEnCount = 0;
      rightEnCount = 0;
      interrupts();

      double deltaL = signL * (PI * d * leftPulseCount) / PPR;
      double deltaR = signR * (PI * d * rightPulseCount) / PPR;

      if (angle) {
        var = var + 2 * (deltaR - deltaL) / l;
        controller.compute();
        move(-outvar, outvar);
      }
      if (distance) {
        var = var + (deltaR + deltaL) / 2;
        controller.compute();
        move(outvar, outvar);
      }

      controller.debug(&Serial, "var", PRINT_INPUT | PRINT_OUTPUT | PRINT_SETPOINT |
                                 PRINT_BIAS | PRINT_P | PRINT_I | PRINT_D);

      prevTime = currentTime;
    }
  }
  move(0, 0);
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

void move(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    signL = 1;
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
  } else {
    signL = -1;
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  }

  if (rightSpeed >= 0) {
    signR = 1;
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  } else {
    signR = -1;
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
  }

  if (leftSpeed == 0 && rightSpeed == 0) {
    analogWrite(enR, 0);
    analogWrite(enL, 0);
  } else {
    analogWrite(enR, abs(rightSpeed) + mintorqueR);
    analogWrite(enL, abs(leftSpeed) + mintorqueL + compensator);
  }
}

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

void turn(int leftSpeed, int rightSpeed) {
  // For PWM, the maximum possible value is 0 to 255
  analogWrite(enR, rightSpeed);
  analogWrite(enL, leftSpeed);

  // Set motors to move forward
  digitalWrite(inL1, HIGH);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnR(int leftSpeed, int rightSpeed) {
  // For PWM, the maximum possible value is 0 to 255
  analogWrite(enR, rightSpeed);
  analogWrite(enL, leftSpeed);

  // Set motors to move forward
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}