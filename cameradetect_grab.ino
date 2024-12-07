#include <Servo.h>

// Define motor control pins
int enL = 4;    // Enable pin for left motor
int inL1 = 9;   // Input 1 for left motor
int inL2 = 8;   // Input 2 for left motor
int enR = 5;    // Enable pin for right motor
int inR1 = 11;  // Input 1 for right motor
int inR2 = 10;  // Input 2 for right motor

// Define pins for ultrasonic sensor
const int trigPin = 42;  // Trig pin for ultrasonic sensor
const int echoPin = 40;  // Echo pin for ultrasonic sensor

// Define pin for servo (gripper)
Servo gripper;
const int gripperPin = 6;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize motor control pins as output
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach servo to gripper pin
  gripper.attach(gripperPin);
  gripper.write(45);  // Initialize gripper to open position
}

long getDistance() {
  // Send ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the time it takes for the echo to return
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  long distance = duration * 0.034 / 2;
  return distance;
}

void moveForward() {
  analogWrite(enL, 100);  // Adjusted speed to 50% of original
  analogWrite(enR, 100);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnLeft() {
  analogWrite(enL, 50);   // Reduced left motor speed
  analogWrite(enR, 100);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void turnRight() {
  analogWrite(enL, 100);
  analogWrite(enR, 50);   // Reduced right motor speed
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, HIGH);
  digitalWrite(inR2, LOW);
}

void stopMotors() {
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'F') {
      moveForward();
    } else if (command == 'L') {
      turnLeft();
    } else if (command == 'R') {
      turnRight();
    } else if (command == 'S') {
      stopMotors();
    } else if (command == 'D') {
      long distance = getDistance();
      Serial.println(distance);  // Send distance as a clean string
    } else if (command == 'G') {
      gripper.write(0);  // Close gripper
      delay(1000);
      gripper.write(180); // Open gripper after gripping (if required)
    }
  }
}
