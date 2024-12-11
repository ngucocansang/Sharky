#include <Arduino.h>

// Define motor control pins
int enL = 4;    // Enable pin for left motor
int inL1 = 9;   // Input 1 for left motor
int inL2 = 8;   // Input 2 for left motor
int enR = 5;    // Enable pin for right motor
int inR1 = 11;  // Input 1 for right motor
int inR2 = 10;  // Input 2 for right motor

// Constants
#define MAX_VELOCITY 1.0    // Maximum wheel velocity in m/s
#define PWM_RESOLUTION 55  // Maximum PWM value

// Robot-specific parameters
float wheel_radius = 0.05;  // Wheel radius in meters
float max_rpm = 70.0;      // Max motor speed in RPM
float max_pwm = 55.0;      // Max PWM value

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud
  Serial.println("Arduino Ready");

  // Motor pins setup
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
}

void loop() {
  static String received_data = "";
  
  // Check if data is available on the serial port
  while (Serial.available() > 0) {
    char incoming_char = Serial.read();
    if (incoming_char == '\n') {
      // Parse the received velocities
      handleCommand(received_data);
      received_data = ""; // Reset the string
    } else {
      received_data += incoming_char; // Append character to string
    }
  }
}

void handleCommand(String data) {
  float v_left = 0.0, v_right = 0.0;

  // Parse the input string
  int commaIndex = data.indexOf(',');
  if (commaIndex > 0) {
    v_left = data.substring(0, commaIndex).toFloat();
    v_right = data.substring(commaIndex + 1).toFloat();
    
    // Debug print
    Serial.print("Received v_left: ");
    Serial.print(v_left);
    Serial.print(" m/s, v_right: ");
    Serial.println(v_right);
    
    // Control motors
    controlMotors(v_left, v_right);
  } else {
    Serial.println("Invalid data format!");
  }
}

void controlMotors(float v_left, float v_right) {
  // Convert velocities to PWM values
  int pwm_left = velocityToPWM(v_left);
  int pwm_right = velocityToPWM(v_right);

  // Control left motor
  if (v_left >= 0) {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  } else {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
  }
  analogWrite(enL, abs(pwm_left));

  // Control right motor
  if (v_right >= 0) {
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  } else {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
  }
  analogWrite(enR, abs(pwm_right));
}

int velocityToPWM(float velocity) {
  // Map velocity to a PWM value
  float rpm = (velocity / (2 * PI * wheel_radius)) * 60.0; // Convert m/s to RPM
  int pwm = (rpm / max_rpm) * max_pwm;
  pwm = constrain(pwm, 0, PWM_RESOLUTION); // Ensure PWM is within 0-255
  return pwm;
}
