// Mega2560
// external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
// pin                  2         3      21      20      19      18

#include <TimerOne.h>

#define PI 3.1415926535897932384626433832795
float d = 0.185; // wheel distance
float r = 0.0225; // wheel radius
float samplingTime = 0.1; //0.5;  // sampling time
const int ENCODER_RESOLUTION = 2100;
float M_PER_REV = 2*PI*r;

// Left Motor
int enL = 4;
int inL1 = 9;
int inL2 = 8;

// Right motor
int enR = 5;
int inR1 = 11;
int inR2 = 10;

// For encoder
int enLA = 2;
int enLB = 3;

int enRA = 18;
int enRB = 19;

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
float vR, vL;

// PID constants
float KpL = 450;  // Proportional gain
float KiL = 490;  // Integral gain
float KdL = 00;  // Derivative gain
// PID constants
float KpR = 200;  // Proportional gain
float KiR = 400;  // Integral gain
float KdR = 0;  // Derivative gain

float set_vL = 0, set_vR = 0;
float err_vL = 0, err_vR = 0, pre_err_vL = 0, pre_err_vR = 0;
float integralL = 0, integralR = 0, derivativeR = 0, derivativeL = 0;
float controlOutputL = 0, controlOutputR = 0;

// Flag to track if data is received
bool dataReceived = false;

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
	
	// Turn off motors - Initial state
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);

  // Initialize TimerOne to call the toggleLED function every 1 second (1000000 microseconds)
  // Note: This timer will use pin 12 so do not use it for other function 
  Timer1.initialize(1000000*samplingTime); // 1,000,000 microseconds = 1 second
  Timer1.attachInterrupt(VelCtrlTimer); // Attach the interrupt to the calculate velocity 
  while (!Serial) { ; // wait for serial port to connect. Needed for native USB port only }
  }
}

void loop() {
//  positionControl(1,1,0);
//    goForward(100);
//  set_speedL_PID(70);
//  delay(10000);
    // Serial.print("rightEnCount: ");
    // Serial.println(rightEnCount);
    //  Serial.print("leftEnCount: ");
    // Serial.println(leftEnCount);
    // delay(200);

//

// Check if data is available on the serial port 
// if (Serial.available() > 0) {
//   // Read the incoming data
//   set_vL = Serial.parseFloat();
//   set_vR = Serial.parseFloat();
//   // Clear the buffer
//   // Serial.read(); // To consume the newline character
 
// }
if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read the incoming string until newline
    dataReceived = true; // Mark data as received
    
    // Split the input into vL and vR
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      String vL_str = input.substring(0, spaceIndex);
      String vR_str = input.substring(spaceIndex + 1);

      // Convert strings to float
      set_vL = vL_str.toFloat();
      set_vR = vR_str.toFloat();

      // delay(3000);

      // Print received values
      Serial.print("Received vL: ");
      Serial.println(vL);
      Serial.print("Received vR: ");
      Serial.println(vR);
    }
  
  }

//  else if (!dataReceived) {
//     // If no new data is received, reset velocities to 0
//     set_vL = 0;
//     set_vR = 0;
//     dataReceived = false; // Reset the flag

//     Serial.println("No signal detected: vL and vR set to 0");

//  }



  
// Print velocities to Serial Monitor
//  Serial.println("Set to 0.1");
//  set_vL = 0.1;
//   set_vR = 0.1;
//    delay(3000);
//     Serial.println("Set to 0.1");
//  set_vL = 0.14;
//   set_vR = 0.06;
//    delay(3000);
//   Serial.println("Set to -0.1");
//   set_vL = 0.06;
//   set_vR = 0.14;
//   delay(3000);
//    Serial.println("Set to 0.1");
//  set_vL = 0.06;
//   set_vR = 0.10;
//    delay(3000);

}


void VelCtrlTimer() {
  // Calculate wheel velocities in m/s
  // v = (leftEncoderCount/ENCODER_RESOLUTION)  * M_PER_REV / samplingTime
  vR = (float(rightEnCount)/ENCODER_RESOLUTION)*M_PER_REV/samplingTime;
  vL = (float(leftEnCount)/ENCODER_RESOLUTION)*M_PER_REV/samplingTime;

  // Reset encoder counts for the next calculation
  leftEnCount = 0;
  rightEnCount = 0;

/*
  if (set_vL < 0) {
    vL = -vL;
  }

  if (set_vR < 0) {
    vR = -vR;
  }
*/

  // PID calculations
  err_vL = set_vL - vL;
  err_vR = set_vR - vR;
  
  integralR += err_vR * samplingTime;
  derivativeR = (err_vR - pre_err_vR) / samplingTime;
  controlOutputR = KpR * err_vR + KiR * integralR + KdR * derivativeR;
  pre_err_vR = err_vR;

  integralL += err_vL * samplingTime;
  derivativeL = (err_vL - pre_err_vL) / samplingTime;
  controlOutputL = KpL * err_vL + KiL * integralL + KdL * derivativeL;
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

  Serial.print("vL: ");
  Serial.print(vL);
  Serial.println(" m/s");

  Serial.print("vR: ");
  Serial.print(vR);
  Serial.println(" m/s");

/*
  Serial.print("P: ");
  Serial.print(Kp * err_vL);
  Serial.print(" - I: ");
  Serial.print(Ki * integralL);
  Serial.print(" - D: ");
  Serial.print(Kd * derivativeL);
  */
  Serial.print(" - All_L: ");
  Serial.print((int)controlOutputL);
  Serial.print(" - vL: ");
  Serial.print(vL);
  Serial.print("        - All_R: ");
  Serial.print((int)controlOutputR);
  Serial.print(" - vR: ");
  Serial.println(vR);


}

void setMotorSpeedL(int speed) {

  // Stop
  if (speed == 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);   
  }

  // set motor direction
  if (speed > 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
  }
  
  if (speed < 0) {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  }
  
  // Set motor speed
  speed = abs(speed);

  if (speed > 255) {
    speed = 255;
  }
  if (speed < 20) {
    speed = 20;
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
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  }
  
  if (speed < 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
  }
  
  // Set motor speed
  speed = abs(speed);
  if (speed > 255) {
    speed = 255;
  }
  if (speed < 20) {
    speed = 20;
  }

  analogWrite(enR, speed);
}


void goForward(int speed) {
  // Reset encoder counter
 // rightEnCount = 0;
//  leftEnCount = 0;

	// For PWM maximum possible values are 0 to 255
	analogWrite(enR, speed);

//  int motor_L_speed = speed + K*(rightEnCount-leftEnCount);  
//  analogWrite(enL, motor_L_speed);
  analogWrite(enL, speed);
	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);
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
