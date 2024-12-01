#include <Servo.h>

// Khai báo servo
Servo leftPincer;  // Servo điều khiển cánh tay kẹp bên trái

// Khai báo chân cảm biến siêu âm
const int trigPin = 42;  // Chân Trig của cảm biến siêu âm
const int echoPin = 40;  // Chân Echo của cảm biến siêu âm

// Tốc độ âm thanh (cm/us)
const float speedOfSound = 0.0343;

void setup() {
  Serial.begin(9600);

  // Gán servo vào chân điều khiển
  leftPincer.attach(6);  // Servo tay trái gắn vào chân 6

  // Khai báo chân cảm biến siêu âm
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Hiển thị thông báo khởi động
  Serial.println("Robot Initialized. Arm is open by default.");
  openPincer(); // Đặt trạng thái tay robot luôn mở lúc khởi động
}

void loop() {
  // Đo khoảng cách bằng cảm biến siêu âm
  float distance = measureDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Nếu phát hiện vật trong khoảng cách dưới 5cm
  if (distance <= 5.0) {
    Serial.println("Object detected within 5 cm!");

    // Đóng cánh tay kẹp để giữ vật
    closePincer();
    delay(10000); // Chờ 10 giây (thực hiện công việc)

    // Mở cánh tay kẹp trở lại
    openPincer();
    

    Serial.println("Pincer reset to open position.");
  } else {
    Serial.println("No object detected.");
  }

  delay(100); // Tránh việc đo liên tục quá nhanh
}

// Hàm đo khoảng cách bằng cảm biến siêu âm
float measureDistance() {
  // Phát xung Trig
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Nhận xung Echo
  long duration = pulseIn(echoPin, HIGH);

  // Tính khoảng cách (cm)
  float distance = duration * speedOfSound / 2;

  return distance;
}

// Hàm mở cánh tay kẹp
void openPincer() {
  leftPincer.write(90);  // Servo bên trái mở ra
  Serial.println("Pincer is open.");
}

// Hàm đóng cánh tay kẹp
void closePincer() {
  leftPincer.write(360);   // Xoay servo về vị trí giữa (đóng lại)
  Serial.println("Pincer is closed.");
}
