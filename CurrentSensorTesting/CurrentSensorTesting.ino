#include <ESP32Servo.h>

//====Servo Settings====
Servo myServo;
const int SERVO_PIN = 4;  //pin for servo, must be PWM capable
const int CLOSED_ANGLE = 0;   //angle to close servo
const int OPEN_ANGLE = 60;    //angle of open servo

//====Current Sensor Settings====
const int AC_SENSOR_PIN = 32;   //analog pin to read current sensor
const float ON_THRESHOLD  = 12.0;   // tune experimentally
const float OFF_THRESHOLD = 8.0;    // lower than ON threshold

bool toolOn = false;

float readActivity()
{
  const int N = 300;
  long sum = 0;

  // First pass: estimate baseline
  for (int i = 0; i < N; i++) {
    sum += analogRead(AC_SENSOR_PIN);
  }
  float mean = (float)sum / N;

  // Second pass: average absolute deviation from baseline
  float activity = 0;
  for (int i = 0; i < N; i++) {
    activity += fabs(analogRead(AC_SENSOR_PIN) - mean);
  }

  return activity / N;
}

void updateToolStatus() {
  float activity = readActivity();

  if (!toolOn && activity > ON_THRESHOLD) {
    toolOn = true;
  } else if (toolOn && activity < OFF_THRESHOLD) {
    toolOn = false;
  }

  Serial.print("activity = ");
  Serial.print(activity);
  Serial.print("   state = ");
  Serial.println(toolOn ? "ON" : "OFF");
}

//====Setup====
void setup() {
  Serial.begin(115200);  //serial monitor for debugging of course

  myServo.attach(SERVO_PIN, 1000, 2000);  //attaching servo and its pulse limits
  myServo.write(CLOSED_ANGLE);     //set servo to closed position 
}

//====Main Loop====
void loop() {
  updateToolStatus();

  if (toolOn) {
    myServo.write(OPEN_ANGLE);
  } else {
    myServo.write(CLOSED_ANGLE);
  }

  // Once adding sending code, only send tool state if it is on, and message has not been sent withing the last 500 ms or the last message failed to send
  // This makes there not need to be off messages sent, and reciver handles timeouts for tools

  delay(10);
}

