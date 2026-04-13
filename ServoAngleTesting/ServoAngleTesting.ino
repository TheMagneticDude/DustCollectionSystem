#include <ESP32Servo.h>


// === Servo Setup ===
Servo myServo;

// === Button & State ===
const int buttonPin = 16;    //pin for input button
bool toolState = false;      //tracks current state of tool 
int lastButtonState = HIGH;

int val = 0;

void setup() {
  Serial.begin(115200);  //for debugging ofc

  // === Setup Servo ===
  myServo.attach(16, 1000, 2000);  //servo connected to pin 4, but be PWM capable 
  myServo.write(0);  //start closed
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();

    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      Serial.print("Angle set to: ");
      Serial.println(angle);
    }

    while (Serial.available() > 0) {
      Serial.read();
    }
  }
  
  /*
  myServo.write(val);
  val++;
  if (val > 180) {
    val = 0;
  }
  delay(50);
  */
}
