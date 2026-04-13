#include <ESP32Servo.h>


bool dustCollectorState = false;

Servo myServo;

const int buttonPin = 13;    // pin for input button


void turnDustCollectorOff() {
  myServo.write(0);  
  delay(300);
  myServo.write(90);  
}

void turnDustCollectorOn() {
  myServo.write(180);  
  delay(300);
  myServo.write(90);  
}


void setup() {
  Serial.begin(115200);  //for debugging ofc

  pinMode(buttonPin, INPUT_PULLUP); 

  // === Setup Servo ===
  myServo.attach(16, 1000, 2000);  
  myServo.write(90);  
}

void loop() {
  // Read the state of the button
  int buttonState = digitalRead(buttonPin); // a value of high means button is not pressed

  if (buttonState == LOW) {
    if (dustCollectorState) {
      turnDustCollectorOff();
      dustCollectorState = false;
    } else {
      turnDustCollectorOn();
      dustCollectorState = true;
    }
  } 

  delay(10);
}
