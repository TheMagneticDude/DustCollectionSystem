// When any tool sends an ON signal, the servo will go to the ON position,
// then return to neutral. If no ON signal is detected, only then will
// the servo go to the OFF position, then return to neutral.
//If the reciever is powered off and then turned back on when the servo has most recently
//been turned on, the board will go into recovery mode. The LED will blink for 60 seconds 
//(adjustable) and then turn the servo to the off state. If during the recovery period a tool 
//turns on, the reciever will exit the recovery period. 
//By Vika Vorona 
//Final version created 06/19/2025

#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

// === Servo Setup ===
Servo myServo;
const int servoPin = 16;   //pin for servo, must be PWM capable
const int closedAngle = 72;  //angle when servo turns button off
const int openAngle = 92;  //angle when servo turns button on
const int neutralAngle = 82; //angle returned to after opening/closing

// === LED Setup ===
const int ledPin = 21;  //pin connected to LED for status change
bool ledState = false;  //tracks current LED state
unsigned long lastBlinkTime = 0;

// === EEPROM Setup ===
#define EEPROM_SIZE 1  //one byte of EEPROM memory 
#define SERVO_STATE_ADDR 0  //address to store servo state 

// === ESP-NOW Setup ===
typedef struct struct_message {
  int id;  //id of board recieved 1 through 8
  int x;   //tool signal state (1=ON, 0=OFF)
  int y;
} struct_message;

struct_message myData;


unsigned long lastHeardTime = 0;        //last time an ON signal was received
const unsigned long ON_TIMEOUT = 1000; //time after last ON to trigger grace period

// === Grace Period Settings ===
const unsigned long GRACE_PERIOD_DURATION = 60000; //grace period after ON_TIMEOUT
bool gracePeriodActive = false;         //true if grace period countdown is happening
unsigned long gracePeriodStart = 0;     //when grace period started

unsigned long actionStartTime = 0;      //time when servo action started 
bool isMoving = false;                  //flag to track if servo is moving 
int targetPosition = neutralAngle;      //angle to return to after moving
bool recoveryMode = false;              //true if recovery mode is active after power up 
bool cancelRecoveryDueToActivity = false;  //set if recovery is interrupted by activity
unsigned long recoveryStartTime = 0;    //when recovery period started
bool toolActive = false;                // if we currently think a tool is running

// === EEPROM Helpers ===
void saveServoStateToEEPROM(bool isOpen) {
  EEPROM.write(SERVO_STATE_ADDR, isOpen ? 1 : 0);  //save 1 if open, 0 if closed
  EEPROM.commit();
}

bool readServoStateFromEEPROM() {
  return EEPROM.read(SERVO_STATE_ADDR) == 1;
}

// === Trigger Servo Movement ===
void triggerServoAction(int angle) {  //move servo to open/closed angle
  myServo.write(angle);
  targetPosition = neutralAngle;
  actionStartTime = millis();
  isMoving = true;

  //if moving to open position
  if (angle == openAngle) {      
    digitalWrite(ledPin, HIGH);
    ledState = true;
    saveServoStateToEEPROM(true);
  } else if (angle == closedAngle) {
    digitalWrite(ledPin, LOW);
    ledState = false;
    saveServoStateToEEPROM(false);
  }
}

// === Callback for Received ESP-NOW Data ===
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  if (myData.id >= 1 && myData.id <= 8) {
    if (myData.x == 1 || myData.y == 1) {
      Serial.printf("Tool %d ON signal received\n", myData.id);

      lastHeardTime = millis();  //reset timeout

      //if an ON signal is heard during the grace period, cancel it
      if (gracePeriodActive) {
        Serial.println("→ ON signal during grace period → Cancelling grace period");
        gracePeriodActive = false;
      }

      //if tool wasn't already active
      if (!toolActive) {
        triggerServoAction(openAngle);
        Serial.println("→ Tool activity started → Open + LED ON");
        toolActive = true;
      }

      //if recovering and activity 
      if (recoveryMode) {
        Serial.println("→ Activity detected during recovery → Cancelling recovery");
        recoveryMode = false;
        cancelRecoveryDueToActivity = true;
        digitalWrite(ledPin, HIGH);  //return LED to solid
        ledState = true;
      }
    }
  } else {
    Serial.println("Error: Board ID out of range!");
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  ledState = false;

  EEPROM.begin(EEPROM_SIZE);

  myServo.attach(servoPin, 500, 2500);  //attach servo to PWM capable pin 
  myServo.write(neutralAngle);   //start in neutral position 
  delay(100);

  WiFi.mode(WIFI_STA);         //set wifi to station mode
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));   //register recieve callback DONT CHANGE THIS LINE

  //check last saved servo state in EEPROM
  if (readServoStateFromEEPROM()) {
    Serial.println("Recovered: Last state was OPEN, entering recovery delay...");
    recoveryMode = true;
    cancelRecoveryDueToActivity = false;
    recoveryStartTime = millis();   //recovery timer
    toolActive = true;
    digitalWrite(ledPin, HIGH);
    ledState = true;
  } else {
    Serial.println("Recovered: Last state was CLOSED");
    digitalWrite(ledPin, LOW);
    ledState = false;
  }
}

// === Loop ===
void loop() {
  unsigned long now = millis();

  // === Return servo to neutral after move ===
  if (isMoving && now - actionStartTime >= 2000) {
    myServo.write(neutralAngle);
    Serial.println("→ Returned to neutral (90°)");
    isMoving = false;
  }

  // === Recovery LED Blinking ===
  if (recoveryMode) {
    if (now - lastBlinkTime >= 500) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState ? HIGH : LOW);
      lastBlinkTime = now;
    }

    if (now - recoveryStartTime >= 60000) {    //after 60 seconds of recovery time 
      if (!cancelRecoveryDueToActivity) {
        Serial.println("→ Recovery timer expired with no activity → Closing");
        triggerServoAction(closedAngle);
        toolActive = false;
      } else {
        Serial.println("→ Recovery expired but activity was detected → No action");
      }
      recoveryMode = false;     //exit recovery mode 
      cancelRecoveryDueToActivity = false;
    }
  }

  // === Grace period logic ===
  if (toolActive && !recoveryMode) {
    //start grace period after ON_TIMEOUT of silence
    if (!gracePeriodActive && now - lastHeardTime >= ON_TIMEOUT) {
      Serial.println("→ No ON signal recently → Starting grace period");
      gracePeriodActive = true;
      gracePeriodStart = now;
    }

    //if grace period active and expired, close servo
    if (gracePeriodActive && now - gracePeriodStart >= GRACE_PERIOD_DURATION) {
      Serial.println("→ Grace period expired → Closing servo");
      triggerServoAction(closedAngle);
      toolActive = false;
      gracePeriodActive = false;
    }
  }
}
