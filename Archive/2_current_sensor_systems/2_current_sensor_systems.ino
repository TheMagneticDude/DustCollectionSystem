//This code is for the two current sensor systems part of the automated dust collection system
//These are referred to as systems 1, 5, and 8.
//When a tool is turned on, the servo turns on and the
//ON signal is sent to the reciever ESP32 at random intervals
//as long as the tool is on. When the tool is turned off, 
//the ON signal is no longer sent to the reciever,
//the grace period begins, and at its end the servo closes.
//The current sensor value code was adapted from the code given by DFRobot
//for the SEN0211. 
//By Vika Vorona 
//Final version created 06/19/2025
//vorona.1@osu.edu

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// ==== Servo Settings ====
Servo myServo;
const int servoPin = 4;   //pin for servo, must be PWM capable 
const int closedAngle = 0;  //closed angle for servo
const int openAngle = 60;  //open angle for servo

// ==== Current Sensor Settings ====
const int ACPin1 = 32;  //first current sensor
const int ACPin2 = 33;  //second current sensor
#define ACTectionRange 20
#define VREF 3.30
#define CURRENT_THRESHOLD 0.05  //min current to detect tool usage

// ==== Grace Period (adjustable global) ====
const unsigned long GRACE_PERIOD_MS = 60000;  

// ==== Startup delay ====
const unsigned long STARTUP_DELAY_MS = 2000; //2 seconds startup delay
unsigned long startupTime = 0;
bool startupComplete = false;

// ==== ESP-NOW Settings ====
uint8_t broadcastAddress[] = {0x6C, 0xC8, 0x40, 0x35, 0x0B, 0x60};  //mac address of reciever board

typedef struct struct_message {
  int id;  //device id
  int x;  // 1 = ON, 0 = OFF
  int y;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// ==== Tool + Timer States ====
bool toolOn = false;   //tracks if tool is currently running
unsigned long lastOffTime = 0;   //time when tool is turned off
bool gracePeriodActive = false;

// ==== Retry Logic ====
const int MAX_RETRIES = 3;
int retryCount = 0;
bool lastSendSuccess = true; //track last send status

// ==== ON Jitter Logic ====
unsigned long lastOnSend = 0;
unsigned long nextSendInterval = 0;  //random jitter between ON signals

// ==== Setup ====
void setup() {
  Serial.begin(115200);  
  WiFi.mode(WIFI_STA); //set wifi to station mode

  // Attach servo
  myServo.attach(servoPin, 500, 2500);  //attach servo and set pulse width range
  myServo.write(closedAngle);  //start with servo closed

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  myData.id = 8;  //id of system, will be 1, 5, or 8
  myData.y = 0;

  startupTime = millis();  //record startup time
}

// ==== Callback ====
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  Serial.print("Send Status: ");
  Serial.println(lastSendSuccess ? "Success" : "Fail");
}

// ==== Current Reading ====
float readACCurrentValue(int pin) {
  float peak = 0;
  for (int i = 0; i < 5; i++) {
    peak += analogRead(pin);
    delay(1);
  }
  peak = peak / 5;
  float vrms = peak * 0.707;
  vrms = (vrms / 4095.0 * VREF) / 2;
  return vrms * ACTectionRange;
}

// ==== Function to send data with retry ====
void sendDataWithRetry() {
  retryCount = 0;
  do {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
    if (result == ESP_OK) {
      delay(50);
      if (lastSendSuccess) {
        Serial.println("Send successful.");
        break;
      }
    } else {
      Serial.println("Send request failed to start.");
    }
    retryCount++;
    Serial.printf("Retry #%d...\n", retryCount);
    delay(200);
  } while (retryCount < MAX_RETRIES);

  if (retryCount == MAX_RETRIES && !lastSendSuccess) {
    Serial.println("Failed to send data after retries.");
  }
}

// ==== Main Loop ====
void loop() {
  unsigned long now = millis();

  //wait for startup delay before allowing servo to open
  if (!startupComplete) {
    if (now - startupTime >= STARTUP_DELAY_MS) {
      startupComplete = true;
      Serial.println("Startup delay complete, monitoring current.");
    } else {
      delay(100);
      return; //skip rest of loop until delay passed
    }
  }

  float current1 = readACCurrentValue(ACPin1);
  float current2 = readACCurrentValue(ACPin2);
  Serial.print("Current1: ");
  Serial.print(current1, 3);
  Serial.print(" A, Current2: ");
  Serial.println(current2, 3);

  bool currentDetected = (current1 > CURRENT_THRESHOLD || current2 > CURRENT_THRESHOLD);

  //either current sensor is ON
  if (currentDetected) {
    if (!toolOn) {
      toolOn = true;
      gracePeriodActive = false;
      myServo.write(openAngle);
      myData.x = 1;
      sendDataWithRetry();  //use retry send
      lastOnSend = now;
      nextSendInterval = random(2000, 5000);  //jitter 2–5 sec
      Serial.println("Tool turned ON: Servo Open");
    } else {
      gracePeriodActive = false;
      //resending the on
      if (now - lastOnSend >= nextSendInterval) {
        myData.x = 1;
        sendDataWithRetry();
        lastOnSend = now;
        nextSendInterval = random(2000, 5000);
        Serial.println("Re-sent ON (jittered)");
      }
    }
  } else {
    if (toolOn && !gracePeriodActive) {
      gracePeriodActive = true;
      lastOffTime = now;
      Serial.println("Tool OFF, starting grace period...");
    }

    if (gracePeriodActive && now - lastOffTime >= GRACE_PERIOD_MS) {
      toolOn = false;
      gracePeriodActive = false;
      myServo.write(closedAngle);
      Serial.println("Grace period expired: Servo Closed");
      
    }
  }

  delay(200);
}
