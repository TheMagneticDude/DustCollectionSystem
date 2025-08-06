//This code is for the one current sensor systems part of the automated dust collection system
//These are referred to as systems 3, 4, and 7.
//When a tool is turned on, the servo turns on and the
//ON signal is sent at ranomd intervals to the reciever ESP32. When the tool is turned off, 
//the ON signal is no longer sent and the grace period begins. 
//Once this grace period is over, the servo closes
//and an OFF signal is sent to the reciever ESP32. 
//The current sensor value code was adapted from the code given by DFRobot
//for the SEN0211. 
//By Vika Vorona :)
//Created 06/19/2025
//Updates:
//07/20/25: Updates to logic


#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

//====Servo Settings====
Servo myServo;
const int servoPin = 4;  //pin for servo, must be PWM capable
const int closedAngle = 0;   //angle to close servo
const int openAngle = 60;    //angle of open servo

//====Current Sensor Settings====
const int ACPin = 32;   //analog pin to read current sensor
#define ACTectionRange 20
#define VREF 3.3
#define CURRENT_THRESHOLD 0.03  //small nonzero threshold

//====Grace Period====
const unsigned long GRACE_PERIOD_MS = 60000;  //chnage this value to adjust grace period time in milliseconds

//====Startup Delay====
const unsigned long STARTUP_DELAY_MS = 2000;  //delay at startup
unsigned long startTime;
bool startupComplete = false;

//====ESP-NOW Settings====
uint8_t broadcastAddress[] = {0x6C, 0xC8, 0x40, 0x35, 0x0B, 0x60}; //mac address specific to the reciever esp32

typedef struct struct_message {
  int id;   //system id
  int x;  //1 = ON, 0 = OFF
  int y;  //kept for potential applications
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

//====Tool State Tracking====
bool toolOn = false;    //tracks whether tool is ON or OFF
unsigned long lastOffTime = 0;
bool gracePeriodActive = false;  //indicates whether grace period is active

//====Low Reading Filter====
int lowReadingCount = 0;
const int LOW_READING_THRESHOLD = 5;  //must be below threshold 5 consecutive times

//====Retry Settings====
bool sendInProgress = false;
bool sendFailed = false;
int retryCount = 0;
const int MAX_RETRIES = 3;
unsigned long lastRetryTime = 0;
const unsigned long RETRY_INTERVAL_MS = 500;  //wait 500 ms between retries

//====ON Jitter Resend====
unsigned long lastOnSend = 0;   //time when last ON was sent
unsigned long nextSendInterval = 0;  //random jitter between ON messages

//====Function to send data====
void sendData() {
  sendFailed = false;
  sendInProgress = true;
  retryCount = 0;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));  //send data using espnoww
  if (result != ESP_OK) {
    Serial.print("Initial send failed with error:");
    Serial.println(result);
    sendFailed = true;
    sendInProgress = false;  //since send didn't start properly
  } else {
    Serial.println("Initial send called");
  }
  lastRetryTime = millis();
}

//====Callback====
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  sendInProgress = false;
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Send Status:Success");
    sendFailed = false;
    retryCount = 0;
  } else {
    Serial.println("Send Status:Fail");
    sendFailed = true;
    lastRetryTime = millis();
  }
}

//====Current Reading Function====
//modified from current sensor code, see datasheet 
float readACCurrentValue(int pin) {
  float peak = 0;
  for (int i = 0; i < 10; i++) {   //taking ten readings
    peak += analogRead(pin);
    delay(1);
  }
  peak = peak / 10;
  float vrms = peak * 0.707;
  vrms = (vrms / 4095.0 * VREF) / 2;
  return vrms * ACTectionRange;
}

//====Setup====
void setup() {
  Serial.begin(115200);  //serial monitor for debugging of course
  WiFi.mode(WIFI_STA);   //setting wifi to station mode

  myServo.attach(servoPin, 500, 2500);  //attaching servo and its pulse limits
  myServo.write(closedAngle);     //set servo to closed position 

  startTime = millis();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);  //register callback for the send status
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  //set channel
  peerInfo.encrypt = false;  

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  myData.id = 7;   //set device id, either 3, 4, or 7
  myData.y = 0;
}

//====Main Loop====
void loop() {
  unsigned long now = millis();

  //startup delay: ignore current for the first STARTUP_DELAY_MS milliseconds
  if (!startupComplete) {
    if (now - startTime >= STARTUP_DELAY_MS) {
      startupComplete = true;
      Serial.println("Startup delay complete, beginning normal operation");
    } else {
      //keep servo closed during startup delay and send OFF state once
      myServo.write(closedAngle);
      static bool sentOffOnce = false;
      if (!sentOffOnce) {
        myData.x = 0;
        sendData();
        sentOffOnce = true;
      }
      delay(200);
      return;  //skip the rest of the loop
    }
  }

  //retry logic
  if (sendFailed && retryCount < MAX_RETRIES && (now - lastRetryTime >= RETRY_INTERVAL_MS)) {
    retryCount++;
    Serial.print("Retry attempt #");
    Serial.println(retryCount);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
    if (result != ESP_OK) {
      Serial.print("Retry send failed immediately with error:");
      Serial.println(result);
      //keep sendFailed true to try again later
    } else {
      sendFailed = false;  //will confirm in callback anyway
    }
    lastRetryTime = now;
  }

  float current = readACCurrentValue(ACPin);  //read current from senor
  Serial.print("Current:");
  Serial.print(current, 3);
  Serial.println(" A");

  bool currentDetected = (current > CURRENT_THRESHOLD); 

  if (currentDetected) {
    lowReadingCount = 0;
    if (!toolOn) {
      toolOn = true;  //setting tool to ON
      gracePeriodActive = false;  //cancel graceperiod
      myServo.write(openAngle); //open servo
      myData.x = 1;  
      sendData(); //send tool on message
      lastOnSend = now;
      nextSendInterval = random(2000, 5000);  //random time interval for next ON
      Serial.println("Tool turned ON:Servo Open");
    } else {
      gracePeriodActive = false;
      // resend ON jittered
      if (now - lastOnSend >= nextSendInterval) {
        myData.x = 1;
        sendData();
        lastOnSend = now;
        nextSendInterval = random(2000, 5000);  //next random time interval for ON send
        Serial.println("Re-sent ON (jittered)");
      }
    }
  } else {
    lowReadingCount++;

    if (lowReadingCount >= LOW_READING_THRESHOLD && toolOn && !gracePeriodActive) {
      gracePeriodActive = true;
      lastOffTime = now;
      Serial.println("Tool OFF(after several low readings),starting grace period...");
    }

    if (gracePeriodActive && now - lastOffTime >= GRACE_PERIOD_MS) {
      toolOn = false;
      gracePeriodActive = false;
      myServo.write(closedAngle);
      Serial.println("Grace period expired:Servo Closed");
     
    }
  }

  delay(200); 
}
// :)
