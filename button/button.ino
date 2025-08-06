//This code is for the button systems part of the automated dust collection system
//These are referred to as system 2 and system 6.
//When a button is pressed, the servo turns on and the
//ON signal is sent to the reciever ESP32. When the button is pressed again,
//the ON signal is no longer sent to the reciever and
//the grace period begins. Once this grace period is over, the servo closes
//and an OFF signal is sent to the reciever ESP32. 
//By Vika Vorona 
//Final version created 06/19/2025
//vorona.1@osu.edu

#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// === Servo Setup ===
Servo myServo;

// === Receiver MAC Address ===
uint8_t broadcastAddress[] = {0x6C, 0xC8, 0x40, 0x35, 0x0B, 0x60}; //specific to the reciever ESP32

// === Data Structure ===
typedef struct struct_message {
  int id;  //id of the sender
  int x;  // 1 = ON, 0 = OFF
  int y;  // currently unused, for potential added functionality 
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// === Button & State ===
const int buttonPin = 16;    //pin for input button
bool toolState = false;      //tracks current state of tool 
int lastButtonState = HIGH;

// === Grace Period Config ===
unsigned long gracePeriodMillis = 60000;  //<-- Change this to adjust OFF delay
unsigned long offRequestTime = 0;         //time when off was requested 
bool offPending = false;

// === Startup delay config ===
const unsigned long startupDelayMillis = 2000; //startup delay
unsigned long startTime;

// === Retry config ===
const int maxRetries = 3;   //max number of resend attemps 
int sendRetryCount = 0;
bool sendPending = false;   
uint8_t* lastSendData = nullptr;
size_t lastSendSize = 0;

// === Jitter Protection ===
bool lastStableToolState = false;  //used to avoid false toggles

// === Jittered Repeated ON Send Setup ===
unsigned long lastSendTime = 0;
unsigned long nextSendDelay = 0;
const unsigned long minJitterMillis = 300;  //minimum jitter delay (ms)
const unsigned long maxJitterMillis = 700;  //maximum jitter delay (ms)

//send callback to track success/failure
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  if (status == ESP_NOW_SEND_SUCCESS) {   //check if send was successful 
    Serial.println("Delivery Success");
    sendRetryCount = 0;
    sendPending = false;  //clear retry flag on success
  } else {
    Serial.println("Delivery Fail");
    if (sendRetryCount < maxRetries) {
      sendRetryCount++;
      Serial.printf("Retrying send #%d...\n", sendRetryCount);
      esp_now_send(broadcastAddress, lastSendData, lastSendSize);
    } else {
      Serial.println("Max retries reached, giving up.");
      sendRetryCount = 0;
      sendPending = false;
    }
  }
}

//wrapper to send with retry logic
void sendWithRetry(uint8_t* data, size_t size) {
  lastSendData = data;
  lastSendSize = size;
  sendRetryCount = 0;
  sendPending = true;
  esp_now_send(broadcastAddress, data, size);   //sending the data using espnow
}

void setup() {
  Serial.begin(115200);  //for debugging ofc

  // === Setup Servo ===
  myServo.attach(4, 500, 2500);  //servo connected to pin 4, but be PWM capable 
  myServo.write(0);  //start closed

  // === Setup Button ===
  pinMode(buttonPin, INPUT_PULLUP);  //using internal pullup resistor 

  // === Setup WiFi & ESP-NOW ===
  WiFi.mode(WIFI_STA); //wifi to station mode

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);  //send callback for when data is sent 

  memcpy(peerInfo.peer_addr, broadcastAddress, 6); //copying mac address into peerInfo
  peerInfo.channel = 0;   //set communication channel 
  peerInfo.encrypt = false;  //encryption disabled 

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {   //adding peer to espnow
    Serial.println("Failed to add peer");
    return;
  }

  myData.id = 6;   //device id for system (2 or 6)
  myData.y = 0;

  //record start time for startup delay
  startTime = millis();
}

void loop() {
  //wait for startup delay before allowing button to act
  if (millis() - startTime < startupDelayMillis) {
    delay(10);
    return;
  }

  int currentButtonState = digitalRead(buttonPin);

  //button just pressed
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    delay(50);  //debounce
    if (digitalRead(buttonPin) == LOW) {
      toolState = !toolState;

      if (toolState) {
        // === Turn ON  ===
        myData.x = 1;  //set data to ON
        sendWithRetry((uint8_t *)&myData, sizeof(myData));
        myServo.write(65);  //open servo angle
        Serial.println("Tool state sent: ON");
        Serial.println("Servo: OPEN");
        offPending = false; //cancel any pending OFF

        //initialize jitter timers on turning ON
        lastSendTime = millis();
        nextSendDelay = random(minJitterMillis, maxJitterMillis);
      } else {
        // === Turn OFF after grace period ===
        offRequestTime = millis();
        offPending = true;
        Serial.println("Tool OFF requested, waiting grace period...");
      }
    }
  }

  // === Jittered repeated ON send while toolState is ON ===
  if (toolState) {
    if (millis() - lastSendTime >= nextSendDelay) {
      myData.x = 1;  //ON signal
      sendWithRetry((uint8_t *)&myData, sizeof(myData));
      Serial.println("Repeated ON signal sent with jitter");
      lastSendTime = millis();
      nextSendDelay = random(minJitterMillis, maxJitterMillis);
    }
  } else {
    //reset timers when tool is OFF
    lastSendTime = 0;
    nextSendDelay = 0;
  }

  //handle delayed OFF
  if (offPending && millis() - offRequestTime >= gracePeriodMillis) {
    //check if state stayed off during grace period (jitter protection)
    if (!toolState && !lastStableToolState) {
      myData.x = 0;
      sendWithRetry((uint8_t *)&myData, sizeof(myData)); //send off message
      myServo.write(0);  //servo closed angle
      Serial.println("Tool state sent: OFF");
      Serial.println("Servo: CLOSED");
      offPending = false;
    } else {
      Serial.println("OFF canceled due to reactivation during grace period");
      offPending = false;
    }
  }

  lastStableToolState = toolState;  //remember last stable state
  lastButtonState = currentButtonState;
  delay(10);
}
