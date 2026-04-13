// When any tool sends an ON signal, the servo will go to the ON position,
// then return to neutral. If no ON signal is detected, only then will
// the servo go to the OFF position, then return to neutral.
//If the reciever is powered off and then turned back on when the servo has most recently
//been turned on, the board will go into recovery mode. The LED will blink for 60 seconds 
//(adjustable) and then turn the servo to the off state. If during the recovery period a tool 
//turns on, the reciever will exit the recovery period. 
//By Vika Vorona 
//Modified by James Mills
//Rev 2 started 1/14/2026

#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

//number of tools there are 
#define MAX_TOOLS 8
#define TOOLTIMEOUTTIME 30000 //in ms  and want to make smaller but need to update senders code first to send more frequently. 
//WHEN SENDERS ARE UPDATED -> TODO: add code so that a timout happens right, and once all things are makred as off, the timeout starts properly
//If what is implemented now works ok, this may not be needed to do, but just consider it 

const int LEDPIN = 21;  //pin connected to LED for status change
const int BUTTONPIN = 13; //Pin the pushbutton is connected to
bool dustCollectorStatus = 0;
bool SeenButtonPressed = false;
Servo myServo;

// Structure for storing tool objects
struct Node {
  bool isOn = false; 
  unsigned long lastRecivedOnMessage = 0;
};

Node tools[MAX_TOOLS + 1];  // Array to store all nodes states + 1 for fake tool that is button presses on reciver

void turnDustCollectorOff() {
  myServo.write(0);  
  delay(300);
  myServo.write(90);  
  dustCollectorStatus = 0;
}

void turnDustCollectorOn() {
  myServo.write(180);  
  delay(300);
  myServo.write(90); 
  dustCollectorStatus = 1; 
}

bool isToolOn(Node tools[]) {
  for (int i = 0; i < MAX_TOOLS + 1; i++) {
    if (tools[i].isOn) {
      return true;
    }
  }
  return false;
}

// Toggles isOn for special tool if button has not been pressed in over 1 second (handles debouncing)
void buttonPressISR () {
  SeenButtonPressed = true;
}

// === ESP-NOW Setup ===
typedef struct struct_message {
  int id;  //id of board recieved 1 through 8
  int x;   //tool signal state (1=ON, 0=OFF)
  int y;
} struct_message;

struct_message myData;

// === Callback for Received ESP-NOW Data ===
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  int toolID = myData.id - 1; //get tool id 
  Serial.printf("Recived packet from Tool ID: %d\n", myData.id);

  if (toolID >= 0 && toolID < MAX_TOOLS) {
    if (myData.x == 1 || myData.y == 1) {
      Serial.printf("Tool %d ON signal received\n", myData.id);

      //tool id system
      //mark tool as active
      tools[toolID].isOn = true;  //Marking tool as on
      tools[toolID].lastRecivedOnMessage = millis(); //Recording time that message was recived at
      }
  } else {
    Serial.println("Error: Board ID out of range!");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  myServo.attach(16, 1000, 2000); 

  WiFi.mode(WIFI_STA);         //set wifi to station mode
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));   //register recieve callback DONT CHANGE THIS LINE
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), buttonPressISR, RISING);

  turnDustCollectorOff();
}

void loop() {
  unsigned long now = millis();

  // Logic for if dust collector needs to be turn on or off
  if (isToolOn(tools) && !dustCollectorStatus) {
    turnDustCollectorOn();
  } else if (!isToolOn(tools) && dustCollectorStatus) {
    turnDustCollectorOff();
  }

  //Timouts for making tools as off logic 
  //The special extra tool at end is for manual button, this does not time it out
  for(int i = 0; i < MAX_TOOLS; i++) {
    if (tools[i].isOn)
      if (now - tools[i].lastRecivedOnMessage > TOOLTIMEOUTTIME)
        tools[i].isOn = false;
  }

  //led output (up button led will light up if collector is currently on)
  dustCollectorStatus ? digitalWrite(LEDPIN, HIGH) :   digitalWrite(LEDPIN, LOW);

  if (SeenButtonPressed && now - tools[MAX_TOOLS].lastRecivedOnMessage > 250) {   // debounce
    tools[MAX_TOOLS].isOn = !tools[MAX_TOOLS].isOn;
    tools[MAX_TOOLS].lastRecivedOnMessage = now;
    SeenButtonPressed = false;
    Serial.println("Button was pressed, and button as tool been toggled");
  }
}
