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

#define FALSE false
#define TRUE true

bool dustCollectorStatus = 0;

// Structure for storing tool objects
struct Node {
  bool isOn = FALSE; 
  unsigned long lastRecivedOnMessage = 0;
};

// Array to store all nodes states + 1 for last Node to be recivers local button pushes 
Node tools[MAX_TOOLS + 1];

// === Button Setup ====
const int BUTTONPIN = 13; //Pin the pushbutton is connected to
volatile bool ButtonPressed;

// === Servo Setup ===
Servo myServo;
const int SERVOPIN = 16;   //pin for servo, must be PWM capable
const int OFFANGLE = 72;  //angle when servo turns button off
const int ONANGLE = 92;  //angle when servo turns button on
const int NEUTRALANGLE = 82; //angle returned to after opening/closing

// === LED Setup ===
const int LEDPIN = 21;  //pin connected to LED for status change
bool ledState = false;  //tracks current LED state
unsigned long lastBlinkTime = 0;

// === ESP-NOW Setup ===
typedef struct struct_message {
  int id;  //id of board recieved 1 through 8
  int x;   //tool signal state (1=ON, 0=OFF)
  int y;
} struct_message;

struct_message myData;

// === Function to turn dust collector on and off ===
void setDustCollectorPower(bool state) { // 0 is off, 1 is on
  if (state){ //if 1 setting to on posistion
    myServo.write(ONANGLE);
    dustCollectorStatus = 1;
    digitalWrite(LEDPIN, HIGH);
  } else {
    myServo.write(OFFANGLE);
    dustCollectorStatus = 0;
    digitalWrite(LEDPIN, LOW);
  }
  delay(500); //Wait 500 ms for servo to move and hold poisition for dust collector to turn on

  myServo.write(NEUTRALANGLE); // Move servo back to nutral posistion
}

bool isToolOn(Node tools[]) {
  for (int i = 0; i < MAX_TOOLS + 1; i++) {
    if (tools[i].isOn) {
      return TRUE;
    }
  }
  return FALSE;
}


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
      tools[toolID].isOn = TRUE;  //Marking tool as on
      tools[toolID].lastRecivedOnMessage = millis(); //Recording time that message was recived at
      }
  } else {
    Serial.println("Error: Board ID out of range!");
  }
}

// Toggles isOn for special tool if button has not been pressed in over 1 second (handles debouncing)
void buttonPressISR () {
  ButtonPressed = TRUE;
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  myServo.attach(SERVOPIN, 500, 2500);  //attach servo to PWM capable pin 

  WiFi.mode(WIFI_STA);         //set wifi to station mode
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));   //register recieve callback DONT CHANGE THIS LINE
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), buttonPressISR, RISING);

  setDustCollectorPower(0); // Turns dust collector off when board starts up
}

// === Loop ===
void loop() {
  unsigned long now = millis();

  // Logic for if dust collector needs to be turn on or off
  if (isToolOn(tools) && !dustCollectorStatus) {
    setDustCollectorPower(1);
  } else if (!isToolOn(tools) && dustCollectorStatus) {
    setDustCollectorPower(0);
  }

  //Timouts for making tools as off logic 
  //The special extra tool at end is for manual button, this does not time it out
  for(int i = 0; i < MAX_TOOLS + 1; i++) {
    if (tools[i].isOn)
      if (now - tools[i].lastRecivedOnMessage > TOOLTIMEOUTTIME)
        tools[i].isOn = FALSE;
  }

  static unsigned long lastButtonTime = 0;
  if (ButtonPressed) {
  ButtonPressed = FALSE;  // clear the event

  if (now - lastButtonTime > 250) {   // debounce
    tools[MAX_TOOLS].isOn = !tools[MAX_TOOLS].isOn;
    tools[MAX_TOOLS].lastRecivedOnMessage = now;
    lastButtonTime = now;
    Serial.println("Button was pressed");
  }
}
  
}
