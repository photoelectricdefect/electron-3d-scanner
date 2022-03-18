#include <ArduinoJson.h>
#include <MultiStepper.h>
#include <AccelStepper.h>
#include "BluetoothSerial.h"

const char* NAME_BT="scanner_bt";

const int MAX_LINE_LEN=256;

const int PIN_DIR=48;
const int PIN_STEP=50;

const int PIN_MS1=28;
const int PIN_MS2=30;
const int PIN_MS3=32;

const int PIN_LASER=52;

const int INTERFACE_TYPE=1;

const String ROTATE = "rotate";
const String LASER = "laser";

const int ERR_OK = 0;

AccelStepper stepper = AccelStepper(INTERFACE_TYPE, PIN_STEP, PIN_DIR);

/* Check if Bluetooth configurations are enabled in the SDK */
/* If not, then you have to recompile the SDK */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

String read_line() {
  char line[MAX_LINE_LEN];
  memset(&line,0,MAX_LINE_LEN);
  char c='\0';
  int pos=0;

  while(c!='\r') {
    if(SerialBT.available()){
      c=SerialBT.read();
      line[pos]=c;
      pos++;
    }
    delay(1);    
  }

  Serial.println(String(line));

  return String(line);
}

void setup() {
  pinMode(PIN_LASER, OUTPUT);
  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);
  digitalWrite(PIN_MS1, LOW);
  digitalWrite(PIN_MS2, LOW);
  digitalWrite(PIN_MS3, LOW);
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(100);
  Serial.begin(9600);
  /* If no name is given, default 'ESP32' is applied */
  /* If you want to give your own name to ESP32 Bluetooth device, then */
  /* specify the name as an argument SerialBT.begin("myESP32Bluetooth"); */

  if(!SerialBT.begin(NAME_BT)){
    Serial.println("An error occurred initializing Bluetooth");
  }
  
  Serial.println("Bluetooth Started! Ready to pair...");
}

void loop() {
  String command_string=read_line();
  
  StaticJsonDocument<128> json_command;
  DeserializationError error = deserializeJson(json_command,command_string);
  String name=json_command["name"].as<String>();
    
  if(name.equals(ROTATE)) {
    int steps = json_command["steps"].as<int>();
    int direction = json_command["direction"].as<int>();
    int t_delay = json_command["delay"].as<int>();
    stepper.move(steps);
    stepper.runToPosition();
    delay(t_delay);
    StaticJsonDocument<32> json_response;
    json_response["errcode"]=ERR_OK;
    String output;
    serializeJson(json_response, output);  
    SerialBT.print(output+"\n");
  }
  else if(name.equals(LASER)) {
    int state = json_command["state"].as<int>();
    int t_delay = json_command["delay"].as<int>();

    if (state == 0)
      digitalWrite(PIN_LASER, LOW);
    else
      digitalWrite(PIN_LASER, HIGH);

    delay(t_delay);
    StaticJsonDocument<32> json_response;
    json_response["errcode"]=ERR_OK;
    String output;
    serializeJson(json_response, output);  
    SerialBT.print(output+"\n");  
  }
   
  delay(10);
}
