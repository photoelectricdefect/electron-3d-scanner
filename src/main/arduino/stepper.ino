#include <AccelStepper.h>

//#define half_step_pin  0
#define dir_pin 48 
#define step_pin 50

#define ms1_pin 28
#define ms2_pin 30
#define ms3_pin 32

#define laser_pin 52

#define interface_type 1

#define mics 1000
#define gear1 16
#define gear2 30

const String ROTATE="rotate";
const String LASER="laser";

const int ERR_OK=0;

const int FULL_STEP=1.8;
const int HALF_STEP=0.9;


AccelStepper stepper = AccelStepper(interface_type, step_pin, dir_pin);

float in = 0.0;
float kot_motor = 0.0;
float n = gear1/gear2;
int koraki = 0;
int pol_korak = 0;

//https://stackoverflow.com/questions/29671455/how-to-split-a-string-using-a-specific-delimiter-in-arduino
String get_value(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//TODO: laser, branje iz serial
//void hstep()
//{
  //digitalWrite(half_step_pin, HIGH);
  //digitalWrite(step_pin, HIGH);
  //delayMicroseconds(mics);
  //digitalWrite(step_pin, LOW);
  //delayMicroseconds(mics);
  //digitalWrite(half_step_pin, LOW); 
//}

//void fstep()
//{
  //digitalWrite(step_pin, HIGH);
  //delayMicroseconds(mics);
  //digitalWrite(step_pin, LOW);
//}


void setup()
{
  pinMode(laser_pin,OUTPUT);
  pinMode(ms1_pin,OUTPUT);
  pinMode(ms2_pin,OUTPUT);
  pinMode(ms3_pin,OUTPUT);
  digitalWrite(ms1_pin,LOW);
  digitalWrite(ms2_pin,LOW);
  digitalWrite(ms3_pin,LOW);
  stepper.setMaxSpeed(1000);
  Serial.begin(9600);
}

void loop()
{    
  if(Serial.available()>0) {
  String in = Serial.readStringUntil('\r');    
  String name=get_value(in,';',0);

  if(name.equals(ROTATE)) {
      String s1=get_value(in,';',1);
      String s2=get_value(in,';',2);
      float angle=get_value(s1,':',1).toFloat();
      int dir=get_value(s2,':',1).toInt();

      int steps=angle/FULL_STEP;
  stepper.setCurrentPosition(0);
  stepper.runToNewPosition(steps);
  delay(50);

      Serial.print("{'errcode':"+String(ERR_OK)+"}\n");
  
   //motor
      //kot_motor = in * 3.0;

  //kot v korake
      //if (int(kot_motor*10) % int(1.8*10) == 0){
        //koraki = int(kot_motor/1.8);
        //pol_korak=0;
      //}
      //else{
        //pol_korak = 1;
        //koraki = int((kot_motor-0.9)/1.8);
      //}

      //for(int i; i < koraki; i++){
        //fstep();
      //}

      //if(pol_korak == 1){
        //hstep();
      //} 
  }
  else if(name.equals(LASER)){
      String s1=get_value(in,';',1);
      int state=get_value(s1,':',1).toInt();

      if(state==0) digitalWrite(laser_pin,LOW);
      else digitalWrite(laser_pin,HIGH);

      delay(10); 
      Serial.print("{'errcode':"+String(ERR_OK)+"}\n");
  } 
  }
}
