

#define HalfStepPin  0
#define DirPin 1 
#define StepPin 2
#define mics 1000
#define gear1 16
#define gear2 30



float  in = 0.0;
float kot_motor = 0.0;
float n = gear1/gear2;
int koraki = 0;
int pol_korak = 0;

//TODO: laser, branje iz serial
void HalfStep()
{
  digitalWrite(HalfStepPin, HIGH);
  digitalWrite(StepPin, HIGH);
  delayMicroseconds(mics);
  digitalWrite(StepPin, LOW);
  delayMicroseconds(mics);
  digitalWrite(HalfStepPin, LOW); 
}

void FullStep()
{
  digitalWrite(StepPin, HIGH);
  delayMicroseconds(mics);
  digitalWrite(StepPin, LOW);
}


void setup()
{
  pinMode(HalfStepPin, OUTPUT);
  pinMode(DirPin, OUTPUT);
  pinMode(StepPin, OUTPUT);
  Serial.begin(9600);


  
}

void loop()
{
  //Serial
  //in = Serial.parseFloat(); //obrat mizice
  ///Serial.println(in);

  

//motor
  kot_motor = in * 3.0;

  //kot v korake
  if (int(kot_motor*10) % int(1.8*10) == 0){
    koraki = int(kot_motor/1.8);
    pol_korak=0;
  }
  else{
    pol_korak = 1;
    koraki = int((kot_motor-0.9)/1.8);
  }

  for(int i; i < koraki; i++){
    FullStep();
  }

  if(pol_korak == 1){
    HalfStep();
  }

}
