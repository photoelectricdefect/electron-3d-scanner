#include <Stepper.h>
const int steps_per_rev = 180;
Stepper stepper(steps_per_rev, 8, 9, 10, 11);
String status_msg(String status) {
  return String("{") + String( "status:") + String(status) + String("}");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool running = true;

  while(running) {
    Serial.println(status_msg("OK"));
    delay(50);
  }
}
