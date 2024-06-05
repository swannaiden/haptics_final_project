
// Includes
#include <math.h>
const int pin9 = 9;
const int pin10 = 10;
float force = 0;



void setup() {
  // put your setup code here, to run once:
  pinMode(pin9, OUTPUT);
  pinMode(pin10, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
   // receive force from python serial
  if (Serial.available() > 0) {
    force = Serial.parseFloat();
    if (force >= 8){
      analogWrite(pin9, 85);
      analogWrite(pin10, 85);
    }
    else{
      analogWrite(pin9, 0);
      analogWrite(pin10, 0);
    }

  }
}
