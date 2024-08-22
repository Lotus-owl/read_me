/*
HC06 for stm32
*/
#include <SoftwareSerial.h>

SoftwareSerial HC06(2, 3); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  HC06.begin(9600);
}

void loop() { // run over and over
  if (HC06.available()) {
    Serial.write(HC06.read());
  }
  if (Serial.available()) {
    HC06.write(Serial.read());
  }
}
