#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial xbee(2, 3);
char val;
String value;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {;}
  xbee.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    char val = Serial.read();
    String value=String(val);
    xbee.print(value);
  }
}
