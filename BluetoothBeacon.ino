#include <SoftwareSerial.h>

#define BT_NAME "BUS-1070"

SoftwareSerial btModule(2,3);

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Serial.begin(9600);
  btModule.begin(9600);

  delay(100);
  btModule.print("AT+NAME" BT_NAME);
  
  digitalWrite(13, LOW);
}

void loop() {
  if (btModule.available()) {
    String line = btModule.readStringUntil('\n');
    Serial.print(line);
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    btModule.print(line);
  }
}
