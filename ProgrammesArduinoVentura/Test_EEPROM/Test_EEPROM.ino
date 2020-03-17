#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
  int addres = 0;
  int leg = EEPROM.read(addres);
  
  Serial.begin(9600);
  Serial.println(leg);
}

void loop() {
  // put your main code here, to run repeatedly:

}
