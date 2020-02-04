 void setup()
{
  Serial.begin(9600);
  //pinMode(4, OUTPUT);
  DDRG = DDRG | B00100000;
}

void loop()
{
  PORTG = PORTG ^ B00100000; 
  delay(300);
}
