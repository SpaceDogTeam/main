
/*  Sample code to control the position of an actuator with potentiometer feedback using a MegaMoto.

The main loop of this program checks the potentiometer, and moves the actuator accordingly.


 Written by Progressive Automations

  This example code is in the public domain.
*/

const int enable = 2;  //motor pwm pin
const int in1 = 3;  //motor direction pin1
const int in2 = 4;  //motor direction pin2

void setup()
{
  pinMode(enable, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1,HIGH);
  digitalWrite(in2,HIGH);
  analogWrite(enable,0);
  Serial.begin(9600);
  Serial.println("hello");
  delay(2000);
  
}

void loop()
{ 
  Serial.println("forward");
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  analogWrite(enable,255);
  delay(2000);
  Serial.println("backward");
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  analogWrite(enable,255);
  delay(2000);
}
