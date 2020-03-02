
/*  
 *Test AVentura 27-02-2020
 *
 *
*/

const int feedback = A0;

const int enable = 2;  //motor pwm pin
const int in1 = 3;  //motor direction pin1
const int in2 = 4;  //motor direction pin2



void setup()
{
  pinMode(feedback, INPUT);
  
  pinMode(enable, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  analogWrite(enable,0);
  Serial.begin(9600);
  Serial.println("hello");
  delay(2000);
  
}

void loop()
{ 
  
  
  Serial.print("Position :  ");
  Serial.println(analogRead(feedback));
  Serial.println("forward");
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  analogWrite(enable,255);
  delay(5000);
  Serial.print("Position :  ");
  Serial.println(analogRead(feedback));
  Serial.println("backward");
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  analogWrite(enable,255);
  delay(5000);
  
  
}
