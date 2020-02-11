#include <SPI.h>
//#include <Math.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x04  //i2c arduino address

//----------------Front left leg-----------------
#define A1PWM_PIN  7       // Speed of motor A and B in PWM units, value from 0 to 255
#define A1EN_PIN   8        // Motor A is top motor B is botom

#define B1PWM_PIN  5      // This sets the direction of motor A and B, set to HIGH for outward motion and LOW for inward motion
#define B1EN_PIN   6


//----------------Front right leg-----------------
#define A2PWM_PIN  11
#define A2EN_PIN   12

#define B2PWM_PIN  9 
#define B2EN_PIN   10

//----------------Encoders-----------------------
#define CSA2 44 //Chip or Slave select Angle A leg 2
#define CSB2 42 //Chip or Slave select Angle B leg 2

#define CSA1 48 //Chip or Slave select Angle A leg 1
#define CSB1 46 //Chip or Slave select Angle B leg 1

#define WAITING_MS 20 // time left by the master between reads

#define WAITING_DS 150 // time left by the master between reads

//----------------Angles Mesurments and calculation variables--------

uint8_t temp[2]; // contains the MSB, LSB of the position

double deg1 = 0;
double deg2 = 0;

uint16_t ABSposition = 0;  //  stores the Absolute position to be read from the encoder
//uint16_t ABSposition_last = 0; //  stores the last known position

unsigned long s = 0;
unsigned long lastTrans = 0;

int motorPwmPinsArray[4] = {A1PWM_PIN, B1PWM_PIN, A2PWM_PIN, B2PWM_PIN};
int motorEnablePinsArray[4] = {A1EN_PIN, B1EN_PIN, A2EN_PIN, B2EN_PIN};

unsigned long Dt = 0;
unsigned long Mt = 0;
unsigned long St = 0;
unsigned long currentMillis = 0;
unsigned long startedMovingMillis[4] = {0 ,0 ,0 ,0};

bool flagZero = 0;
int flagMoving[4] = {0, 0, 0, 0};

int moveMotorDur[4] = {0, 0, 0, 0};

uint16_t reachMargin = 1;  // angle margin when reaching new position (total margin is twice that amount)
double minAngleA, minAngleB, maxAngleA, maxAngleB; //Set min and max position of leg, min retracted max extended
                                                   //Actual value doesnt matter (min can be > max)
double AJointAng, BJointAng;  // actual angular position of the joints received from the encoders
double AJointSetpoint, BJointSetpoint;

double legsAngles[4] = {0, 0, 0, 0};   //A1, B1, A2, B2
double legsAnglesPrevious[4] = {0, 0, 0, 0};    // memory of previous angle value mesured
double legsAnglesZero[4] = {0, 0, 0, 0};
double legsAnglesTarget[4] = {0, 0, 0, 0};

int encoderCSPinsArray[4]={CSA1, CSB1, CSA2, CSB2};

int tryErrorAngle = 0;


//---------------data and transmissions variables----------------

byte dataReceived = 0;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use by strtok() function

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};

boolean newData = false;
//---------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  pinMode(CSA1,OUTPUT);//Slaves encoders configuration 
  pinMode(CSB1,OUTPUT);
  pinMode(CSA2,OUTPUT); 
  pinMode(CSB2,OUTPUT);
  digitalWrite(CSA1,HIGH);  //Slaves encoders deSelected
  digitalWrite(CSB1,HIGH);
  digitalWrite(CSA2,HIGH);
  digitalWrite(CSB2,HIGH);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  Serial.begin(9600);

  Serial.println("starting");
  Serial.flush();
  Dt = millis();
  
  while(millis() < Dt+WAITING_DS);
  Dt = millis();
  SPI.end();
  Mt = micros();

  pinMode(A1PWM_PIN, OUTPUT);
  pinMode(A1EN_PIN, OUTPUT);

  pinMode(B1PWM_PIN, OUTPUT);
  pinMode(B1EN_PIN, OUTPUT);

  pinMode(A2PWM_PIN, OUTPUT);
  pinMode(A2EN_PIN, OUTPUT);

  pinMode(B2PWM_PIN, OUTPUT);
  pinMode(B2EN_PIN, OUTPUT);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("Quad Motor Leg Test : Start!");
  
  s = millis();

}

uint8_t SPI_T (int index, uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //  variable to hold received data
   digitalWrite(encoderCSPinsArray[index],LOW);  //  select spi device 
   msg_temp = SPI.transfer(msg); // sends (receive) message to (from) the encoder #index
   digitalWrite(encoderCSPinsArray[index],HIGH); //  deselect spi device
   return (msg_temp);    //return received byte
}

void getPosition(int index){
  int tryAmount = 0;
  uint8_t received = 0xA5;    //just a temp variable that the encoder will send if he's not ready of there isn't any remaining data to be sent
  ABSposition = 0;    //reset position variable
  
  SPI.begin();    //start transmition
  
  SPI_T(index,0x10);   //issue read command to encoder #1
  received = SPI_T(index,0x00);    //issue NOP to check if encoder is ready to send
  while (received != 0x10 and tryAmount < 3)    //loop while encoder is not ready to send 
  {
    received = SPI_T(index,0x00);    //check again if both encoders are still working 
    //Serial.println("Encoder #1 not ready ! "); 
    while(micros() < Mt+WAITING_MS);
    Mt = micros();
    tryAmount += 1;
    //Serial.println(tryAmount);
  }
  
  temp[0] = SPI_T(index,0x00);    //Receive MSB
  temp[1] = SPI_T(index,0x00);    //Receive LSB
  temp[0] &=~ 0xF0;    //mask out the first 4 bits
  
  SPI.end();    //end transmition
  
  ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
  ABSposition += temp[1];    // add LSB to ABSposition message to complete message
  
  legsAngles[index] = ABSposition;
  legsAngles[index] = legsAngles[index] * 0.08789;    // approx 360/4096
  if (index == 1 or index == 2){
    legsAngles[index] = map(legsAngles[index],0,360,360,0);    
  }
  //Serial.println(legsAngles[index]);     //send position in degrees
  
  if ((legsAngles[index] < legsAnglesPrevious[index]-1 or legsAngles[index] > legsAnglesPrevious[index]+1) and tryErrorAngle < 3 and flagZero){     //discard wrong measurments 
    legsAngles[index] = legsAnglesPrevious[index];
    tryErrorAngle += 1;
  }
  else{
    legsAnglesPrevious[index] = legsAngles[index];
    tryErrorAngle = 0;
  }
}
/**
void setAngleLimits(int choiceAngleToLimit){
  //getPosition();
  if (choiceAngleToLimit == 0){
    minAngleA = AJointAng;
    flagMinA = 1;
    Serial.println("Minimum Angle A: ("+String(minAngleA)+")");
  }
  else if (choiceAngleToLimit == 1){
    maxAngleA = AJointAng;
    flagMaxA = 1;
    Serial.println("Maximum Angle A: ("+String(maxAngleA)+")");
  }
  else if (choiceAngleToLimit == 2){
    minAngleB = BJointAng;
    flagMinB = 1;
    Serial.println("Minimum Angle B: ("+String(minAngleB)+")");
  }
  else if (choiceAngleToLimit == 3){
    maxAngleB = BJointAng;
    flagMaxB = 1;
    Serial.println("Maximum Angle B: ("+String(maxAngleB)+")");
  }
  if (flagMinA and flagMaxA and flagMinB and flagMaxB){
    if (minAngleA > maxAngleA){
      flagReverseMotorA = 1;
      Serial.println("Inverted motor A");
    }
    else{
      flagReverseMotorA = 0;
      Serial.println("Normal motor A");
    }
    if (minAngleB > maxAngleB){
      flagReverseMotorB = 1;
      Serial.println("Inverted motor B");
    }
    else{
      flagReverseMotorB = 0;
      Serial.println("Normal motor B");
    }
  }
}
**/
/**
void startMoveOneMotor(int choiceMotor, int choiceDir, int choiceSpe, int choiceDur){     //mouvement avec timer
  if (flagMoving[choiceMotor] == 0){
    digitalWrite(motorEnablePinsArray[choiceMotor], choiceDir);
    analogWrite(motorPwmPinsArray[choiceMotor], choiceSpe);
    startedMovingMillis[choiceMotor] = millis();
    moveMotorDur[choiceMotor] = choiceDur;
    flagMoving[choiceMotor] = 1;
    Serial.println("Starting Move " + String(choiceMotor) + String(choiceDir));
  }
}

void updateMotorTime(){                                                                   //mouvement avec timer
  for (int i = 0; i<=3; i++){
    if (flagMoving[i]==1){
      tryStopMoveOneMotor(i);
    }
  }
}

void tryStopMoveOneMotor(int choiceMotor){                                                //mouvement avec timer
  if (currentMillis > startedMovingMillis[choiceMotor] + moveMotorDur[choiceMotor]){
    analogWrite(motorPwmPinsArray[choiceMotor], 0);
    flagMoving[choiceMotor] = 0;
    Serial.println("Ended Move");
  }
}
**/
void changeOneMotorAngle(int choiceMotor, int choiceDir, double choiceAngle){     //mouvement avec angle
  if (flagZero){
    if (choiceDir){
      legsAnglesTarget[choiceMotor] += choiceAngle;
    }
    else{
      legsAnglesTarget[choiceMotor] -= choiceAngle;
    }
  }
}

void updateMotorAngle(){                                                               //mouvement avec angle
  if (flagZero){
    for (int i = 0; i<=1; i++){
      getPosition(i);
      if (flagMoving[i]==0){
        tryStartMoveOneMotor(i);
      }
      else if (flagMoving[i]==1){
        tryStopMoveOneMotor(i);
      }
    }
  }
}

void tryStartMoveOneMotor(int index){
  if (legsAngles[index] > legsAnglesTarget[index] + 1){
    digitalWrite(motorEnablePinsArray[index], 0);
    analogWrite(motorPwmPinsArray[index], 255);
    flagMoving[index] = 1;
  }
  else if (legsAngles[index] < legsAnglesTarget[index] - 1){
    digitalWrite(motorEnablePinsArray[index], 1);
    analogWrite(motorPwmPinsArray[index], 255);
    flagMoving[index] = 1;
  }
}

void tryStopMoveOneMotor(int index){
  if (legsAngles[index] > legsAnglesTarget[index] - 1 and legsAngles[index] < legsAnglesTarget[index] + 1){
    analogWrite(motorPwmPinsArray[index], 0);
    flagMoving[index] = 0;
  }
}

/**
void manualMoveTwoLeg(int choiceIteration){
  unsigned long tSign = millis();
  analogWrite(motorPwmPinsArray[0], 255);
  analogWrite(motorPwmPinsArray[2], 255);
    for (int i = 0; i<choiceIteration; i++){
      tSign = millis();
      digitalWrite(motorEnablePinsArray[0], 1);
      digitalWrite(motorEnablePinsArray[2], 0);
      while(millis() < tSign + 2000);
      tSign = millis();
      digitalWrite(motorEnablePinsArray[0], 0);
      digitalWrite(motorEnablePinsArray[2], 1);
      while(millis() < tSign + 2000);
    }
  analogWrite(motorPwmPinsArray[0], 0);
  analogWrite(motorPwmPinsArray[2], 0);
}
**/
/**
void reachAngle(double angle1, double angle2, int choiceSpeA, int choiceSpeB, int choiceTimeOut) {
  if (flagMinA and flagMaxA and flagMinB and flagMaxB){
    if ((!flagReverseMotorA and angle1 >= minAngleA and angle1 <= maxAngleA) 
    or (flagReverseMotorA and angle1 <= minAngleA and angle1 >= maxAngleA)){
      if ((!flagReverseMotorB and angle2 >= minAngleB and angle2 <= maxAngleB)
      or (flagReverseMotorB and angle2 <= minAngleB and angle2 >= maxAngleB)){
        AJointSetpoint = angle1;
        BJointSetpoint = angle2;
        if (AJointAng > AJointSetpoint+reachMargin or AJointAng < AJointSetpoint-reachMargin){
          flagReachedAngleA = 0;
        }
        else{
          flagReachedAngleA = 1;
        }
        if (BJointAng > BJointSetpoint+reachMargin or BJointAng < BJointSetpoint-reachMargin){
          flagReachedAngleB = 0;
        }
        else{
          flagReachedAngleB = 1;
        }
        unsigned long tSign = millis();
      
        while ((!flagReachedAngleA or !flagReachedAngleB) and millis() < tSign + choiceTimeOut) {
          //getPosition();
          Serial.println(String(AJointAng) + " -> " + String(AJointSetpoint));
          Serial.println(String(BJointAng) + " -> " + String(BJointSetpoint));
        
          if(AJointAng > AJointSetpoint + reachMargin){
            digitalWrite(A1EN_PIN, flagReverseMotorA);
            analogWrite(A1PWM_PIN, choiceSpeA);
          }
          else if(AJointAng < AJointSetpoint - reachMargin){
            digitalWrite(A1EN_PIN, !flagReverseMotorA);
            analogWrite(A1PWM_PIN, choiceSpeA);
          }
          else{
            analogWrite(B1PWM_PIN, 0);
            flagReachedAngleA = 1;
          }
          if(BJointAng > BJointSetpoint + reachMargin){
            digitalWrite(B1EN_PIN, flagReverseMotorB);
            analogWrite(B1PWM_PIN, choiceSpeB);
          }
          else if(BJointAng < BJointSetpoint - reachMargin){
            digitalWrite(B1EN_PIN, !flagReverseMotorB);
            analogWrite(B1PWM_PIN, choiceSpeB);
          }
          else{
            analogWrite(B1PWM_PIN, 0);
            flagReachedAngleB = 1;
          }
        }
        analogWrite(A1PWM_PIN, 0);
        analogWrite(B1PWM_PIN, 0);
        Serial.println("Ending loop");
      }
      else{
        Serial.println("Angle B targeted not in range");
      }
    }
    else{
      Serial.println("Angle A targeted not in range");
    }
  }
  else{
    Serial.println("Not calibrated");
  }
}
**/

void receiveData(int byteCount) {
  while (Wire.available()) {
    dataReceived = Wire.read();
    Serial.print("data received: ");
    Serial.println(dataReceived);
  }
}
void sendData() {
  Wire.write(dataReceived);
}

void readReceivedData(){
  byte maskMot = 3;
  byte maskDir = 1;
  
  if (dataReceived and dataReceived < 16){
    changeOneMotorAngle(dataReceived>>2 & maskMot, dataReceived>>1 & maskDir, 5);
  }
  else if(dataReceived == 16){
    setZero();
  }
  dataReceived = 0;
}

void setZero(){
  for (int i = 0; i<=1; i++){
     getPosition(i);
     legsAnglesZero[i] = legsAngles[i];
     legsAnglesTarget[i] = legsAngles[i];
     
     Serial.println("zero set for "+String(i)+" : "+String(legsAnglesZero[i]));
  }
  flagZero = 1;
}

void loop() {
  currentMillis = millis();
  
  readReceivedData();
  //updateMotorTime();
  updateMotorAngle();
  //getPosition(0);
  Serial.print(legsAngles[0]);
  Serial.print("  ->  ");
  Serial.println(legsAnglesTarget[0]);
  //getPosition(1);
}
