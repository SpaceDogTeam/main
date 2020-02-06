#include <SPI.h>
#include <Math.h>


#define APWM_PIN  7       // Speed of motor A and B in PWM units, value from 0 to 255
#define AEN_PIN   8

#define BPWM_PIN  5      // This sets the direction of motor A and B, set to HIGH for outward motion and LOW for inward motion
#define BEN_PIN   6

#define CS1 46 //Chip or Slave select  #1
#define CS2 48 //Chip or Slave select  #2

#define WAITING_MS 20 // time left by the master between reads

#define WAITING_DS 150 // time left by the master between reads

long initialMillis = 0; // This records the initial time, at which the program started running
long currentMillis = 0;

uint8_t temp1[1]; // contains the MSB of the position
uint8_t temp2[1]; //  contains the LSB of the position

double deg1 = 0;
double deg2 = 0;

uint16_t ABSposition1 = 0;  //  stores the Absolute position to be read from the encoder
uint16_t ABSposition_last1 = 0; //  stores the last known position
uint16_t ABSposition2 = 0;
uint16_t ABSposition_last2 = 0;

unsigned long s = 0;
unsigned long lastTrans = 0;

unsigned long Dt = 0;
unsigned long Mt = 0;
unsigned long St = 0;

bool flagReachedAngleA, flagReachedAngleB;
bool flagMinA, flagMaxA, flagMinB, flagMaxB;
bool flagReverseMotorA, flagReverseMotorB;
uint16_t reachMargin = 1;  // angle margin when reaching new position (total margin is twice that amount)
double minAngleA, minAngleB, maxAngleA, maxAngleB; //Set min and max position of leg, min retracted max extended
                                                   //Actual value doesnt matter (min can be > max)
double AJointAng, BJointAng;  // actual angular position of the joints received from the encoders
double AJointSetpoint, BJointSetpoint;


//============
int flagTransmission;
int val1;
int val2;
int val3;
int val4;
int val5;
int val6;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use by strtok() function

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};

boolean newData = false;
//============

void setup() {
  // put your setup code here, to run once:
  pinMode(CS1,OUTPUT);//Slave#1 configuration 
  pinMode(CS2,OUTPUT);//Slave#2 configuration 
  digitalWrite(CS1,HIGH);  //  Slave#1 deSelected
  digitalWrite(CS2,HIGH);  //  Slave#2 deSelected
  
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

  pinMode(APWM_PIN, OUTPUT);
  pinMode(AEN_PIN, OUTPUT);

  pinMode(BPWM_PIN, OUTPUT);
  pinMode(BEN_PIN, OUTPUT);

  Serial.println("Dual Motor Leg Test : Start!");
  
  s = millis();
  
  getPosition();
  flagReachedAngleA = 1;
  flagReachedAngleB = 1;
  flagMinA = 0;
  flagMaxA = 0;
  flagMinB = 0;
  flagMaxB = 0;
  flagReverseMotorA=0;
  flagReverseMotorB=0;
  
}

uint8_t SPI_T (int index, uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //  variable to hold received data
   if (index == 1){
    digitalWrite(CS1,LOW);  //  select spi device 
    msg_temp = SPI.transfer(msg); // sends (receive) message to (from) the encoder #1
    digitalWrite(CS1,HIGH); //  deselect spi device
   }
   else{
    digitalWrite(CS2,LOW);
    msg_temp = SPI.transfer(msg);
    digitalWrite(CS2,HIGH);
   }
   return (msg_temp);    //return received byte
}

void getPosition(){
  uint8_t received = 0xA5;    //just a temp variable that the encoder will send if he's not ready of there isn't any remaining data to be sent
   ABSposition1    = 0;    //reset position variable
   ABSposition2 = 0;    //reset position variable
   
   SPI.begin();    //start transmition
   
   SPI_T(1,0x10);   //issue read command to encoder #1
   received = SPI_T(1,0x00);    //issue NOP to check if encoder is ready to send
   while (received != 0x10)    //loop while encoder is not ready to send 
   {
     received = SPI_T(1,0x00);    //check again if both encoders are still working 
     //Serial.println("Encoder #1 not ready ! "); 
     while(micros() < Mt+WAITING_MS);
     Mt = micros();
   }
   
   temp1[0] = SPI_T(1,0x00);    //Receive MSB
   temp1[1] = SPI_T(1,0x00);    //Receive LSB
   temp1[0] &=~ 0xF0;    //mask out the first 4 bits
   
   SPI_T(2,0x10);
   received = SPI_T(2,0x00);
   while (received != 0x10)
   {
     received = SPI_T(2,0x00);
     //Serial.println("Encoder #2 not ready ! "); 
     while(micros() < Mt+WAITING_MS);
     Mt = micros();
   }
   
   temp2[0] = SPI_T(2,0x00);
   temp2[1] = SPI_T(2,0x00);
   temp2[0] &=~ 0xF0;
   
   SPI.end();    //end transmition
   
   ABSposition1 = temp1[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
   ABSposition1 += temp1[1];    // add LSB to ABSposition message to complete message
   ABSposition2 = temp2[0] << 8;
   ABSposition2 += temp2[1];
    
   if (ABSposition1 != ABSposition_last1)    //if nothing has changed dont wast time sending position
   {
     ABSposition_last1 = ABSposition1;    //set last position to current position
     deg1 = ABSposition1;
     deg1 = deg1 * 0.08789;    // approx 360/4096

     deg1 = map(deg1,0,360,360,0);

     //Serial.println(deg1);     //send position in degrees
   }else{
    //Serial.println("Encoder #1 value received unchanged ! "); 
   }
   if (ABSposition2 != ABSposition_last2)
   {
     ABSposition_last2 = ABSposition2;
     deg2 = ABSposition2;
     //deg2 = (deg2 * 0.08789)%360;
     //Serial.println(deg2);
     deg2 = deg2 * 0.08789;

     deg2 = map (deg2,360,0,0,360);

   }
   else{
    //Serial.println("Encoder #2 value received unchanged ! "); 
   }
   AJointAng = deg1 ;
   BJointAng = deg2 ;
}

void setAngleLimits(int choiceAngleToLimit){
  getPosition();
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

void manualMove(int choiceDirA, int choiceDirB, int choiceSpeA, int choiceSpeB, int choiceDur){
  unsigned long tSign = millis();
  digitalWrite(AEN_PIN, choiceDirA);
  digitalWrite(BEN_PIN, choiceDirB);
  analogWrite(APWM_PIN, choiceSpeA);
  analogWrite(BPWM_PIN, choiceSpeB);
  while(millis() < tSign + choiceDur);   //  ms tempo equal to choiceDur (duration)
  analogWrite(APWM_PIN, 0);
  analogWrite(BPWM_PIN, 0);
  Serial.println("Moved");
}

void reachAngle(double angle1, double angle2) {
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
      
        while ((!flagReachedAngleA or !flagReachedAngleB) and millis() < tSign + 25000) {
          getPosition();
          Serial.println(String(AJointAng) + " -> " + String(AJointSetpoint));
          Serial.println(String(BJointAng) + " -> " + String(BJointSetpoint));
        
          if(AJointAng > AJointSetpoint + reachMargin){
            digitalWrite(AEN_PIN, flagReverseMotorA);
            analogWrite(APWM_PIN, 180);
          }
          else if(AJointAng < AJointSetpoint - reachMargin){
            digitalWrite(AEN_PIN, !flagReverseMotorA);
            analogWrite(APWM_PIN, 180);
          }
          else{
            analogWrite(BPWM_PIN, 0);
            flagReachedAngleA = 1;
          }
          if(BJointAng > BJointSetpoint + reachMargin){
            digitalWrite(BEN_PIN, flagReverseMotorB);
            analogWrite(BPWM_PIN, 180);
          }
          else if(BJointAng < BJointSetpoint - reachMargin){
            digitalWrite(BEN_PIN, !flagReverseMotorB);
            analogWrite(BPWM_PIN, 180);
          }
          else{
            analogWrite(BPWM_PIN, 0);
            flagReachedAngleB = 1;
          }
        }
        analogWrite(APWM_PIN, 0);
        analogWrite(BPWM_PIN, 0);
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

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {

  // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(tempChars,",");      // get the first part - the string
  flagTransmission = atoi(strtokIndx); // copy it to messageFromPC
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  val1 = atoi(strtokIndx);     // convert this part to an integer,
                               //can be to a float too with atof?
  strtokIndx = strtok(NULL, ",");
  val2 = atoi(strtokIndx);     // convert this part to an int

  strtokIndx = strtok(NULL, ",");
  val3 = atoi(strtokIndx);     // convert this part to an int

  strtokIndx = strtok(NULL, ",");
  val4 = atoi(strtokIndx);     // convert this part to an int

  strtokIndx = strtok(NULL, ",");
  val5 = atoi(strtokIndx);     // convert this part to an int

  strtokIndx = strtok(NULL, ",");
  val6 = atoi(strtokIndx);     // convert this part to an int

}

void interpretData() {

  char  buff[30];
  sprintf(buff,"ack <%d,%d,%d,%d,%d,%d>",flagTransmission,val1,val2,val3,val4,val5);
  Serial.flush();
  Serial.print(buff);
  
  if (flagTransmission == 0){
      setAngleLimits(val1);  //0 minA, 1 maxA, 2 minB, 3 maxB
  }
  else if (flagTransmission == 1){  //manually move 1 motor with direction,speed and duration.
      manualMove(val1,val2,val3,val4,val5);   //dirA (0 retract, 1 extend), dirB, speedA(0-255, 165 min to move), speedB, duration(ms)
  }
  else if (flagTransmission == 2){
    getPosition();
    Serial.println("Angles: ("+String(AJointAng)+", "+String(BJointAng)+")");
  }
  else if (flagTransmission == 3){
    reachAngle(val1,val2);
  }
  else if (flagTransmission == 4){
    
  }
  else if (flagTransmission == 5){
    
  }
  else if (flagTransmission == 6){
   
  }
  else if (flagTransmission == 7){
    
  }
  else if (flagTransmission == 8){
    if (val1 != 0){
      
    }
    else if (val2 != 0){
 
    }
  }
  else if (flagTransmission == 9){
   
  }
  else if (flagTransmission == 10){
    if (val1 != 0){
      
    }
    else if (val2 != 0){
  
    }
  }
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() replaces the commas with \0
    parseData();
    /*showParsedData();*/
    interpretData();
    newData = false;
  }
}
