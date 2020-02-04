#include <SPI.h>
#include <PID_v1.h>
#include <Math.h>


//------------------------------------------------------------------------
#define LED_PIN   4
#define APWM_PIN  5
#define AEN_PIN   6

#define BPWM_PIN  7
#define BEN_PIN   8

#define LOOPTIME  10   // loop time in ms

#define TIMEOUT   500  // Timeout in ms

#define DEADZONE  10    // /256 points  --- interval of confidence: +-10

//-----------------------------------------------------------------------
#define CS1 48 //Chip or Slave select  #1
#define CS2 46 //Chip or Slave select  #2

#define WAITING_MS 20 // time left by the master between reads

#define WAITING_DS 150 // time left by the master between reads


double T1_Off = 0;
double T2_Off = 0;

uint8_t temp1[1]; // contains the MSB of the position
uint8_t temp2[1]; //  contains the LSB of the position

double deg1 = 4.66;
double deg2 = 186.06;

//bool motionEnabled = !false;
bool motionEnabled = true;
bool LED_status = false;

unsigned long s = 0;
unsigned long lastTrans = 0; 

unsigned long Dt = 0;
unsigned long Mt = 0;
unsigned long St = 0;

uint16_t ABSposition1 = 0;  //  stores the Absolute position to be read from the encoder
uint16_t ABSposition_last1 = 0; //  stores the last known position
uint16_t ABSposition2 = 0;
uint16_t ABSposition_last2 = 0;

//-----------------------------------------------------------------------
double theta21[] = {40, 42.3, 44.6, 46.9, 49.2, 51, 53.3, 55.6, 53, 50.5, 47.3, 45.0, 42.5, 41, 40.5 };
double theta11[] = {85, 87.5, 90, 92.5, 95, 97.5, 100, 103, 100, 97.5, 95, 92, 89, 87, 85 };


int tLength = 15;

int pathIndex = 0;
int pathIndex2 = 0;
//------------------------------------    PID control variables --------------------------------------

uint16_t maxAngleMargin = 0.5;
double AJointSpd, BJointSpd;  // output variables to themotor
double AJointAng, BJointAng;  // actual angular position of the joints received from the encoders
double AJointSetpoint, BJointSetpoint, AJointSetpointOld, BJointSetpointOld;
double previousAJointAng = AJointSetpoint;
double previousBJointAng = BJointSetpoint;

// Tuning parameters
const int sampleRate = 10; //  sampling rate

int k1 = 0.8*10 , k2 = 0.9*0.3/4, k3 = 0.9*4*0;

double KpA=k1*3,  KpB=1.4*4;  //Initial Proportional Gain 
double KiA=k3*10/10,  KiB=k3;  //Initial Integral Gain 
double KdA=k2*5,  KdB=0.1*1;  //Initial Differential Gain 


// Instantiate X and Y axis PID controls.
PID MotorAPID(&AJointAng, &AJointSpd, &AJointSetpoint, KpA, KiA, KdA, DIRECT); 
PID MotorBPID(&BJointAng, &BJointSpd, &BJointSetpoint, KpB, KiB, KdB, REVERSE); //setting up the PID


void setup() {
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


  //------------------------------------------------------------------------

  pinMode(LED_PIN, OUTPUT);
  pinMode(APWM_PIN, OUTPUT);
  pinMode(AEN_PIN, OUTPUT);

  pinMode(BPWM_PIN, OUTPUT);
  pinMode(BEN_PIN, OUTPUT);

  Serial.println("Dual Motor Leg Test : Start!");
  
  s = millis();
  
  //-------------------------------------------------------------------------
  MotorAPID.SetMode(AUTOMATIC);
  MotorAPID.SetSampleTime(sampleRate);
  MotorAPID.SetOutputLimits(0,75);
  
  MotorBPID.SetMode(AUTOMATIC);
  MotorBPID.SetSampleTime(sampleRate);
  MotorBPID.SetOutputLimits(0,75);

  MotorAPID.Compute();
  MotorBPID.Compute();
}


//---------------------------- SPI function ----------------------
uint8_t SPI_T (int index, uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //  variable to hold received data
   if (index == 1){
    digitalWrite(CS1,LOW);  //  select spi device 
    msg_temp = SPI.transfer(msg); // sends (receive) message to (from) the encoder #1
    digitalWrite(CS1,HIGH); //  deselect spi device
   }else{
    digitalWrite(CS2,LOW);
    msg_temp = SPI.transfer(msg);
    digitalWrite(CS2,HIGH);
   }
   return (msg_temp);    //return received byte
}

//------------------------ getPosition -------------------------------------
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

   }else{
    //Serial.println("Encoder #2 value received unchanged ! "); 
   }

   
   AJointAng = deg1 ;
   BJointAng = deg2 ;
}

//------------------------ checkEnable -----------------------------------

void checkEnable(){
  int act = 0;
  if (Serial.available()>=2){
    char c = ' ';
    do{
      c = Serial.read();
    }while(c != '$'); // reads the incoming stream until he catches '$' which is the beginning of incoming data
    act = Serial.parseInt();
    Serial.print("received boolean ");
    Serial.println(act);
    lastTrans = millis(); //  keeps track of the time he received the packet
    
  
  }
  if (act == 1) motionEnabled = true;
  else if (act == 0) motionEnabled = false;
}


//--------------------------------- Computation ---------------------------

void computation(){
  if(millis() > lastTrans+TIMEOUT ) {
    
    MotorAPID.Compute();
    MotorBPID.Compute();
    analogWrite(APWM_PIN, AJointSpd);
    analogWrite(BPWM_PIN, BJointSpd);
  }
}


//----------------------------------- SetPosControl ----------------------------------


void setPosControl(){
  St = millis();
  

  //Serial.print("pathIndexes A and B are : ");
  //Serial.println(String(pathIndex) + "," + String(pathIndex2));

  
  if (pathIndex2 <= 9){
    if ((BJointAng < theta21[pathIndex2] + 1*maxAngleMargin)) {
      pathIndex2 += 1;
      //Serial.println("pathIndex has changed: path 1B" + String (theta21[pathIndex2]));
    }
  }

  else if (pathIndex2 > 9){
     if ((BJointAng > theta21[pathIndex2] - 1*maxAngleMargin)) {
      pathIndex2 += 1;
      //Serial.println("pathIndex has changed: path 2B" + String (theta21[pathIndex2]));
    }
    if (pathIndex2 == 14 && BJointAng > 59.9) pathIndex = 0;
  }/*
  else {  
    Serial.println("path 2");
    if (! (AJointAng >= theta11[pathIndex] + maxAngleMargin )&& ! (BJointAng <= theta21[pathIndex] -maxAngleMargin) ) {
      pathIndex +=1;
      Serial.println("pathIndex has changed: path 2");
      AJointSpd = 195;
      BJointSpd = 195;
    }
  }*/
  //while ( millis() < St + 50*1);  // wait for a while  !NEEDS TO GO

  if (pathIndex <= 9){
    if ((AJointAng > theta11[pathIndex] - maxAngleMargin)) {
      pathIndex += 1;
      //Serial.println("pathIndex has changed: path 1A" + String (theta11[pathIndex]));
    }
  }

  else if (pathIndex > 9){
     if ((AJointAng < theta11[pathIndex] + 3*maxAngleMargin)) {
      pathIndex += 1;
      //Serial.println("pathIndex has changed: path 2A" + String (theta11[pathIndex]));
    }
    else if (pathIndex ==14){
      //pathIndex = 0;
    }
  }
  

  //pathIndex += 1;
  
  //pathIndex = pathIndex%15;      // sets the counter to zero after a cycle has been completed
  pathIndex2 = pathIndex2%15;

  
/*
  if (pathIndex == 14 && pathIndex2 == 14){
    motionEnabled = 0;
    //BJointSpd = 0;
    //AJointSpd = 0;
    pathIndex = 0;
    pathIndex2 = 0;
    analogWrite(APWM_PIN, AJointSpd);
    analogWrite(BPWM_PIN, BJointSpd);
  
  }
*/
  AJointSetpoint = theta11[pathIndex];
  BJointSetpoint = theta21[pathIndex2];
  //AJointSetpoint = constrain(AJointSetpoint, previousAJointAng-maxAngleMargin, previousAJointAng+maxAngleMargin);
  //BJointSetpoint = constrain(BJointSetpoint, previousBJointAng-maxAngleMargin, previousBJointAng+maxAngleMargin);

    /*
    Serial.print("AJointSetpoint : ");
    Serial.println(AJointSetpoint);
    Serial.print("BJointSetpoint : ");
    Serial.println(BJointSetpoint);
    
    */
}


//------------------------------speedControl --------------------------------

void speedControl(){
  
  if(AJointAng > AJointSetpoint) digitalWrite(AEN_PIN, LOW);  //  change the direction of the rotation
  if(AJointAng < AJointSetpoint) digitalWrite(AEN_PIN, HIGH);
  if(BJointAng > BJointSetpoint) digitalWrite(BEN_PIN, !LOW);  //  change the direction of the rotation
  if(BJointAng < BJointSetpoint) digitalWrite(BEN_PIN, !HIGH);

  //MotorAPID.Compute();
  //MotorBPID.Compute();

  
  if (pathIndex == 10) AJointSpd = 255;
  else{
  AJointSpd = abs(AJointSpd) +175;
  }
  if ( BJointSpd == -1 ){
      
  }
  //if ( pathIndex >=10)(BJointSpd = 175);
  
  else BJointSpd = (BJointSpd) +180;

  //Serial.print("The value of the speed is" + String(BJointSpd));

  analogWrite(APWM_PIN, AJointSpd);
  analogWrite(BPWM_PIN, BJointSpd);
  /*
  Serial.print("ASPD:");
  Serial.print(AJointSpd);
  Serial.print("    BSPD:");
  Serial.println(BJointSpd);
*/

  previousAJointAng = AJointAng;
  previousBJointAng = BJointAng;
}



void loop() {
  // put your main code here, to run repeatedly:
  //checkEnable();
  

  //while(Serial.available()==0);

  Serial.println("Loop: start");
  while (motionEnabled){
 
  //dropOut();
  //getPosition();
  //computation();
  setPosControl();
  speedControl();

  // for plotting purposes

  //Serial.print (AJointAng);
  //Serial.print(" ");
  //Serial.println(AJointSetpoint);
  //Serial.print(" ");
  /*Serial.print(AJointAng);
  Serial.print(" ");
  Serial.println(AJointSetpoint);
  Serial.print(" ");*/

  if (LED_status) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
  LED_status = !LED_status;

  /*Serial.print("DEG1:");
  Serial.print(AJointAng);
  Serial.print("  DEG2:");
  Serial.print(BJointAng);
  Serial.print("  ASPD:");
  Serial.print(AJointSpd);
  Serial.print("  BSPD:");
  Serial.print(BJointSpd);
  Serial.print("  ASP:");
  Serial.print(AJointSetpoint);
  Serial.print("  BSP:");
  Serial.println(BJointSetpoint);*/
  }

}
