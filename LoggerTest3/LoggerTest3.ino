#include <SPI.h>
#include <PID_v1.h>
#include <Math.h>


//------------------------------------------------------------------------
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

bool motionEnabled = !false;

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

int tLength = 15;

//------------------------------------    PID control variables --------------------------------------

//uint16_t maxAngleMargin = 0.5;
double maxAngleMargin = 1;
double AJointSpd, BJointSpd;  // output variables to themotor
double AJointAng, BJointAng;  // actual angular position of the joints received from the encoders
double AJointSetpoint, BJointSetpoint, AJointSetpointOld, BJointSetpointOld;
double previousAJointAng = AJointSetpoint;
double previousBJointAng = BJointSetpoint;

// Tuning parameters
const int sampleRate = 10; //  sampling rate

int k1 = 0.8*10 , k2 = 1, k3 = 1;

double KpA=k1*3,  KpB=1;  //Initial Proportional Gain 
double KiA=k3*10/10,  KiB=5;  //Initial Integral Gain 
double KdA=k2*5,  KdB=6;  //Initial Differential Gain 


// Instantiate X and Y axis PID controls.
PID MotorAPID(&AJointAng, &AJointSpd, &AJointSetpoint, KpA, KiA, KdA, DIRECT); 
PID MotorBPID(&BJointAng, &BJointSpd, &BJointSetpoint, KpB, KiB, KdB, REVERSE); //setting up the PID

const float Pi = 3.14159;
int buffersize = 1000;
int bufferindex = 0;
double bufferres [1000];
int cur_iter = 0;
bool running = true;

double mapfloat(double x, double in_min, double in_max, double out_min, double out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
  
  pinMode(APWM_PIN, OUTPUT);
  pinMode(AEN_PIN, OUTPUT);

  pinMode(BPWM_PIN, OUTPUT);
  pinMode(BEN_PIN, OUTPUT);

  DDRG = DDRG | B00100000;

  Serial.println("Dual Motor Leg Test : Start!");
  
  s = millis();
  
  //-------------------------------------------------------------------------
  MotorAPID.SetMode(AUTOMATIC);
  MotorAPID.SetSampleTime(sampleRate);
  MotorAPID.SetOutputLimits(0,75);
  
  MotorBPID.SetMode(AUTOMATIC);
  MotorBPID.SetSampleTime(sampleRate);
  MotorBPID.SetOutputLimits(0,75);

  AJointSetpoint = 306;
  BJointSetpoint = 94;

  while(millis() < s + 100)
  {
    unsigned long looptimer = millis();
    getPosition();
    while(millis() < looptimer + 10);
    bufferres[bufferindex] = BJointAng;
    bufferindex++;
  }

  AJointSetpoint = 306;
  BJointSetpoint = 84;
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

     deg1 = mapfloat(deg1,0,360,360,0);

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

     deg2 = mapfloat(deg2,360.0,0.0,0.0,360.0);

   }else{
    //Serial.println("Encoder #2 value received unchanged ! "); 
   }

   
   AJointAng = deg1 ;
   BJointAng = deg2 ;
}


//----------------------------------- SetPosControl ----------------------------------


void setPosControl(){
  AJointSetpoint = 306;
  BJointSetpoint = 84;
}


//------------------------------speedControl --------------------------------

void speedControl(){
  
  if(AJointAng > AJointSetpoint) digitalWrite(AEN_PIN, LOW);  //  change the direction of the rotation
  if(AJointAng < AJointSetpoint) digitalWrite(AEN_PIN, HIGH);
  if(BJointAng > BJointSetpoint) digitalWrite(BEN_PIN, !LOW);  //  change the direction of the rotation
  if(BJointAng < BJointSetpoint) digitalWrite(BEN_PIN, !HIGH);

  MotorAPID.Compute();
  MotorBPID.Compute();

  AJointSpd = abs(AJointSpd) +175;
  if ( BJointSetpoint - maxAngleMargin < BJointAng and BJointAng < BJointSetpoint + maxAngleMargin){
      BJointSpd = 0;
  } 
  else BJointSpd = (BJointSpd) +180;

  analogWrite(APWM_PIN, AJointSpd);
  analogWrite(BPWM_PIN, BJointSpd);

  previousAJointAng = AJointAng;
  previousBJointAng = BJointAng;
}

void loop() {
  while(running & bufferindex < buffersize){
      unsigned long looptimer = millis();
      getPosition();
      //setPosControl();
      speedControl();

      //PORTG = PORTG ^ B00100000;
      while(millis() < looptimer + 10);
      bufferres[bufferindex] = BJointAng;
      bufferindex+=1;
  }

  Serial.println("MEASUREMENTS");
  for(int i=0; i<buffersize; i+=1)
  {
    Serial.println(String(bufferres[i]));
  }

  Serial.println("END MEASUREMENTS");
  while(true);
}
