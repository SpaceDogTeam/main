// Programm for each arduino of the SpaceDog Project from The SPACE by Andrea Ventura
// Each arduino control 1 leg
// The programm is the same for each leg, EXCEPT:
// !!!!!! In the PID creation, motor A is direct and B reverse
// !!!!!! for the front left and back right leg
// !!!!!! and the opposite for the front right and back left legs
// !!!!!! because of the way the encoders for angle mesurments are mounted

#include <SPI.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <PID_v1.h>
//#include <Math.h>
//#include <Wire.h>

//----------------leg----------------------------
#define A_PWM_PIN  5       // Speed of motor A and B in PWM units, value from 0 to 255
#define A_EN_PIN   6       // Motor A is top motor B is botom

#define B_PWM_PIN  7      // This sets the direction of motor A and B, set to HIGH for outward motion and LOW for inward motion
#define B_EN_PIN   8

//----------------Encoders-----------------------
#define A_CS_PIN 48 //Chip or Slave select Angle A
#define B_CS_PIN 46 //Chip or Slave select Angle B

#define WAITING_MS 20 // time left by the master between reads
#define WAITING_DS 150 

//----------------Angles Mesurments and calculation variables--------

uint8_t temp[2]; // contains the MSB, LSB of the position

uint16_t ABSposition = 0;  //  stores the Absolute position to be read from the encoder

int motorPwmPinsArray[2] = {A_PWM_PIN, B_PWM_PIN};
int motorEnablePinsArray[2] = {A_EN_PIN, B_EN_PIN};
int encoderCSPinsArray[2]={A_CS_PIN, B_CS_PIN};

unsigned long Dt = 0;
unsigned long Mt = 0;

bool flagZero = 0;


double legsAngles[2] = {0, 0};   //A, B
double legsAnglesPrevious[2] = {0, 0};    // memory of previous angle value mesured

double legsAnglesZero[2] = {0, 0};
double anglesSetpointsArray[2] = {0, 0};
double setpointMargin = 1;  // angle margin when reaching new position (total margin is twice that amount)
double deltaError[2] = {0,0};
double pwmValue[2] = {0,0};
int encoderNtours[2] = {0,0};

int tryErrorAngle[2] = {0,0};

int kP = 100;

//--------------------PID Lib variables--------------------------

PID myPIDmotorA(&legsAngles[0],&pwmValue[0],&anglesSetpointsArray[0],100,0,0,DIRECT);
PID myPIDmotorB(&legsAngles[1],&pwmValue[1],&anglesSetpointsArray[1],100,0,0,REVERSE);
//---------------------------------------------------------------
//---------------data and transmissions variables----------------
int val1, val2;
double val3;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use by strtok() function

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};

boolean newData = false;
//---------------------------------------------------------------

void setup() 
{
  // put your setup code here, to run once:
  pinMode(A_CS_PIN,OUTPUT);//Slaves encoders configuration 
  pinMode(B_CS_PIN,OUTPUT);

  pinMode(A_PWM_PIN, OUTPUT);
  pinMode(A_EN_PIN, OUTPUT);
  pinMode(B_PWM_PIN, OUTPUT);
  pinMode(B_EN_PIN, OUTPUT);
  
  digitalWrite(A_CS_PIN,HIGH);  //Slaves encoders de-selected
  digitalWrite(B_CS_PIN,HIGH);

  

  //------initialize Timer3 for measure and Timer4 for PID--------
  
  //visit http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
  //for a great little tutorial about timer interrupts
  //most of the basic interupt routine used here comes from there
  
  
  cli();  // disable global interrupts
  
  TCCR3A = 0; // set entire TCCR3A register to 0
  TCCR3B = 0; // same for TCCR3B

  TCNT3 = 2500;            //offset the timer by 10ms
  // set compare match register to desired timer count:
  OCR3A = 5000;            //5000 for 20ms timer with 64 prescaler
  // turn on CTC mode:
  TCCR3B |= (1 << WGM32);
  // Set CS30 and 31 bit for 64 prescaler:
  TCCR3B |= (1 << CS30);
  TCCR3B |= (1 << CS31);
  // enable timer compare interrupt:
  TIMSK3 |= (1 << OCIE3A);
  
  //------------------------------------------------------------------
  
  TCCR4A = 0; // set entire TCCR4A register to 0
  TCCR4B = 0; // same for TCCR4B

  // set compare match register to desired timer count:
  OCR4A = 5000;            //5000 for 20ms timer with 64 prescaler
  // turn on CTC mode:
  TCCR4B |= (1 << WGM42);
  // Set CS40 and 41 bit for 64 prescaler:
  TCCR4B |= (1 << CS40);
  TCCR4B |= (1 << CS41);
  // enable timer compare interrupt:
  TIMSK4 |= (1 << OCIE4A);
  
  sei();  // enable global interrupts
  
  
  //--------------------End initialize Timer4--------------------------
  //----------Initialize SPI comm for encoders mesurments--------------
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  SPI.end();

  
  //---------------------------End SPI---------------------------------
  //-----------------------PID Library setup---------------------------
  
  myPIDmotorA.SetOutputLimits(-255, 255);
  myPIDmotorA.SetSampleTime(0);
  myPIDmotorB.SetOutputLimits(-255, 255);
  myPIDmotorB.SetSampleTime(0);
  
  //-------------------------------------------------------------------
  Serial.begin(9600);
  Serial.flush();
}

ISR(TIMER3_COMPA_vect)  //function executed when timer3 interrupt
{                       //Angular position mesurments
  getPosition(0);
  getPosition(1);
} 

ISR(TIMER4_COMPA_vect)  //function executed when timer4 interrupt
{                       //PID computation
  
  myPIDmotorA.Compute();
  moveMotor(0);
  myPIDmotorB.Compute();
  moveMotor(1);
}

double mapfloat(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t SPI_T (int index, uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //  variable to hold received data
   digitalWrite(encoderCSPinsArray[index],LOW);  //  select spi device 
   msg_temp = SPI.transfer(msg); // sends (receive) message to (from) the encoder #index
   digitalWrite(encoderCSPinsArray[index],HIGH); //  deselect spi device
   delayMicroseconds(10);   //delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
   return (msg_temp);    //return received byte
}

void getPosition(int index){
  int tryAmount = 0;
  uint8_t received = 0xA5;    //just a temp variable that the encoder will send if he's not ready of there isn't any remaining data to be sent
  uint16_t ABSposition = 0;  //  stores the Absolute position to be read from the encoder

  SPI.begin();    //start transmition
  
  SPI_T(index,0x10);   //issue read command to encoder #1
  received = SPI_T(index,0x00);    //issue NOP to check if encoder is ready to send
  while (received != 0x10 and tryAmount++ < 5)    //loop while encoder is not ready to send 
  {
    received = SPI_T(index,0x00);    //check again if both encoders are still working 
  }
  
  temp[0] = SPI_T(index,0x00);    //Receive MSB
  temp[1] = SPI_T(index,0x00);    //Receive LSB
  temp[0] &=~ 0xF0;    //mask out the first 4 bits
  
  SPI.end();    //end transmition
  
  ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
  ABSposition += temp[1];    // add LSB to ABSposition message to complete message

  legsAngles[index] = (ABSposition * 0.08789)+encoderNtours[index]*360; // approx 360/4096
  //--------------Angle Roll-Over Managements-----------
  if (legsAnglesPrevious[index]-(encoderNtours[index]*360) > 355 and legsAngles[index]-(encoderNtours[index]*360) < 5){
    legsAngles[index] += 360;
    encoderNtours[index] += 1;
  }
  else if (legsAnglesPrevious[index]-(encoderNtours[index]*360) < 5 and legsAngles[index]-(encoderNtours[index]*360) > 355){
    legsAngles[index] -= 360;
    encoderNtours[index] -= 1;
  }
  legsAnglesPrevious[index] = legsAngles[index];
}

void IncrSetPoint(int choiceMotor, double choiceAngle){
  if (flagZero){
    anglesSetpointsArray[choiceMotor] += choiceAngle;
  }
}

void abslSetPoint(int choiceMotor, double choiceAngle){
  if (flagZero){
    anglesSetpointsArray[choiceMotor] = legsAnglesZero[choiceMotor] + choiceAngle;
  }
}

void setZero(){
  for (int i = 0; i<=1; i++){
     getPosition(i);
     legsAnglesZero[i] = legsAngles[i];
     anglesSetpointsArray[i] = legsAngles[i];
     
     //Serial.println(":zero set for "+String(i)+" -> "+String(legsAnglesZero[i])+";");
  }
  flagZero = 1;
  myPIDmotorA.SetMode(AUTOMATIC);
  myPIDmotorB.SetMode(AUTOMATIC);
}

void moveMotor(int index){    //apply the pwm calculated by the PID to the motor
  if (pwmValue[index] > 165){
    digitalWrite(motorEnablePinsArray[index], 1);
    analogWrite(motorPwmPinsArray[index], pwmValue[index]);
  }
  else if (pwmValue[index] < -165){
    digitalWrite(motorEnablePinsArray[index], 0);
    analogWrite(motorPwmPinsArray[index], -pwmValue[index]);
  }
  else {
    analogWrite(motorPwmPinsArray[index], 0);
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
  
  strtokIndx = strtok(tempChars,",");     
  val1 = atoi(strtokIndx);             // convert this part to an integer,    
                                       //can be to a float too with atof?
  strtokIndx = strtok(NULL, ","); 
  val2 = atoi(strtokIndx);     
                               
  strtokIndx = strtok(NULL, ",");
  val3 = atof(strtokIndx);             // convert this part to a float
}

void interpretData() {
  transmission(0);
  if (val1 == 0){
    setZero();
  }
  else if (val1 == 1){
    IncrSetPoint(val2, val3);
  }
  else if (val1 == 2){
    abslSetPoint(val2, val3);
  }
  else if (val1 == 3){
    transmission(1);
  }
}

void reception(){
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() replaces the commas with \0
    parseData();
    //showParsedData();
    interpretData();
    newData = false;
  }
}

void transmission(int flag){
  char  buff[20];
  if (flag == 0){
    char  s0[20];
    dtostrf(val3,3,2,s0);
    sprintf(buff,":ack <%d,%d,%s>;",val1,val2,s0);
    Serial.flush();
    Serial.println(buff);
  }
  else if (flag == 1){
    char  s0[20];
    char  s1[20];
    dtostrf(legsAngles[0],3,2,s0);
    dtostrf(legsAngles[1],3,2,s1);
    sprintf(buff,":pos is %s,%s;",s0,s1);
    Serial.flush();
    Serial.println(buff);
  }
}

void loop() {
  reception();
}
