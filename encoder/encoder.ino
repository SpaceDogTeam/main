/* Spacedog - Leg prototype
* Command the leg and retrieve the angular position of its joints
*/


#include <SPI.h>

#define CS1 48 //Chip or Slave select  #1
#define CS2 46 //Chip or Slave select  #2

#define WAITING_MS 20 // time left by the master between reads

#define WAITING_DS 100 // time left by the master between reads

unsigned long Dt = 0;
unsigned long Mt = 0;

uint16_t ABSposition1 = 0;  //  stores the Absolute position to be read from the encoder
uint16_t ABSposition_last1 = 0; //  stores the last know position
uint16_t ABSposition2 = 0;
uint16_t ABSposition_last2 = 0;

uint8_t temp1[1]; // contains the MSB of the position
uint8_t temp2[1]; //  contains the LSB of the position

float deg1 = 0.00;
float deg2 = 0.00;

int buffersize = 10;
int bufferindex = 0;
float ResBuffer[10];

void setup()
{
  pinMode(CS1,OUTPUT);//Slave#1 configuration 
  digitalWrite(CS1,HIGH);  //  Slave#1 deSelected
  pinMode(CS2,OUTPUT);//Slave#2 configuration 
  digitalWrite(CS2,HIGH);  //  Slave#2 deSelected
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  Serial.begin(115200);
  Serial.println("starting");
  Serial.flush();
  Dt = millis();
  
  while(millis() < Dt+WAITING_DS);
  Dt = millis();
  SPI.end();
  Mt = micros();
}


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


void loop()
{ 
   uint8_t received = 0xA5;    //just a temp variable that the encoder will send if he's not ready of there isn't any remaining data to be sent
   ABSposition1 = 0;    //reset position variable
   ABSposition2 = 0;    //reset position variable
   
   SPI.begin();    //start transmition
   
   SPI_T(1,0x10);   //issue read command to encoder #1
   received = SPI_T(1,0x00);    //issue NOP to check if encoder is ready to send
   while (received != 0x10)    //loop while encoder is not ready to send 
   {
     received = SPI_T(1,0x00);    //check again if both encoders are still working 
     //Serial.println("Encoder #1 not ready ! "); 
     delay(2);    //wait a bit
   }
   
   temp1[0] = SPI_T(1,0x00);    //Receive MSB
   temp1[1] = SPI_T(1,0x00);    // receive LSB
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
     deg1 = map(deg1,0,360,360,0) +23;
     Serial.println(deg1);     //send position in degrees
   }else{
    Serial.print("Encoder #1 value received unchanged !        "); 
    Serial.println(deg1);
   }
   if (ABSposition2 != ABSposition_last2)
   {
     ABSposition_last2 = ABSposition2;
     deg2 = ABSposition2;
     deg2 = deg2 * 0.08789;
     deg2 = map (deg2,360,0,0,360)-243 ;
     Serial.println(deg2);
   }else{
    Serial.print("Encoder #2 value received unchanged !      ");
    Serial.println(deg2);
   }
   ResBuffer[bufferindex] = deg2;
   delay(1000);    //wait a bit till next check

   if(bufferindex >= buffersize - 1)
   {
     Serial.println("Logging complete");

     for(int i=0; i<buffersize; i++){
       Serial.println(ResBuffer[i]);
     }

     while(true){};
   }
   bufferindex += 1;
   Serial.println("Buffer currently storing " + String(bufferindex) + " items");
}

// float Cmd1[] = {36.39,35.77,43.41,47.55,48.69,49.31,47.46,44.12,39.02,34.10,30.76,26.98,22.59,18.19,15.03,14.41}
// float Cmd2[] = {283.05,283.01,274.48,268.94,266.13,262.53,260.77,260.15,261.12,264.55,267.36,271.4,277.56,283.80,290.04,292.05}
