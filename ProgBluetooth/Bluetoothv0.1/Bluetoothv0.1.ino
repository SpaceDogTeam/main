#include <SoftwareSerial.h>
#include <EasyTransfer.h>

#define R_PIN 5
#define G_PIN 6
#define B_PIN 7

#define RX_PIN 10
#define TX_PIN 11

SoftwareSerial BT_Serial(RX_PIN, TX_PIN); //RX|TX
EasyTransfer ETin, ETout; 

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;


void setup() {
  pinMode(R_PIN, OUTPUT); 
  pinMode(G_PIN, OUTPUT); 
  pinMode(B_PIN, OUTPUT); 
  Serial.begin(9600);
  BT_Serial.begin(9600);
  ETin.begin(details(rxdata), &BT_Serial);
  ETout.begin(details(txdata), &BT_Serial);
  Serial.println("Starting");
}
 
void loop() {

  //ETout.sendData();
  //there's a loop here so that we run the recieve function more often then the 
  //transmit function. This is important due to the slight differences in 
  //the clock speed of different Arduinos. If we didn't do this, messages 
  //would build up in the buffer and appear to cause a delay.
  for(int i=0; i<5; i++){
    ETin.receiveData();
    analogWrite(R_PIN, rxdata.red);
    analogWrite(G_PIN, rxdata.green);
    analogWrite(B_PIN, rxdata.blue);
    delay(10);
  }
}
