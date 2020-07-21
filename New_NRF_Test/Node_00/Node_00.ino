/*
  Arduino Wireless Network - Multiple NRF24L01 Tutorial
          == Base/ Master Node 00==
  by Dejan, www.HowToMechatronics.com
  Libraries:
  nRF24/RF24, https://github.com/nRF24/RF24
  nRF24/RF24Network, https://github.com/nRF24/RF24Network
*/
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
RF24 radio(7, 8);               // nRF24L01 (CE,CSN)
RF24Network network(radio);      // Include the radio in the network
const uint16_t this_node = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t node01 = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
double incomingData[3];
void setup() {
  Serial.begin(9600);
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
}
void loop() {
  network.update();
  //===== Receiving =====//
  while ( network.available() ) {     // Is there any incoming data?
    RF24NetworkHeader header;
    network.read(header, &incomingData, sizeof(incomingData)); // Read the incoming data
    
    if (header.from_node == 1) {    // If data comes from Node 02
      Serial.print("From Node01 -- ");
      Serial.print(incomingData[0]);
      Serial.print("  ");
      Serial.print(incomingData[1]);
      Serial.print("  ");
      Serial.println(incomingData[2]);
    }
    if (header.from_node == 2) {    // If data comes from Node 012
      Serial.print("From Node02 -- ");
      Serial.print(incomingData[0]);
      Serial.print("  ");
      Serial.print(incomingData[1]);
      Serial.print("  ");
      Serial.println(incomingData[2]);
    }
  }
}
