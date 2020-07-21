/*
  Arduino Wireless Network - Multiple NRF24L01 Tutorial
        == Node 02 (Child of Master node 00) ==
*/
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>
#define led 2
RF24 radio(7, 8);               // nRF24L01 (CE,CSN)
RF24Network network(radio);      // Include the radio in the network
const uint16_t this_node = 01;   // Address of our node in Octal format ( 04,031, etc)
const uint16_t master00 = 00;    // Address of the other node in Octal format
const uint16_t node02 = 02;    // Address of the other node in Octal forma
double data[] = {1.11, 2.22, 3.33};
void setup() {
  SPI.begin();
  radio.begin();
  network.begin(90, this_node); //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
}
void loop() {
  network.update();
  //===== Receiving =====//
  RF24NetworkHeader header(master00);
  RF24NetworkHeader header2(node02);
  for(int i=0; i<20; i++){
    data[0] = i;
    bool ok = network.write(header, &data, sizeof(data)); // Send the data
    bool ok2 = network.write(header2, &data, sizeof(data)); // Send the data
  }
  for(int i=20; i>0; i--){
    data[0] = i;
    bool ok = network.write(header, &data, sizeof(data)); // Send the data
    bool ok2 = network.write(header2, &data, sizeof(data)); // Send the data
  }
  
}
