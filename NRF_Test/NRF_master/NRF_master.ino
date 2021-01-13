/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
int pinP = 0;
int pinD = 1;
double Kp_R = 0;
double Ki_R = 0;
double temp_arr[3];
void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  Kp_R = analogRead(pinP);
  Ki_R = analogRead(pinD);
  temp_arr[0] = 150;
  temp_arr[1] = 150;
  temp_arr[2] = 1;
  radio.write(&temp_arr, sizeof(temp_arr));
}
