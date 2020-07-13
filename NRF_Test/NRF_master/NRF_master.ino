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
float x = 0;
void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
void loop() {
  for(int i=0; i<60; i++){
    x += 0.5;
    radio.write(&x, sizeof(x));
    delay(100);
  }
  for(int i=59; i>0; i--){
    x -= 0.5;
    radio.write(&x, sizeof(x));
    delay(100);
  }
}
