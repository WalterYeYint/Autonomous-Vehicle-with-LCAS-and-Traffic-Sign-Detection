/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
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
double temp_arr[3]; 

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if (radio.available()) {
    radio.read(&temp_arr, sizeof(temp_arr));
    Serial.print(temp_arr[0]);
    Serial.print(" ");
//    Serial.print(70);
//    Serial.print(" ");
//    Serial.print(50);
//    Serial.print(" ");
    Serial.print(temp_arr[1]);
    Serial.print(" ");
    Serial.println(temp_arr[2]);
  }
}
