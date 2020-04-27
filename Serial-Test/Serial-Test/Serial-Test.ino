//int data;
//void setup() {
//  Serial.begin(9600);
//}
//void loop() {
//  if (Serial.available() > 0) {
//    for (int i=0; i<3; i++) {
//      data = Serial.readStringUntil('\t');
//      Serial.print(data);
//      Serial.print('\t');
//    }
//    delay(1000);
//  }
//}



String r;
void setup(){
  Serial.begin(9600);
}
void loop(){
  if(Serial.available()){         //From RPi to Arduino
    for(int i=0; i<3; i++){
      r = (Serial.readStringUntil('\t'));  //conveting the value of chars to integer
      Serial.print(r.toInt());
      Serial.print('\t');
    }
    Serial.println();
  }
}
