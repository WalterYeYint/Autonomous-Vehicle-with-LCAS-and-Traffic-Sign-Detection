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



int r = 1;
void setup(){
  Serial.begin(9600);
}
void loop(){
  if(Serial.available()){         //From RPi to Arduino
    r = r * (Serial.read() - '0');  //conveting the value of chars to integer
    Serial.println(r);
  }
}
