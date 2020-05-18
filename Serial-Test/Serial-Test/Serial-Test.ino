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
int data[3];
int setpoint = 0;
void setup(){
  Serial.begin(9600);
  delay(5000);
}
void loop(){
  if(Serial.available()){         //From RPi to Arduino
    for(int i=0; i<3; i++){
      r = (Serial.readStringUntil('\t'));  //conveting the value of chars to integer
      data[i] = r.toInt();
      Serial.print(data[i]);
      Serial.print('\t');
    }
    printfunc();
  }
  else{
    for(int i=0; i<3; i++){
      Serial.print(data[i]);
      Serial.print('\t');  
    }
    printfunc();
  }
}

void printfunc(){
  Serial.println();
  if(data[0] > setpoint){
    Serial.print(data[0]);
    Serial.print(" is bigger than ");
    Serial.println(setpoint);
  }
  else{
    Serial.print(data[0]);
    Serial.print(" is less than ");
    Serial.println(setpoint);
  }
}
