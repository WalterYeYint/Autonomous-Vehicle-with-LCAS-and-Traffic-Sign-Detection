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
float data[4];
float setpoint = 0;
float Input = 90;
float Kp = 0;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 0; 
void setup(){
  Serial.begin(9600);
  delay(5000);
}
void loop(){
  Input = readSerialData();
}

float readSerialData(){
  float Input = 0;
  if(Serial.available()){
    for(int i=0; i<4; i++){
      r = (Serial.readStringUntil('\t'));  //conveting the value of chars to integer
      data[i] = r.toFloat();
//      Serial.print(data[i]);
//      Serial.print('\t');
    }
    Input = data[0];
//    Kp = data[1];
//    Ki = data[2];
//    Kd = data[3];
    Serial.println(Input);
  }
  return Input;
}
