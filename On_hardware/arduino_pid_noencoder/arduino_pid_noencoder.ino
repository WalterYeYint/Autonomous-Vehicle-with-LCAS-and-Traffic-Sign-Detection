
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.

#include <PID_v1.h>

//PID ideal values
//value 1 ==> Kp = 6.5, Ki = 0, Kd = 1.1

#define L298N_A       11   //A pin of motor driver (L298N)
#define L298N_B       10   //B pin of motor driver (L298N)
#define L298N_EN      6  //EN pin of motor driver (L298N)
#define L298N_C       9   //A pin of motor driver (L298N)
#define L298N_D       8   //B pin of motor driver (L298N)
#define L298N_EN_B      5   //EN_B pin of motor driver (L298N)



String r;
float data[4];

//float Kp = 15.1;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
//float Ki = 0.15;                                                      //Initial Integral Gain
//float Kd = 0.05;  //Intitial Derivative Gain
float Kp = 0;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 0;   
double Setpoint = 90, Input, Output;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.


void setup() {

  Serial.begin(9600); //Attach Servo
  delay(5000);
  motorInitialize();
  Input = 90; 
  readSerialData();
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-100, 100);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
  
//    myPID.SetTunings(Kp, Ki, Kd); 
  readSerialData();
  myPID.Compute();  
  Serial.println(Output);
  motorMove(Output);     
//  Serial.print(" "); 
//  Serial.println(Output);
//  Serial.println("");
//    Serial.print(Kp);
//    Serial.print(" ");
//    Serial.print(Ki);
//    Serial.print(" ");
//    Serial.println(Kd);
}

void motorInitialize(){
  pinMode (L298N_A, OUTPUT);
  pinMode (L298N_B, OUTPUT);
  pinMode (L298N_C, OUTPUT);
  pinMode (L298N_D, OUTPUT);
  digitalWrite(L298N_A, 0);
  digitalWrite(L298N_B, 0);
  analogWrite(L298N_EN, 0); 
  digitalWrite(L298N_C, 0);
  digitalWrite(L298N_D, 0);
  analogWrite(L298N_EN_B, 0);  
} 

void motorMove(int velocity){
  if (velocity > 255) velocity = 255;
  else if (velocity < -255) velocity = -255;
  
  digitalWrite(L298N_A, LOW);
  digitalWrite(L298N_B, HIGH);
  digitalWrite(L298N_C, LOW);
  digitalWrite(L298N_D, HIGH);
  if(velocity >= 0){
    analogWrite(L298N_EN, abs(255 - velocity));
    analogWrite(L298N_EN_B, 255);
  }
  else{
    analogWrite(L298N_EN, 255);
    analogWrite(L298N_EN_B, abs(255 - velocity));
  }
}     

void readSerialData(){
  if(Serial.available()){ 
    for(int i=0; i<4; i++){
      r = (Serial.readStringUntil('\t'));  //conveting the value of chars to integer
      data[i] = r.toFloat();
//      Serial.print(data[i]);
//      Serial.print('\t');
    }
    Input = data[0];
  }
}

