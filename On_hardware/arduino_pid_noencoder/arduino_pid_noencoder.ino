
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
//pretty good value => Kp=2,Ki=0,Kd=0.3
//better value => Kp=2,Ki=0,Kd=0.03/0.04

float Kp = 3.1;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 0.07;   
double Setpoint = 90, Input, Output;                                       
float baseSpeed = 70;
float cm = 90;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.


void setup() {

  Serial.begin(9600); //Attach Servo
  delay(5000);
  motorInitialize();
  Input = 90; 
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-255 + baseSpeed, 255 - baseSpeed);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
  if(Serial.available()){
    //    myPID.SetTunings(Kp, Ki, Kd); 
    Input = readSerialData();
    myPID.Compute();  
  //  Serial.println(Input);
  //  Serial.println(Output);
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
  
  digitalWrite(L298N_A, HIGH);
  digitalWrite(L298N_B, LOW);
  digitalWrite(L298N_C, HIGH);
  digitalWrite(L298N_D, LOW);
  if(velocity >= 0){
    analogWrite(L298N_EN, baseSpeed);
    analogWrite(L298N_EN_B, baseSpeed + abs(velocity));
  }
  else{
    analogWrite(L298N_EN, baseSpeed + abs(velocity));
    analogWrite(L298N_EN_B, baseSpeed);
  }
}     

float readSerialData(){
  for(int i=0; i<4; i++){
    r = (Serial.readStringUntil('\t'));  //conveting the value of chars to integer
    data[i] = r.toFloat();
//      Serial.print(data[i]);
//      Serial.print('\t');
  }
  cm = data[0];
//    Kp = data[1];
//    Ki = data[2];
//    Kd = data[3];
//    Serial.println(Input);
  return cm;
}
