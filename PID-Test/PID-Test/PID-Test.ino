
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.

#include <PID_v1.h>

//PID ideal values
//value 1 ==> Kp = 6.5, Ki = 0, Kd = 1.1

String r;
float data[3];

long duration, cm = 0;
//float Kp = 15.1;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
//float Ki = 0.15;                                                      //Initial Integral Gain
//float Kd = 0.05;  //Intitial Derivative Gain
float Kp = 0;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 0;   
double Setpoint, Input, Output;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.


void setup() {

  Serial.begin(9600); //Attach Servo
  delay(5000);
  Input = cm;       
 
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(0, 100);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
  if(Serial.available()){ 
    for(int i=0; i<3; i++){
      r = (Serial.readStringUntil('\t'));  //conveting the value of chars to integer
      data[i] = r.toFloat();
//      Serial.print(data[i]);
//      Serial.print('\t');
    }
    Kp = data[0];
    Ki = data[1];
    Kd = data[2];
    myPID.SetTunings(Kp, Ki, Kd);
  }
  Setpoint = 100;
  Input = cm;     
  myPID.Compute();
//  Serial.print(cm);
//  Serial.print(" ");     
//  Serial.print(Setpoint);
//  Serial.print(" "); 
//  Serial.print(Output);
//  Serial.println("");
    Serial.print(Kp);
    Serial.print(" ");
    Serial.print(Ki);
    Serial.print(" ");
    Serial.println(Kd);
  cm = cm + Output;                              
  
  
}
