
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.

#include <PID_v1.h>

unsigned int MAX_DISTANCE = 400;

#define L298N_A       11   //A pin of motor driver (L298N)
#define L298N_B       10   //B pin of motor driver (L298N)
#define L298N_EN      6  //EN pin of motor driver (L298N)
#define L298N_C       9   //A pin of motor driver (L298N)
#define L298N_D       8   //B pin of motor driver (L298N)
#define L298N_EN_B      5   //EN_B pin of motor driver (L298N)

//PID ideal values
//value 1 ==> Kp = 6.5, Ki = 0, Kd = 1.1

long duration, cm;
float Kp = 15.1;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki = 0.15;                                                      //Initial Integral Gain
float Kd = 0.05;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output;                                       

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.


void setup() {

  Serial.begin(9600); //Attach Servo
  delay(5000);
  motorInitialize();      //Initialize the motor
  Input = readPosition();       
 
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-150, 150);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
 
  Setpoint = 30;
  Input = readPosition();                                            
  if(cm > 400 || cm < 1)
    Input = 400;
  myPID.Compute();
  Serial.print(cm);
  Serial.print("\t");     
  Serial.println(Output);
  motorMove(Output);                              
  
  
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
  
  digitalWrite(L298N_A, velocity > 0 ? LOW : HIGH);
  digitalWrite(L298N_B, velocity > 0 ? HIGH : LOW);
  digitalWrite(L298N_C, velocity > 0 ? LOW : HIGH);
  digitalWrite(L298N_D, velocity > 0 ? HIGH : LOW);
  analogWrite(L298N_EN, abs(velocity));
  analogWrite(L298N_EN_B, abs(velocity));
}

float readPosition() {
  delay(40);        
  //Insert the desired position data from camera here
  return cm;  //Returns distance value.
}
