
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.

#include <PID_v1.h>

//PID ideal values
//value 1 ==> Kp = 6.5, Ki = 0, Kd = 1.1

#define L298N_A       11   //A pin of motor driver (L298N)
#define L298N_B       10   //B pin of motor driver (L298N)
#define L298N_EN      6  //EN pin (Left wheel) of motor driver (L298N)
#define L298N_C       9   //A pin of motor driver (L298N)
#define L298N_D       8   //B pin of motor driver (L298N)
#define L298N_EN_B      5   //EN_B (Right wheel) pin of motor driver (L298N)



String r;
float data[4];
//pretty good value => Kp=0.62,Ki=0,Kd=0.37
//better value => Kp=0.47,Ki=0,Kd=0.34

float Kp = 0.47;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 0.34;   

float pinP = 0;    //pin Analog 0 for the input of the potentiometer
float pinD = 1; 

double Setpoint = 90, Input, Output;   
int maxspeed = 150;                                    
int baseSpeed = 60;
float cm = 90;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.


void setup() {

  Serial.begin(9600); //Attach Servo
  delay(5000);
  motorInitialize();
  Input = 90; 
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-maxspeed, maxspeed);                                     //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
//  Kp = analogRead(pinP);
//  Kp = Ki/100;
//  Kd = analogRead(pinD);
//  Kd = Kd/100;
//  myPID.SetTunings(Kp, Ki, Kd);
  if(Serial.available()){
     
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
  int left_speed = baseSpeed - velocity;
  int right_speed = baseSpeed + velocity;
  
  left_speed = constrain(left_speed , 0, maxspeed);
  right_speed = constrain(right_speed, 0, maxspeed);
  
  digitalWrite(L298N_A, HIGH);
  digitalWrite(L298N_B, LOW);
  digitalWrite(L298N_C, HIGH);
  digitalWrite(L298N_D, LOW);

  analogWrite(L298N_EN, left_speed);
  analogWrite(L298N_EN_B, right_speed);
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
