#include <TimerOne.h>
#include <PID_v1.h>

float Kp_R = 0.14;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki_R = 0.13;                                                      //Initial Integral Gain
float Kd_R = 0.04;

float Kp_L = 0.14;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki_L = 0.13;                                                      //Initial Integral Gain
float Kd_L = 0.04;

float pinP = 0;    //pin Analog 0 for the input of the potentiometer
float pinD = 1; 

int maxspeed = 255;
double Setspeed = 200;
double Input_R = 0, Output_R = 0;
double Input_L = 0, Output_L = 0;

PID PID_R(&Input_R, &Output_R, &Setspeed, Kp_R, Ki_R, Kd_R, DIRECT);
PID PID_L(&Input_L, &Output_L, &Setspeed, Kp_L, Ki_L, Kd_L, DIRECT);

#define L298N_A       11   //A pin of motor driver (L298N)
#define L298N_B       10   //B pin of motor driver (L298N)
#define L298N_EN      6  //EN pin (Left wheel) of motor driver (L298N)
#define L298N_C       9   //A pin of motor driver (L298N)
#define L298N_D       8   //B pin of motor driver (L298N)
#define L298N_EN_B      5   //EN_B (Right wheel) pin of motor driver (L298N)

const byte encoderPin_R = 2;
const byte encoderPin_L = 3;

volatile int slotCount_R = 0;
volatile int prevCount_R = 0;
volatile float speedData_R = 0.0;

volatile int slotCount_L = 0;
volatile int prevCount_L = 0;
volatile float speedData_L = 0.0;

// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk, change if different

// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

float circumference = (wheeldiameter * 3.14) / 10.00; // Calculate wheel circumference in cm

// implementing moving average
const int samplingFreq = 20;    // 20 -> 20 samples per second or sample at every 50 ms

int arrayCount_R = 0;             // index for sample storing array
float sum_R=0.0;          // to collect sum of the samples
float speedArray_R[samplingFreq];   // declaring array with size of sampling freq
float filtered_R;         // filtered value will be stored here

int arrayCount_L = 0;             // index for sample storing array
float sum_L=0.0;          // to collect sum of the samples
float speedArray_L[samplingFreq];   // declaring array with size of sampling freq
float filtered_L;         // filtered value will be stored here

void setup() {
  Timer1.initialize(1000000/samplingFreq);
  Timer1.attachInterrupt(calculateSpeed);
  attachInterrupt(digitalPinToInterrupt(encoderPin_R), ISR_CountPlus_R, RISING); //increase counter when pL298N_D goes high
  attachInterrupt(digitalPinToInterrupt(encoderPin_L), ISR_CountPlus_L, RISING);
  
  Serial.begin(9600);

  pinMode(13, INPUT_PULLUP);
  motorInitialize();
  PID_R.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  PID_R.SetOutputLimits(0, maxspeed);
  PID_L.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  PID_L.SetOutputLimits(0, maxspeed);
  motorMove_R(0);
  motorMove_L(0);
//  delay(3000);

}

void loop() {
  if(digitalRead(13) == LOW)
    Setspeed = 0;
  else
    Setspeed = 120;
  
//  Kp_R = analogRead(pinP);
//  Kp_R = Kp_R/1000;
//  Kd_R = analogRead(pinD);
//  Kd_R = Kd_R/10000;
////  Ki_R = analogRead(pinD);
////  Ki_R = Ki_R/1500;
//  PID_R.SetTunings(Kp_R, Ki_R, Kd_R);

//  Kp_L = analogRead(pinP);
//  Kp_L = Kp_L/1000;
//  Kd_L = analogRead(pinD);
//  Kd_L = Kd_L/10000;
////  Ki_L = analogRead(pinD);
////  Ki_L = Ki_L/1500;
//  PID_L.SetTunings(Kp_L, Ki_L, Kd_L);

  Input_R = filtered_R;
  Input_L = filtered_L;
  
  PID_R.Compute();
  PID_L.Compute();
  // Put whatever you want here!
  motorMove_R(Output_R);
  motorMove_L(Output_L);
//  Serial.println(counter_R);
//  Serial.print("Speed_L = :");
//  Serial.print(Kp_R);
//  Serial.print(" ");
//  Serial.print(Kd_R);
//  Serial.print(" ");
////  Serial.print(Ki_R);
////  Serial.print(" ");
  Serial.print(Input_R);
  Serial.print(" ");

  Serial.print(Kp_L);
  Serial.print(" ");
  Serial.print(Kd_L);
  Serial.print(" ");
//  Serial.print(Ki_L);
//  Serial.print(" ");
  Serial.print(Input_L);
  Serial.print(" ");
//  Serial.print("Speed_R = :");
  Serial.println(Setspeed);
}

void ISR_CountPlus_R(){
  slotCount_R++;
}
void ISR_CountPlus_L(){
  slotCount_L++;
}

void calculateSpeed(){
  int i;                  // loop index
  int currentCount_R;             // declare a temporary variable
  currentCount_R = slotCount_R;     // store current count into that

  speedData_R = (currentCount_R / stepcount) * circumference * samplingFreq;  // calculate speed
  arrayCount_R++;                               //increment array index when a new sample is received

  if(arrayCount_R <= samplingFreq){             // assume samplingFreq = 20 (every sample at 50ms), arraySize = 20
    speedArray_R[arrayCount_R-1]=speedData_R;       // just store the newly received speedData in array while increasing index WHEN total no. of samples is less than 20 (samplingFreq)
  }
  else{                                       // when the 21st sample is received,
    for(i=1; i<=samplingFreq; i++){         // DO THIS FOR 20 TIMES
      sum_R += speedArray_R[i-1];                 // calculate the collective sum of speedData 
      if(i != samplingFreq){
        speedArray_R[i-1]=speedArray_R[i];          // and move the data to one slot above
      }
    }
    speedArray_R[samplingFreq-1] = speedData_R;     // newly received 21st sample is stored into 20th slot
    //sum += speedData;                         // also plus its value into sum
        
  }
  filtered_R = sum_R/samplingFreq;               // filtered valule is total sum divided by no. of samples 
  
//  Serial.print("Actual Speed : ");
//  Serial.println(filtered);

  slotCount_R = 0;     // used current count becomes the previous count
  sum_R = 0;
  //filtered = 0;

  ////////////////////////////////////////////////////////////
  //for Left motor
  int currentCount_L;             // declare a temporary variable
  currentCount_L = slotCount_L;     // store current count into that

  speedData_L = (currentCount_L / stepcount) * circumference * samplingFreq;  // calculate speed
  arrayCount_L++;                               //increment array index when a new sample is received

  if(arrayCount_L <= samplingFreq){             // assume samplingFreq = 20 (every sample at 50ms), arraySize = 20
    speedArray_L[arrayCount_L-1]=speedData_L;       // just store the newly received speedData in array while increasing index WHEN total no. of samples is less than 20 (samplingFreq)
  }
  else{                                       // when the 21st sample is received,
    for(i=1; i<=samplingFreq; i++){         // DO THIS FOR 20 TIMES
      sum_L += speedArray_L[i-1];                 // calculate the collective sum of speedData 
      if(i != samplingFreq){
        speedArray_L[i-1]=speedArray_L[i];          // and move the data to one slot above
      }
    }
    speedArray_L[samplingFreq-1] = speedData_L;     // newly received 21st sample is stored into 20th slot
    //sum += speedData;                         // also plus its value into sum
        
  }
  filtered_L = sum_L/samplingFreq;               // filtered valule is total sum divided by no. of samples 
  
//  Serial.print("Actual Speed : ");
//  Serial.println(filtered);

  slotCount_L = 0;     // used current count becomes the previous count
  sum_L = 0;
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

void motorMove_R(int velocity){
  int right_speed = constrain(velocity, 0, maxspeed);
  digitalWrite(L298N_C, HIGH);
  digitalWrite(L298N_D, LOW);
  analogWrite(L298N_EN_B, right_speed);
}

void motorMove_L(int velocity){
  int left_speed = constrain(velocity , 0, maxspeed);
  digitalWrite(L298N_A, HIGH);
  digitalWrite(L298N_B, LOW);
  analogWrite(L298N_EN, left_speed);
}     
