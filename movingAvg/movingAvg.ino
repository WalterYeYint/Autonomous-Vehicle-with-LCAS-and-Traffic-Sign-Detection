#include <TimerOne.h>
#include <PID_v1.h>

float Kp_R = 0.14;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki_R = 0.13;                                                      //Initial Integral Gain
float Kd_R = 0.04;

float pinP = 0;    //pin Analog 0 for the input of the potentiometer
float pinD = 1; 

int maxspeed = 255;
double Setspeed_R = 200, Input_R, Output_R;

PID PID_R(&Input_R, &Output_R, &Setspeed_R, Kp_R, Ki_R, Kd_R, DIRECT);

#define L298N_A       11   //A pin of motor driver (L298N)
#define L298N_B       10   //B pin of motor driver (L298N)
#define L298N_EN      6  //EN pin (Left wheel) of motor driver (L298N)
#define L298N_C       9   //A pin of motor driver (L298N)
#define L298N_D       8   //B pin of motor driver (L298N)
#define L298N_EN_B      5   //EN_B (Right wheel) pin of motor driver (L298N)

int pwmOutput = 50;  // between 0 and 255
const byte encoderPin = 2;

volatile int slotCount = 0;
volatile int prevCount = 0;
volatile float speedData = 0.0;

// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk, change if different

// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

float circumference = (wheeldiameter * 3.14) / 10.00; // Calculate wheel circumference in cm

// implementing moving average
int arrayCount = 0;             // index for sample storing array
const int samplingFreq = 20;    // 20 -> 20 samples per second or sample at every 50 ms
float sum=0.0;          // to collect sum of the samples
float speedArray[samplingFreq];   // declaring array with size of sampling freq
float filtered;         // filtered value will be stored here

void setup() {
  Timer1.initialize(1000000/samplingFreq);
  Timer1.attachInterrupt(calculateSpeed);
  attachInterrupt(digitalPinToInterrupt(encoderPin), ISR_CountPlus, RISING); //increase counter when pL298N_D goes high
  
  Serial.begin(9600);

  pinMode(13, INPUT_PULLUP);
  motorInitialize();
  Input_R = 200;
  PID_R.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  PID_R.SetOutputLimits(0, maxspeed);
//  motorMove(45);
//  delay(3000);

}

void loop() {
  if(digitalRead(13) == LOW)
    Setspeed_R = 0;
  else
    Setspeed_R = 180;
  
  Kp_R = analogRead(pinP);
  Kp_R = Kp_R/1000;
  Kd_R = analogRead(pinD);
  Kd_R = Kd_R/10000;
//  Ki_R = analogRead(pinD);
//  Ki_R = Ki_R/1500;
  PID_R.SetTunings(Kp_R, Ki_R, Kd_R);

  Input_R = filtered;
  
  PID_R.Compute();
  // Put whatever you want here!
  motorMove(Output_R);
//  Serial.println(counter_R);
//  Serial.print("Speed_L = :");
  Serial.print(Kp_R);
  Serial.print(" ");
  Serial.print(Kd_R);
  Serial.print(" ");
//  Serial.print(Ki_R);
//  Serial.print(" ");
  Serial.print(Input_R);
  Serial.print(" ");
//  Serial.print("Speed_R = :");
  Serial.println(Setspeed_R);
}

void ISR_CountPlus(){
  slotCount++;
}

void calculateSpeed(){
  int i;                  // loop index
  int currentCount;             // declare a temporary variable
  currentCount = slotCount;     // store current count into that

  speedData = (currentCount / stepcount) * circumference * samplingFreq;  // calculate speed
  arrayCount++;                               //increment array index when a new sample is received

  if(arrayCount <= samplingFreq){             // assume samplingFreq = 20 (every sample at 50ms), arraySize = 20
    speedArray[arrayCount-1]=speedData;       // just store the newly received speedData in array while increasing index WHEN total no. of samples is less than 20 (samplingFreq)
  }
  else{                                       // when the 21st sample is received,
    for(i=1; i<=samplingFreq; i++){         // DO THIS FOR 20 TIMES
      sum += speedArray[i-1];                 // calculate the collective sum of speedData 
      if(i != samplingFreq){
        speedArray[i-1]=speedArray[i];          // and move the data to one slot above
      }
    }
    speedArray[samplingFreq-1] = speedData;     // newly received 21st sample is stored into 20th slot
    //sum += speedData;                         // also plus its value into sum
        
  }
  filtered = sum/samplingFreq;               // filtered valule is total sum divided by no. of samples 
  
//  Serial.print("Actual Speed : ");
//  Serial.println(filtered);

  slotCount = 0;     // used current count becomes the previous count
  sum = 0;
  //filtered = 0;
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
  int left_speed = constrain(velocity , 0, maxspeed);
  int right_speed = constrain(velocity, 0, maxspeed);
  
  digitalWrite(L298N_A, LOW);
  digitalWrite(L298N_B, LOW);
  digitalWrite(L298N_C, HIGH);
  digitalWrite(L298N_D, LOW);

  analogWrite(L298N_EN, left_speed);
  analogWrite(L298N_EN_B, right_speed);
}     
