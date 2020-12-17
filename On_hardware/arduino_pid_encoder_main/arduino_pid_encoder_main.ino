
// note: for my beam to be horizontal, Servo Motor angle should be 102 degrees.
#include <TimerOne.h>
#include <PID_v1.h>


//Libraries and object initialization for NRF
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
const uint64_t pipes[] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL };     // , 0xF0F0F0F0E3LL 
double temp_arr[3];

//PID initialvalues
float Kp_R = 0.2;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki_R = 0.3;                                                      //Initial Integral Gain
float Kd_R = 0.09;

float Kp_L = 0.2;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki_L = 0.3;                                                      //Initial Integral Gain
float Kd_L = 0.09;


//PID ideal values
//value 1 ==> Kp = 6.5, Ki = 0, Kd = 1.1

#define L298N_A       12   //A pin of motor driver (L298N)
#define L298N_B       11   //B pin of motor driver (L298N)
#define L298N_EN      6  //EN pin (Left wheel) of motor driver (L298N)
#define L298N_C       10   //A pin of motor driver (L298N)
#define L298N_D       9   //B pin of motor driver (L298N)
#define L298N_EN_B      5   //EN_B (Right wheel) pin of motor driver (L298N)

const byte encoderPin_R = 2;
const byte encoderPin_L = 3;

volatile int slotCount_R = 0;
volatile int prevCount_R = 0;
volatile float speedData_R = 0.0;

volatile int slotCount_L = 0;
volatile int prevCount_L = 0;
volatile float speedData_L = 0.0;

//Length between wheels in cm
float L = 13.5;
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


String r;
float data[4];
//pretty good value => Kp=0.62,Ki=0,Kd=0.37
//better value => Kp=0.47,Ki=0,Kd=0.34

float Kp = 0.01;        //2.5 = default, 6.5 = perfect, 26.5 = shakin                                              //Initial Proportional Gain
float Ki = 0.02;                                                      //Initial Integral Gain
float Kd = 0.15;   

//potentiometers pin no.s
float pinP = 0;    //pin Analog 0 for the input of the potentiometer
float pinD = 1; 

double Setpoint = 0, Input, Output;   
int maxspeed = 120; 
int max_ang_velocity = 40;                                   
int baseSpeed = 40;
double Setspeed = 0;
double Input_R = 0, Output_R = 0, Setspeed_R = 0;
double Input_L = 0, Output_L = 0, Setspeed_L = 0;
float cm = 90;  //Camera offset data
float cm2 = 9;  //traffic_class for readSerialData()
float traffic_class = 0;


//For measuring time between NRF data arrays sent
double curtime=0, lasttime=0, totaltime=0;


PID PID_R(&Input_R, &Output_R, &Setspeed_R, Kp_R, Ki_R, Kd_R, DIRECT);
PID PID_L(&Input_L, &Output_L, &Setspeed_L, Kp_L, Ki_L, Kd_L, DIRECT);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           //Initialize PID object, which is in the class PID.


void setup() {
  //Initialize for NRF
  radio.begin();
//  radio.openWritingPipe(address);
  radio.openWritingPipe(pipes[0]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.stopListening();

  //Initialize Timer and interrupts
  Timer1.initialize(1000000/samplingFreq);
  Timer1.attachInterrupt(calculateSpeed);
  attachInterrupt(digitalPinToInterrupt(encoderPin_R), ISR_CountPlus_R, RISING); //increase counter when pL298N_D goes high
  attachInterrupt(digitalPinToInterrupt(encoderPin_L), ISR_CountPlus_L, RISING);
  

  Serial.begin(9600); //Attach Servo
  delay(5000);
  pinMode(13, INPUT_PULLUP);
  motorInitialize();
  Input = 90; 
  Setspeed = 70;
  myPID.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-max_ang_velocity, max_ang_velocity);                                     //Set Output limits to -80 and 80 degrees. 
  PID_R.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  PID_R.SetOutputLimits(-maxspeed, maxspeed);
  PID_L.SetMode(AUTOMATIC);                                          //Set PID object myPID to AUTOMATIC 
  PID_L.SetOutputLimits(-maxspeed, maxspeed);
}

void loop()
{
  
//  Kp = analogRead(pinP);
//  Kp = Kp/100;
//  Kd = analogRead(pinD);
//  Kd = Kd/100;
//  myPID.SetTunings(Kp, Ki, Kd);

//  Kp_R = analogRead(pinP);
//  Kp_R = Kp_R/2000;
//  Kd_R = analogRead(pinD);
//  Kd_R = Kd_R/5000;
//  Ki_R = analogRead(pinD);
//  Ki_R = Ki_R/1000;
//  PID_R.SetTunings(Kp_R, Ki_R, Kd_R);
  
//  Kp_L = analogRead(pinP);
//  Kp_L = Kp_L/2000;
//  Kd_L = analogRead(pinD);
//  Kd_L = Kd_L/5000;
//  Ki_L = analogRead(pinD);
//  Ki_L = Ki_L/1000;
//  PID_L.SetTunings(Kp_L, Ki_L, Kd_L);
  
  //    Kp = data[1];
  //    Ki = data[2];
  //    Kd = data[3];
  if(Serial.available()){
    readSerialData();
    
    if(traffic_class == 6 || traffic_class == 2){
      motorStop();
      
      while(traffic_class != 1){
        Input_R = 0;
        Input_L = 0;
        temp_arr[0] = Input_R;
        temp_arr[1] = Input_L;
        temp_arr[2] = 1;
        radio.write(&temp_arr, sizeof(temp_arr));
        if(Serial.available())
          readSerialData();
      }
    }
    else{
      if(traffic_class == 3){
        Setspeed = 60;
      }
      else if(traffic_class == 1){
        Setspeed = 70;
      }
      else if(traffic_class == 4){
        Setspeed = 80;
      }
      else if(traffic_class == 5){
        Setspeed = 90;
      }
      
      myPID.Compute();
        Setspeed_R = Setspeed + (Output * L)/2;
        Setspeed_L = Setspeed - (Output * L)/2;
    
      Input_R = filtered_R;
      Input_L = filtered_L;
      PID_R.Compute();
      PID_L.Compute();
      motorMove_R(Output_R);
      motorMove_L(Output_L);  


      //For calculating time between each NRF data array sent
//      curtime = millis();
//      totaltime = curtime - lasttime;
//      lasttime = curtime;
      
      //Send data to Observer base station
      temp_arr[0] = Input_R;
      temp_arr[1] = Input_L;
      temp_arr[2] = 0;
      radio.write(&temp_arr, sizeof(temp_arr));
    //  Serial.println(Input);
    //  Serial.println(Output);
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
}

void ISR_CountPlus_R(){
  slotCount_R++;
}
void ISR_CountPlus_L(){
  slotCount_L++;
}

void calculateSpeed(){
//  lasttime = micros();
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

//  curtime = micros();
//  totaltime = curtime - lasttime;
//  curtime = 0;
//  lasttime = 0;
}


void readSerialData(){
  for(int i=0; i<4; i++){
    r = (Serial.readStringUntil('\t'));  //conveting the value of chars to integer
    data[i] = r.toFloat();
//      Serial.print(data[i]);
//      Serial.print('\t');
  }
  Input = data[0];
  traffic_class = data[1];
//    Serial.println(Input);
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

void motorMove_R(int offset){
  int right_speed = baseSpeed + offset;
  right_speed = constrain(right_speed, 0, maxspeed);
  digitalWrite(L298N_C, HIGH);
  digitalWrite(L298N_D, LOW);
  analogWrite(L298N_EN_B, right_speed);
}

void motorMove_L(int offset){
  int left_speed = baseSpeed + offset;
  left_speed = constrain(left_speed , 0, maxspeed);
  digitalWrite(L298N_A, HIGH);
  digitalWrite(L298N_B, LOW);
  analogWrite(L298N_EN, left_speed);
}     

void motorStop(){
  digitalWrite(L298N_A, 0);
  digitalWrite(L298N_B, 0);
  analogWrite(L298N_EN, 0); 
  digitalWrite(L298N_C, 0);
  digitalWrite(L298N_D, 0);
  analogWrite(L298N_EN_B, 0);  
}
