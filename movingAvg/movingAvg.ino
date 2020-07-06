#include <TimerOne.h>

#define enA 5
#define in1 9
#define in2 8

int pwmOutput = 50;  // between 0 and 255
const byte encoderPin = 2;

volatile int slotCount = 0;
volatile int prevCount = 0;
volatile float speedData = 0.0;

// implementing moving average
int arrayCount = 0;             // index for sample storing array
const int samplingFreq = 20;    // 20 -> 20 samples per second or sample at every 50 ms
float sum=0.0;          // to collect sum of the samples
float speedArray[samplingFreq];   // declaring array with size of sampling freq
float filtered;         // filtered value will be stored here

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  Timer1.initialize(1000000/samplingFreq);
  Timer1.attachInterrupt(calculateSpeed);
  attachInterrupt(digitalPinToInterrupt(encoderPin), ISR_CountPlus, RISING); //increase counter when pin2 goes high
  
  Serial.begin(9600);

}

void loop() {
  analogWrite(enA, pwmOutput);
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

}

void ISR_CountPlus(){
  slotCount++;
}

void calculateSpeed(){
  int i;                  // loop index
  int currentCount;             // declare a temporary variable
  currentCount = slotCount;     // store current count into that

  speedData = (currentCount-prevCount)*1.04*samplingFreq;  // calculate speed
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
  
  Serial.print("Actual Speed : ");
  Serial.println(filtered);

  prevCount = currentCount;     // used current count becomes the previous count
  sum = 0;
  //filtered = 0;
}
