int enA=6;
int in1=11;
int in2=10;
int enB=5;
int in3=9;
int in4=8;
void setup() 
{
  // put your setup code here, to run once:
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  analogWrite(enA, 60);
  analogWrite(enB, 60);
  delay(5000);
}

void demoOne()
{
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  analogWrite(enA,255);/*
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  analogWrite(enB,200);
  delay(2000);
*/
  digitalWrite(in2,HIGH);
  digitalWrite(in1,LOW);/*
  digitalWrite(in4,HIGH);
  digitalWrite(in3,LOW);*/
  delay(2000);

  digitalWrite(in2,LOW);
  digitalWrite(in1,LOW);/*
  digitalWrite(in4,LOW);
  digitalWrite(in3,LOW);*/
}

void demoTwo()
{
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);

  for(int i=0;i<226;i++)
  {
    analogWrite(enA,i);
    analogWrite(enB,i);
    delay(20);
  }

  for(int i=255;i>=0;--i)
  {
    analogWrite(enA,i);
    analogWrite(enB,i);
    delay(20);
  }
  digitalWrite(in2,LOW);
  digitalWrite(in1,LOW);
  digitalWrite(in4,LOW);
  digitalWrite(in3,LOW);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  //demoOne();
  //delay(1000);
  
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
