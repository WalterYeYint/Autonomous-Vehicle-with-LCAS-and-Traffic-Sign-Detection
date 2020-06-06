float Kp = 0;       //will be change using potentiometer
float Kd = 0;       //will be change using potettiometer
float pinP = 0;    //pin Analog 0 for the input of the potentiometer
float pinD = 1; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  Kp = analogRead(pinP);
  Kp = Kp/100;
  Serial.print(Kp);

  Serial.print(" ");
  
  Kd = analogRead(pinD);
  Kd = Kd/100;
  Serial.println(Kd);

}
