/*
  Fade

  This example shows how to fade an LED on pin 9 using the analogWrite()
  function.

  The analogWrite() function uses PWM, so if you want to change the pin you're
  using, be sure to use another PWM capable pin. On most Arduino, the PWM pins
  are identified with a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Fade
*/

String r;
float data[4];
int led = 9;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int cm = 0;

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);
  analogWrite(led, 0);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // set the brightness of pin 9:
  if(Serial.available()){
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
//    Serial.println(cm);
  }
  analogWrite(led, cm);

}
