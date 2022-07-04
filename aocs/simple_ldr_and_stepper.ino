//for ldr
int ldr=A0;//Set A0(Analog Input) for LDR.
int value=0;

//for motor
#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>

MotorDriver motor;

// set speed to 120 revolutions per minute
uint16_t rpm = 120;
void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(3,OUTPUT);    //setting the led (connected to D3) to an output
  motor.init();
}

void loop() {
  value=analogRead(ldr);//Reads the Value of LDR(light).
  Serial.println("LDR value is :");//Prints the value of LDR to Serial Monitor.
  Serial.println(value);
  if(value<400)         //dark
    {
      digitalWrite(3,HIGH);       //Makes the LED glow
      motor.stepperKeepRun(MICRO_STEPPING, rpm, true);    //clockwise
      //delay(3000);
    }
  else                  //light
  {
    digitalWrite(3,LOW);          //Turns the LED OFF
    //motor.stepperKeepRun(MICRO_STEPPING, rpm, false); //anticlockwise
    //delay(3000);
    motor.stepperStop();
    //delay(3000);

  }

}
