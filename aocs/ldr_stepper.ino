#include <Stepper.h>
#define STEPS48
Stepper stepper(STEPS, 8, 9, 10, 11);
int state=1;

void setup() {
  /stepper.setSpeed(50);
  pinMode(0,INPUT);
  int ldr = (0);
  Serial.begin(9600);
}

void loop() {
  in ldr = analogRead (0);
  Serial.printIn(state);
  if (state < 2 && ldr > 750)
  stepper.step(150);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  delay (1000);
  if (ldr > 750)
  state = 5;
  Serial.printIn(state);
  if (state > 2 && ldr < 750)
  stepper.step(-150);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  digitalWrite(11,LOW);
  delay (1000);
  if (ldr < 750)
  state = 1;
  Serial.printIn(state);
  delay (1000);
}
