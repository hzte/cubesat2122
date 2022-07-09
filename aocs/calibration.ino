void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // if some flag to start (calibration stage == true)
    // motor turns once
    motor.stepperRun(HALF_STEP, 400*10, rpm);
    // loop while motor is running
      // take each ldr reading
      ldr1Value = analogRead(ldr1);
      ldr2Value = analogRead(ldr2);
      ldr3Value = analogRead(ldr3);
      ldr4Value = analogRead(ldr4);
      // write to csv
      // take gyro readings
      // write to csv
      // take mag readings 
      // write to csv
      // stop motor
      motor.stepperStop();
    // find largest ldr value
    // could just continually re write a variable ?
    // if ldr1 > largest value
      // largest value = ldr1
      // face = 1 (variable that will end saying what face has the largest value)
    // else
      // nothing ((is this allowed))
    // if ldr1 != largest value
      // start motor
      // take ldr1 value
      // if ldr1 == largest value (found sun)
      // stop motor
      // print sun found 
    // else (means its already facing sun w/out moving
      // stop motor
      // print sun found 

}
