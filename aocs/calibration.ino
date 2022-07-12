#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

int ldr1 = A0;  //set to whatever pin on arduino
int ldr2 = A1;
int ldr3 = A2;
int ldr4 = A3;

int ldr1Value = 0;  //set values to be used later on 
int ldr2Value = 0;
int ldr3Value = 0;
int ldr4Value = 0;

int ldr_values[4];

MotorDriver motor;
Adafruit_MPU6050 mpu;
Adafruit_LIS3MDL lis3mdl;
#define LIS3MDL_CLK 13
#define LIS3MDL_MISO 12
#define LIS3MDL_MOSI 11
#define LIS3MDL_CS 10

uint16_t rpm = 120;

//Define variables for PID
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin();
  Serial.begin(9600);   
  //ldr set up
  pinMode(3,OUTPUT);    //setting the led (connected to D3) to an output
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  motor.init();
  //gyro set up
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  //magnetometer set up
  Serial.println("Adafruit LIS3MDL test!");
  
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!  
}
void loop() {
  // if some flag to start (calibration stage == true)
    // intial ldr values
    ldr1Value = analogRead(ldr1);
    ldr2Value = analogRead(ldr2);
    ldr3Value = analogRead(ldr3);
    ldr4Value = analogRead(ldr4);

    for (int i = 0; i <= 20; i++){                  //repeat 20 times (20*10=200==full turn)
      motor.stepperRun(FULL_STEP, 10*10, rpm);      //move 10 steps
      ldr1Value = ldr1ValueOld;                     //assign readings from last loop as old
      ldr2Value = ldr2ValueOld;
      ldr3Value = ldr3ValueOld;
      ldr4Value = ldr4ValueOld;
      ldr1Value = analogRead(ldr1);                 // take new reading
      if ldr1Value > ldr1ValueOld {                 // if new reading > old, assign as max reading
        ldrMax = ldr1Value;
      }
      else  {                                       // else do nothing (?)  
      }
      if ldr2Value > ldr2ValueOld {                 // if new reading > old, assign as max reading
        ldrMax = ldr2Value;
      }
      else  {                                       // else do nothing (?)  
      }
      if ldr3Value > ldr3ValueOld {                 // if new reading > old, assign as max reading
        ldrMax = ldr3Value;
      }
      else  {                                       // else do nothing (?)  
      }
      if ldr4Value > ldr4ValueOld {                 // if new reading > old, assign as max reading
        ldrMax = ldr4Value;
      }
      else  {                                       // else do nothing (?)  
      }                    
    }
    Serial.print("Maximum light value is: ", ldrMax)  
    
    motor.stepperStop();    // stop motor  
    
    // take gyro readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);          //use for control loop to stay still?

    // change to a pid loop to do this ?
    if ldr1Value != ldrMax  {
        motor.stepperRun(FULL_STEP, 10*10, rpm);      //move 10 steps
    }
    else {
       motor.stepperStop(); 
       // take magnetometer readings
       lis3mdl.read(); 
       float pi = 3.14159
       float heading = (atan2(lis3mdl.y,lis3mdl.x) * 180) / pi;    //get compass heading  
       Serial.print("Sun is found at: ", heading, " degrees")
    }
}
