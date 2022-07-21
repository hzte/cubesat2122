// calibration stage
// rotate once to find the light source (the highest ldr value)
// rotate to that position and stay

// include libraries
#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

// set pins (on Arduino)
int ldr1 = A0;  //set to whatever pin on arduino
int ldr2 = A1;
int ldr3 = A2;
int ldr4 = A3;

#define echoPin 2 // D3 (Arduino) to Echo pin (on ultrasound)
#define trigPin 3 // D2 (Arduino) to Trig pin (on ultrasound)

// inititialise values
int ldr1Value = 0;  //set values to be used later on 
int ldr2Value = 0;
int ldr3Value = 0;
int ldr4Value = 0;

// initialise list
int ldr_values[4];

// initialise motor
MotorDriver motor;
uint16_t rpm = 120;

// define gyro.accel (imu)
Adafruit_MPU6050 mpu;

// for imu
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


//Define variables for PID
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  // imu set up
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
  
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

  // ultrasound sensor set up
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
 
}

void loop() {
  // if some flag to start (calibration stage == true)
    // intial ldr vslues
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
      // take ultrasound reading
      
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
    mpu.getEvent(&a, &g, &temp);          //use in control loop to stay still?

    // add something here to account for drift

    // change to a pid loop to do this ?
    if ldr1Value != ldrMax  {
        motor.stepperRun(FULL_STEP, 10*10, rpm);      //move 10 steps
    }
    else {
       motor.stepperStop(); 
    }

}

// for finding error for imu
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void ultrasound {
        // Clears the trigPin condition
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);
      // Calculating the distance
      distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
      // Displays the distance on the Serial Monitor
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      // send this to raspberry pi 
}
