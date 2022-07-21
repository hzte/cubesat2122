// detection
// takes an input from the raspberry pi of which angle to rotate to
// maintains that angle 

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
  // if some flag to start (detection stage == true)
    desired_angle = //from raspberry pi
    //find current angle
      // === Read acceleromter data === //
      Wire.beginTransmission(MPU);
      Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
      //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
      AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
      AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
      AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
      // Calculating Roll and Pitch from the accelerometer data
      accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 1.02; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
      accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 11.42; // AccErrorY ~(-1.58)
      
      // === Read gyroscope data === //
      previousTime = currentTime;        // Previous time is stored before the actual time read
      currentTime = millis();            // Current time actual time read
      elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
      Wire.beginTransmission(MPU);
      Wire.write(0x43); // Gyro data first register address 0x43
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
      GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
      GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
      GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
      // Correct the outputs with the calculated error values
      GyroX = GyroX + 0.17; // GyroErrorX ~(-0.56)
      GyroY = GyroY + 1.62; // GyroErrorY ~(2)
      GyroZ = GyroZ + 1.99; // GyroErrorZ ~ (-0.8)
      // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
      gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
      gyroAngleY = gyroAngleY + GyroY * elapsedTime;
      yaw =  yaw + GyroZ * elapsedTime;
      // Complementary filter - combine acceleromter and gyro angle values
      roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
      pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

      //assuming we are using yaw
      if yaw != desired_angle
        motor.stepperRun(FULL_STEP, 10*10, rpm);      //move 10 steps

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
