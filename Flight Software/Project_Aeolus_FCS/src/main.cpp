#include <Arduino.h>

/* Include necessary libraries */
#include <Wire.h>               // Wire library (I2C Communication)
#include "wiring_private.h"     // Necessary for the "pinPeripheral" function

#include <Servo.h>

#include "MPU9250.hpp"

#define DEBUG           // Uncomment this line to put the program into debug mode

/* Create instances of classes */
TwoWire mpu9250(&sercom2, 4, 3);    // Create an instance of I2C on pins 3 and 4
Aeolus::MPU9250 sensor(mpu9250, Aeolus::ACCEL_SCALE::AFS_2G, Aeolus::GYRO_SCALE::GFS_250DPS, Aeolus::MAG_SCALE::MFS_14BITS);

Servo servo;

/* Global Variables */
long t, tPrev;
float deltaT;

float roll = 0.0f, pitch = 0.0f;
bool gyroSynched = false;

float accelRoll = 0.0f, accelPitch = 0.0f;

float kp = 1.0f, ki = 0.01f, kd = 0.1f;
float pTerm, iTerm, dTerm;
float pid;
float error, prevError;
float servoSig;
unsigned long pidTime = 10, prevPIDTime = 0;
float desiredHeading = 90.0f;

void setup() {
  SerialUSB.begin(9600);    // Begin USB communication with the computer
  Serial1.begin(9600);      // Begin serial communication with the GPS
  
  mpu9250.begin();          // Begin I2C communication with the MPU-9250

  servo.attach(5);

  // Set pins 4 and 3 for I2C
  pinPeripheral(4, PIO_SERCOM_ALT);
  pinPeripheral(3, PIO_SERCOM_ALT);

  while(!SerialUSB);

  SerialUSB.println("\tWelcome to the Aeolus Flight Computer!\n");
  SerialUSB.println("The Aeolus Flight Computer is a device that allows for automatic guidance of high-altitude payloads for weather balloons.");
  SerialUSB.println("It uses an inertial measurement unit, a GPS, and a compass to correct for the proper heading of a given landing location.");
  SerialUSB.println("By using a rapid control loop, the flight computer can correct for any errors in heading.\n");

  SerialUSB.println("Initializing board...");
  sensor.begin();
  SerialUSB.println("Done.\n");

  SerialUSB.println("Getting ready to calibrate magnetometer...\n");

  delay(3000);
  
  SerialUSB.println("Calibrating magnetometer (move flight computer in a figure eight pattern)...");
  sensor.calibrateMag();
  SerialUSB.println("Done.\n");

  SerialUSB.println("Place computer back down on a level surface...\n");

  delay(5000);
  
  SerialUSB.println("Calibrating IMU (hold steady on a flat surface)...");
  sensor.calibrateMPU();
  delay(100);
  SerialUSB.println("Done.\n");

  delay(1000);

  SerialUSB.println("Entering main control loop in...");
  long start = millis();
  long seconds = 3;
  while (seconds > 0) {
    if (millis() - start >= 1000) {
      SerialUSB.println(seconds);
      seconds--;
      start += 1000;
    }
  }

  SerialUSB.println("\n");
}

void loop() {
  tPrev = t;
  t = millis();
  deltaT = (float)(t - tPrev) / 1000.0f;
  
  float aX = sensor.accelX();
  float aY = sensor.accelY();
  float aZ = sensor.accelZ();

  float angleX = (sensor.gyroX()*deltaT);
  float angleY = (sensor.gyroY()*deltaT);
  
  accelRoll = 0.95*accelRoll + 0.05*((atan2(-aY, aZ)*180.0f)/PI);
  accelPitch = 0.95*accelPitch + 0.05*((atan(aX/sqrt(aY*aY + aZ*aZ))*180.0f)/PI);

  if (gyroSynched) {
    roll = 0.92*(roll + angleX) + 0.08*(accelRoll);
    pitch = 0.92*(pitch + angleY) + 0.08*(accelPitch);
  }
  else {
    roll = accelRoll;
    pitch = accelPitch;
    gyroSynched = true;
  }
  
  #ifdef DEBUG
  SerialUSB.print("Delta T: "); SerialUSB.print(deltaT); SerialUSB.println(" seconds\n");
  
  SerialUSB.print("Gyro X: "); SerialUSB.print(angleX*180); SerialUSB.println(" deg");
  SerialUSB.print("Gyro Y: "); SerialUSB.print(angleY); SerialUSB.println(" deg");
  SerialUSB.print("Accel X: "); SerialUSB.print(accelRoll); SerialUSB.println(" deg");
  SerialUSB.print("Accel Y: "); SerialUSB.print(accelPitch); SerialUSB.println(" deg");
  SerialUSB.print("Roll: "); SerialUSB.print(roll); SerialUSB.println(" deg");
  SerialUSB.print("Pitch: "); SerialUSB.print(pitch); SerialUSB.println(" deg\n");
  #endif

  float magX = sensor.magX();
  float magY = sensor.magY();
  float magZ = sensor.magZ();

  float magPitch = (pitch*PI)/180.0f;
  float magRoll = (roll*PI)/180.0f; 
  
  float yh = magY*cos(magRoll) + magZ*sin(magRoll);
  float xh = magX*cos(magPitch) + magY*sin(magRoll)*sin(magPitch) - magZ*cos(magRoll)*sin(magPitch);
  
  float heading = ((atan2(yh, xh)*180.0f)/PI);

  // heading = (xh < 0) ? 180 + heading : (xh > 0 && yh < 0) ? heading : (xh > 0 && yh > 0) ? 360 + heading : (xh == 0 && yh < 0) ? 90 : 270; 

  #ifdef DEBUG
  SerialUSB.print("Heading: "); SerialUSB.print(heading); SerialUSB.println(" deg of N\n");
  #endif

  if ((millis() - prevPIDTime) > pidTime) {
    error = heading - desiredHeading;

    pTerm = kp*error;

    iTerm += (ki*error)*deltaT;

    dTerm = kd*(error - prevError)/deltaT;

    pid = pTerm + iTerm + dTerm;

    if (pid < -90) pid = -90;
    if (pid > 90) pid = 90;
    prevError = error;
    
    servoSig = 90 - pid;

    servo.write(servoSig);

    #ifdef DEBUG
    SerialUSB.print("PID Value: "); SerialUSB.print(servoSig); SerialUSB.println("");
    SerialUSB.print("Error: "); SerialUSB.print(error); SerialUSB.println("\n");
    #endif

    prevPIDTime = millis();
  }
  delay(5);
}