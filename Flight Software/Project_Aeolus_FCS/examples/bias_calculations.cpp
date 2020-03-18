#include <Arduino.h>

/* Include necessary libraries */
#include <Wire.h>               // Wire library (I2C Communication)
#include "wiring_private.h"     // Necessary for the "pinPeripheral" function

#include <Servo.h>
#include <Stepper.h>

#include "MPU9250.hpp"

/* Create instances of classes */
TwoWire mpu9250(&sercom2, 4, 3);    // Create an instance of I2C on pins 3 and 4
Aeolus::MPU9250 sensor(mpu9250, Aeolus::ACCEL_SCALE::AFS_2G, Aeolus::GYRO_SCALE::GFS_250DPS, Aeolus::MAG_SCALE::MFS_14BITS);


void setup() {
  SerialUSB.begin(9600);    // Begin USB communication with the computer
  Serial1.begin(9600);      // Begin serial communication with the GPS
  
  mpu9250.begin();          // Begin I2C communication with the MPU-9250
                 
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

  SerialUSB.print("MAG BIAS X: "); SerialUSB.println(sensor.magBias[0]);
  SerialUSB.print("MAG BIAS Y: "); SerialUSB.println(sensor.magBias[1]);
  SerialUSB.print("MAG BIAS Z: "); SerialUSB.println(sensor.magBias[2]);

  SerialUSB.print("MAG Scale BIAS X: "); SerialUSB.println(sensor.magBiasScale[0]);
  SerialUSB.print("MAG Scale BIAS Y: "); SerialUSB.println(sensor.magBiasScale[1]);
  SerialUSB.print("MAG Scale BIAS Z: "); SerialUSB.println(sensor.magBiasScale[2]);

  SerialUSB.println();

  SerialUSB.println("Place computer back down on a level surface...\n");

  delay(5000);
  
  SerialUSB.println("Calibrating IMU (hold steady on a flat surface)...");
  sensor.calibrateMPU();
  delay(100);
  SerialUSB.println("Done.\n");

  SerialUSB.print("Gyro Bias X: "); SerialUSB.println(sensor.gyroBias[0]);
  SerialUSB.print("Gyro Bias Y: "); SerialUSB.println(sensor.gyroBias[1]);
  SerialUSB.print("Gyro Bias Z: "); SerialUSB.println(sensor.gyroBias[2]);

  SerialUSB.print("Accel Bias X: "); SerialUSB.println(sensor.accelBias[0]);
  SerialUSB.print("Accel Bias Y: "); SerialUSB.println(sensor.accelBias[1]);
  SerialUSB.print("Accel Bias Z: "); SerialUSB.println(sensor.accelBias[2]);

  Serial.println();
}

void loop() {
}