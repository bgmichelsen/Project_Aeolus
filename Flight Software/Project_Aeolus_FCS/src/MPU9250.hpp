#ifndef _MPU9250_H
#define _MPU9250_H

/* Include necessary libraries */
#include <Wire.h>               // Wire library (I2C Communication)
#include "wiring_private.h"     // Necessary for the "pinPeripheral" function


namespace Aeolus {
    const uint8_t MPU_ADDR = 0x68;

    const uint8_t PWR_MGMT = 0x6B;
    const uint8_t CONFIG = 0x1A;
    const uint8_t INT_CFG = 0x37;

    const uint8_t GYRO_CONFIG = 0x1B;
    const uint8_t ACCEL_CONFIG = 0x1C;
    
    const uint8_t GYRO_X_SELF_TEST = 0x00;
    const uint8_t GYRO_Y_SELF_TEST = 0x01;
    const uint8_t GYRO_Z_SELF_TEST = 0x02;

    const uint8_t ACCEL_X_SELF_TEST = 0x0D;
    const uint8_t ACCEL_Y_SELF_TEST = 0x0E;
    const uint8_t ACCEL_Z_SELF_TEST = 0x0F;

    const uint8_t GYRO_X_OUT = 0x43;
    const uint8_t GYRO_Y_OUT = 0x45;
    const uint8_t GYRO_Z_OUT = 0x47;
    
    const uint8_t ACCEL_X_OUT = 0x3B;
    const uint8_t ACCEL_Y_OUT = 0x3D;
    const uint8_t ACCEL_Z_OUT = 0x3F;

    const uint8_t MAG_ADDR = 0x0C;
    const uint8_t MAG_WHO_AM_I = 0x00;
    const uint8_t MAG_CNTRL = 0x0A;
    const uint8_t MAG_READY = 0x02;
    const uint8_t MAG_OVFLOW = 0x09;
    const uint8_t MAG_AS = 0x10;

    const uint8_t MAG_X_OUT = 0x03;
    const uint8_t MAG_Y_OUT = 0x05;
    const uint8_t MAG_Z_OUT = 0x07;


    enum ACCEL_SCALE {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum GYRO_SCALE {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };

    enum MAG_SCALE {
      MFS_14BITS = 0,
      MFS_16BITS
    };
    
    class MPU9250 {
     private:
        TwoWire& imu;

        float magCal[3];

        uint8_t accelScale, gyroScale, magScale;
        float accelRes, gyroRes, magRes;

        void writeI2C(const uint8_t ADDR, const uint8_t REG, uint8_t data) {
          this->imu.beginTransmission(ADDR);
          this->imu.write(REG);
          this->imu.write(data);
          this->imu.endTransmission();
        }

        void readI2C(const uint8_t ADDR, const uint8_t REG, const uint8_t NUM_BYTES, uint8_t *buf) {
          this->imu.beginTransmission(ADDR);
          this->imu.write(REG);
          this->imu.endTransmission(false);

          uint8_t index = 0;
          this->imu.requestFrom(ADDR, NUM_BYTES);
          while (this->imu.available()) {
            buf[index++] = this->imu.read();
          }
        }

        void initMPU() {
          writeI2C(MPU_ADDR, PWR_MGMT, 0x00);
          delay(100);

          writeI2C(MPU_ADDR, PWR_MGMT, 0x01);
          delay(200);

          writeI2C(MPU_ADDR, CONFIG, 0x02);

          uint8_t conf = 0;

          readI2C(MPU_ADDR, GYRO_CONFIG, 1, &conf);
          conf &= ~0x03;
          conf &= ~0x18;
          conf |= gyroScale << 3;
          writeI2C(MPU_ADDR, GYRO_CONFIG, conf);

          readI2C(MPU_ADDR, ACCEL_CONFIG, 1, &conf);
          conf &= ~0x18;
          conf |= accelScale << 3;
          writeI2C(MPU_ADDR, ACCEL_CONFIG, conf);

          writeI2C(MPU_ADDR, INT_CFG, 0x02);
          delay(100);
        }

        void initMag() {
          uint8_t rawData[3];
          writeI2C(MAG_ADDR, MAG_CNTRL, 0x00);
          delay(10);
          writeI2C(MAG_ADDR, MAG_CNTRL, 0x0F);
          delay(10);
          readI2C(MAG_ADDR, MAG_AS, 3, &rawData[0]);
          magCal[0] = (float)(rawData[0] - 128)/256.0f + 1.0f;
          magCal[1] = (float)(rawData[1] - 128)/256.0f + 1.0f;
          magCal[2] = (float)(rawData[2] - 128)/256.0f + 1.0f;
          writeI2C(MAG_ADDR, MAG_CNTRL, 0x00);
          delay(10);
          writeI2C(MAG_ADDR, MAG_CNTRL, magScale << 4 | 0x02);
          delay(10);
        }

        void readAccel() {
          uint8_t rawData[3*2];
          readI2C(MPU_ADDR, ACCEL_X_OUT, 3*2, &rawData[0]);
          accelBuffer[0] = ((int16_t) rawData[0] << 8 | rawData[1]);
          accelBuffer[1] = ((int16_t) rawData[2] << 8 | rawData[3]);
          accelBuffer[2] = ((int16_t) rawData[4] << 8 | rawData[5]);
        }

        void readGyro() {
          uint8_t rawData[3*2];
          readI2C(MPU_ADDR, GYRO_X_OUT, 3*2, &rawData[0]);
          gyroBuffer[0] = ((int16_t) rawData[0] << 8 | rawData[1]);
          gyroBuffer[1] = ((int16_t) rawData[2] << 8 | rawData[3]);
          gyroBuffer[2] = ((int16_t) rawData[4] << 8 | rawData[5]);
        }

        void readMag() {
          uint8_t rawData[3*2 + 1];
          uint8_t rdy;
          readI2C(MAG_ADDR, MAG_READY, 1, &rdy);
          if (rdy & 0x01) {
            readI2C(MAG_ADDR, MAG_X_OUT, (3*2 + 1),  &rawData[0]);
            if (!(rawData[6] & 0x08)) {
              magBuffer[0] = ((int16_t) rawData[1] << 8 | rawData[0]);
              magBuffer[1] = ((int16_t) rawData[3] << 8 | rawData[2]);
              magBuffer[2] = ((int16_t) rawData[5] << 8 | rawData[4]);
            }
          }
        }
     public:
        float magBias[3], magBiasScale[3];
        int16_t gyroBias[3], accelBias[3];

        int16_t accelBuffer[3];
        int16_t gyroBuffer[3];
        int16_t magBuffer[3];
     
        MPU9250(TwoWire& _imu, uint8_t _aScale, uint8_t _gScale, uint8_t _mScale): imu(_imu), accelScale(_aScale), gyroScale(_gScale), magScale(_mScale) {
          switch (magScale) {
            case MFS_14BITS:
              magRes = 10.0f * 4912.0f/8190.0f;
              break;
            case MFS_16BITS:
              magRes = 10.0f * 4912.0f/32760.0f;
              break;
          }

          switch (accelScale) {
            case AFS_2G:
              accelRes = 2.0f/32768.0f;
              break;
            case AFS_4G:
              accelRes = 4.0f/32768.0f;
              break;
            case AFS_8G:
              accelRes = 8.0f/32768.0f;
              break;
            case AFS_16G:
              accelRes = 16.0f/32768.0f;
              break;
          }

          switch(gyroScale) {
            case GFS_250DPS:
              gyroRes = 250.0f/32768.0f;
              break;
            case GFS_500DPS:
              gyroRes = 500.0f/32768.0f;
              break;
            case GFS_1000DPS:
              gyroRes = 1000.0f/32768.0f;
              break;
            case GFS_2000DPS:
              gyroRes = 2000.0f/32768.0f;
              break;
          }
        }

        void begin() {
          initMPU();
          initMag();
        }

        void calibrateMPU() {
          uint16_t samples = 1000;
          
          uint8_t rawData[3*2];
          int16_t gyroSum[3] = { 0, 0, 0 };
          int16_t accelSum[3] = { 0, 0, 0 };

          for (uint16_t i = 0; i < samples; i++) {
            readI2C(MPU_ADDR, GYRO_X_OUT, 3*2, &rawData[0]);
            gyroSum[0] += ((int16_t) rawData[0] << 8 | rawData[1]);
            gyroSum[1] += ((int16_t) rawData[2] << 8 | rawData[3]);
            gyroSum[2] += ((int16_t) rawData[4] << 8 | rawData[5]);         
          }

          gyroBias[0] = (gyroSum[0] / samples) / 4;
          gyroBias[1] = (gyroSum[1] / samples) / 4;
          gyroBias[2] = (gyroSum[2] / samples) / 4;

          for (uint16_t i = 0; i < samples; i++) {
            readI2C(MPU_ADDR, ACCEL_X_OUT, 3*2, &rawData[0]);
            accelSum[0] += ((int16_t) rawData[0] << 8 | rawData[1]);
            accelSum[1] += ((int16_t) rawData[2] << 8 | rawData[3]);
            accelSum[2] += ((int16_t) rawData[4] << 8 | rawData[5]);         
          }

          accelBias[0] = (accelSum[0] / samples) / 8;
          accelBias[1] = (accelSum[1] / samples) / 8;
          accelBias[2] = ((1/(accelRes)) - (accelSum[2] / samples)) / 8;
        }

        void calibrateMag() {
          uint8_t samples = 128;

          int32_t magB[3], magS[3];
          int16_t magMax[3] = {-32767, -32767, -32767}, magMin[3] = {32767, 32767, 32767};
          int16_t magTemp[3];

          for (uint8_t i = 0; i < samples; i++) {
            readMag();
            magTemp[0] = magBuffer[0];
            magTemp[1] = magBuffer[1];
            magTemp[2] = magBuffer[2];

            for (uint8_t j = 0; j < 3; j++) {
              if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
              if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
            }
            delay(135);
          }

          magB[0] = (magMax[0] + magMin[0])/2;
          magB[1] = (magMax[1] + magMin[1])/2;
          magB[2] = (magMax[2] + magMin[2])/2;

          magBias[0] = (float)magB[0]*magRes*magCal[0];
          magBias[1] = (float)magB[1]*magRes*magCal[1];
          magBias[2] = (float)magB[2]*magRes*magCal[2];

          magS[0] = (magMax[0] - magMin[0])/2;
          magS[1] = (magMax[1] - magMin[1])/2;
          magS[2] = (magMax[2] - magMin[2])/2;

          float avg = magS[0] + magS[1] + magS[2];
          avg /= 3.0f;

          magBiasScale[0] = avg/((float)magS[0]);
          magBiasScale[1] = avg/((float)magS[1]);
          magBiasScale[2] = avg/((float)magS[2]);
        }

        float accelX() {
          readAccel();
          return ((float)(accelBuffer[0] - accelBias[0])*accelRes);
        }

        float accelY() {
          readAccel();
          return ((float)(accelBuffer[1] - accelBias[1])*accelRes);
        }

        float accelZ() {
          readAccel();
          return ((float)(accelBuffer[2] - accelBias[2])*accelRes);
        }

        float gyroX() {
          readGyro();
          return ((float)(gyroBuffer[0] - gyroBias[0])*gyroRes);
        }

        float gyroY() {
          readGyro();
          return ((float)(gyroBuffer[1] - gyroBias[1])*gyroRes);
        }

        float gyroZ() {
          readGyro();
          return ((float)(gyroBuffer[2] - gyroBias[2])*gyroRes);
        }

        float magX() {
          readMag();
          return ((float)(magBuffer[0])*magRes*magCal[0]-magBias[0])*magBiasScale[0];
        }

        float magY() {
          readMag();
          return ((float)(magBuffer[1])*magRes*magCal[1]-magBias[1])*magBiasScale[1];
        }

        float magZ() {
          readMag();
          return ((float)(magBuffer[2])*magRes*magCal[2]-magBias[2])*magBiasScale[2];
        }

        uint8_t magWhoAmI() {
          uint8_t data;
          readI2C(MAG_ADDR, MAG_WHO_AM_I, 1, &data);
          return data;
        }
    };
}

#endif // _MPU9250_H