#ifndef IMUCONTROLLER_H
#define IMUCONTROLLER_H

#include <stdio.h>
#include <Wire.h>
#include "../HardwareController/HardwareController.h"

#define GYROSCOPE_ACTIVE true
#define ACCELEROMETER_ACTIVE true
#define DEGREE 180 / 3.14159 // Radian to degree
//Pins used are 18-SDA0, 19-SCL0, connected to I2C pins of IMU module

extern float IMU_RollRate, IMU_PitchRate, IMU_YawRate;
extern float IMU_AccX, IMU_AccY, IMU_AccZ;
extern float IMU_RollAngle, IMU_PitchAngle;
extern float CalibrationRoll, CalibrationPitch, CalibrationYaw;
extern int gyro_calibration_samples;

void IMU_Init(void);
void gyro_init(void);
void gyro_read(void);
void gyro_calibration(void);
void acc_init(void);
void acc_read(void);
void acc_calibration(void); // placeholder, for future implementation, dynamic calibration: roll sensor along all axis, measure max G for each and calibrate all to 1g, do a state machine for each axis
void PrintAccData(void);
void PrintGyroData(void);
void PrintAccOrientationAngles(void);
void IMU_start(void);
void IMUProcessing(void);
void GetGyroRateMeasurements(float *rollRate, float *pitchRate, float *yawRate);
void GetAccAngleMeasurements(float *accRollAngle, float *accPitchAngle);

#endif // IMUCONTROLLER_H