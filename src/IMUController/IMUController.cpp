#include "IMUController.h"

float IMU_RollRate = 0, IMU_PitchRate = 0, IMU_YawRate = 0;
float IMU_AccX = 0, IMU_AccY = 0, IMU_AccZ = 0;
float IMU_RollAngle = 0, IMU_PitchAngle = 0;
float CalibrationRoll = 0, CalibrationPitch = 0, CalibrationYaw = 0;
int gyro_calibration_samples = 2500; // Nr. of samples read on gyro init, in order to calibrate the axes to reduce possible offsets

void IMU_start(void)
{ 
    // Start MPU-9250 in power mode - activating internal clock
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void IMU_Init(void)
{
    //Moved in main Setup ToggleRedLedON();
    Serial.println("Drone calibration started, do not move the drone!");
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    gyro_init();
    acc_init();
    IMU_start();
    gyro_calibration();
    Serial.println("Drone calibration finished!");
    //Moved in main Setup loop ToggleGreenLedON();
}

void PrintGyroData(void)
{
#if GYROSCOPE_ACTIVE != false
    //gyro_read();
    Serial.print("Roll_Rate[°/s]:");
    Serial.print(IMU_RollRate);
    Serial.print(",");
    Serial.print(" Pitch_Rate[°/s]:");
    Serial.print(IMU_PitchRate);
    Serial.print(",");
    Serial.print(" Yaw_Rate[°/s]:");
    Serial.println(IMU_YawRate);
#endif
}
void PrintAccData(void)
{
#if ACCELEROMETER_ACTIVE != false
    //acc_read();
    Serial.print("AccelerationX[g]:");
    Serial.print(IMU_AccX);
    Serial.print(",");
    Serial.print(" AccelerationY[g]:");
    Serial.print(IMU_AccY);
    Serial.print(",");
    Serial.print(" AccelerationZ[g]:");
    Serial.println(IMU_AccZ);
#endif
}
void PrintAccOrientationAngles(void)
{
#if ACCELEROMETER_ACTIVE != false
    //acc_read();
    Serial.print("RollAngle[°]:");
    Serial.print(IMU_RollAngle);
    Serial.print(",");
    Serial.print(" PitchAngle[°]:");
    Serial.println(IMU_PitchAngle);
#endif
}

void gyro_init(void)
{
    Wire.beginTransmission(0x68);
    // Digital Low pass filter DLPF for Gyroscope 10Hz
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    // Set Full Scale Range Gyroscope +/- 500deg/s for FS_SEL=1
    Wire.beginTransmission(0x68); //@@@@@@@@@@@@@@@@@@@ to check I2C transmission protocol in datasheet
    Wire.write(0x1B);             // select register 0x1B
    Wire.write(0x08);             // write value 0x08 in the register
    Wire.endTransmission();
}
void acc_init(void)
{
    // Digital Low pass filter DLPF for Accelerometer 10Hz
    Wire.beginTransmission(0x68);
    Wire.write(0x1D);
    Wire.write(0x05);
    Wire.endTransmission();
    // Set full scale range for accelerometer  +/-8g
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
}
void acc_read(void)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccX = Wire.read() << 8 | Wire.read();
    int16_t AccY = Wire.read() << 8 | Wire.read();
    int16_t AccZ = Wire.read() << 8 | Wire.read();
    IMU_AccX = (float)AccX / 4096 - 0.007; // 4096 scale factor for +/- 8g
    IMU_AccY = (float)AccY / 4096 + 0.015; //calibration values derived manually for now
    IMU_AccZ = (float)AccZ / 4096;

    IMU_RollAngle = atan(IMU_AccY / sqrt(IMU_AccX * IMU_AccX + IMU_AccZ * IMU_AccZ)) * DEGREE;
    IMU_PitchAngle = -atan(IMU_AccX / sqrt(IMU_AccY * IMU_AccY + IMU_AccZ * IMU_AccZ)) * DEGREE;

    /*
    IMU_RollAngle = atan2(IMU_AccY , sqrt(IMU_AccX * IMU_AccX + IMU_AccZ * IMU_AccZ)) * DEGREE;
    IMU_PitchAngle = -atan2(IMU_AccX , sqrt(IMU_AccY * IMU_AccY + IMU_AccZ * IMU_AccZ)) * DEGREE;
    */
    //No difference noticed, maybe due to the formula when tilting to 90 degrees and above the tilt decreases with both methods
    //maybe use atan2 to prevent anomallies ? or to get -pi to pi range instead of -pi/2 to pi/2, does it matter ?
}
void gyro_read(void)
{
    // Read Gyro Measurements; X,Y,Z
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    IMU_RollRate = (float)GyroX / 65.5 - CalibrationRoll;   // Sensitivity Scale factor for FS_SEL=1 is 65.6 LSB/deg/s
    IMU_PitchRate = (float)GyroY / 65.5 - CalibrationPitch; //@@@@@@@@@@@@@@@@@@ to check where it says to divide the value by scale factor to be sure in datasheet
    IMU_YawRate = (float)GyroZ / 65.5 - CalibrationYaw;
}
void gyro_calibration(void)
{
    float SumCalibrationRoll = 0, SumCalibrationPitch = 0, SumCalibrationYaw = 0;
    for (int i = 0; i < gyro_calibration_samples; i++)
    {
        gyro_read();
        SumCalibrationRoll += IMU_RollRate;
        SumCalibrationPitch += IMU_PitchRate;
        SumCalibrationYaw += IMU_YawRate;
        delay(1);
    }
    CalibrationRoll = (float)SumCalibrationRoll / gyro_calibration_samples;
    CalibrationPitch = (float)SumCalibrationPitch / gyro_calibration_samples;
    CalibrationYaw = (float)SumCalibrationYaw / gyro_calibration_samples;
}

void GetGyroRateMeasurements(float *rollRate, float *pitchRate, float *yawRate){
    *rollRate=IMU_RollRate;
    *pitchRate=IMU_PitchRate;
    *yawRate=IMU_YawRate;
}

void GetAccAngleMeasurements(float *accRollAngle, float *accPitchAngle){
    *accRollAngle=IMU_RollAngle;
    *accPitchAngle=IMU_PitchAngle;
}

void IMUProcessing(void){
    gyro_read();
    acc_read();
}
void acc_calibration(void)
{
    // For possible implementation in the future
    // for future implementation, dynamic calibration: roll sensor along all axis, measure max G for each and calibrate all to 1g, do a state machine for each axis
}