#include "FlightController.h"

uint32_t ControlLoopTimer;

float DesiredRollRate = 0.0, DesiredPichRate = 0.0, DesiredYawRate = 0.0, FC_InputThrottle = 0.0;
float ErrorRollRate = 0.0, ErrorPitchRate = 0.0, ErrorYawRate = 0.0;
float InputRoll = 0.0, InputPitch = 0.0, InputYaw = 0.0;
float RollIMU = 0.0, PitchIMU = 0.0, YawIMU = 0.0;
float PrevErrorRollRate = 0.0, PrevErrorPitchRate = 0.0, PrevErrorYawRate = 0.0;
float PrevItermRollRate = 0.0, PrevItermPitchRate = 0.0, PrevItermYawRate = 0.0;
float PIDOutput[3] = {0, 0, 0};
float MotorCommands[4] = {0, 0, 0, 0};
float AccRollAngle = 0.0, AccPitchAngle = 0.0;
float RollRateInputRC=0.0, PitchRateInputRC=0.0, YawRateInputRC=0.0;

float PRollRate = PCoeffRoll, PPitchRate = PCoeffPitch, PYawRate = PCoeffYaw;
float IRollRate = ICoeffRoll, IPitchRate = ICoeffPitch, IYawRate = ICoeffYaw;
float DRollRate = DCoeffRoll, DPitchRate = DCoeffPitch, DYawRate = DCoeffYaw;

// Kalman Vars

float KalmanRollAngle = 0.0, KalmanUncertaintyRollAngle = 2.0 * 2.0; //(-2,+2) uncertainty, init angle 0
float KalmanPitchAngle = 0.0, KalmanUncertaintyPitchAngle = 2.0 * 2.0;
float Kalman1DOutput[2] = {0.0, 0.0};

// Angle mode vars

extern float KalmanRollAngle, KalmanUncertaintyRollAngle; //(-2,+2) uncertainty, init angle 0
extern float KalmanPitchangle, KalmanUncertaintyPitchAngle;
extern float Kalman1DOutput[2]; // angle prediction, uncertainty prediction

// Angle mode vars, pid angle, outside loop

float DesiredRollAngle= 0.0, DesiredPitchAngle= 0.0;
float ErrorRollAngle= 0.0, ErrorPitchAngle= 0.0;

float PrevErrorRollAngle= 0.0, PrevErrorPitchAngle= 0.0;
float PrevItermRollAngle= 0.0, PrevItermPitchAngle= 0.0;

float PRollAngle=PCoeffRollAngle, PPitchAngle=PCoeffPitchAngle;
float IRollAngle=ICoeffRollAngle, IPitchAngle=ICoeffPitchAngle;
float DRollAngle=DCoeffRollAngle, DPitchAngle=DCoeffPitchAngle;

void pid_calc(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * dT / 2.0;
    float Dterm = D * (Error - PrevError) * 2.0 / dT;
    float PID_Out;

    if (Iterm > AW_Limit_Int)
        Iterm = AW_Limit_Int;
    else if (Iterm < -AW_Limit_Int)
        Iterm = -AW_Limit_Int;

    PID_Out = Pterm + Iterm + Dterm;

    if (PID_Out > AW_Limit_Int)
        PID_Out = AW_Limit_Int;
    else if (PID_Out < -AW_Limit_Int)
        PID_Out = -AW_Limit_Int;

    PIDOutput[0] = PID_Out;
    PIDOutput[1] = Error;
    PIDOutput[2] = Iterm;
}

void pid_reset(void)
{
    PrevErrorRollRate = 0.0;
    PrevErrorPitchRate = 0.0;
    PrevErrorYawRate = 0.0;
    PrevItermRollRate = 0.0;
    PrevItermPitchRate = 0.0;
    PrevItermYawRate = 0.0;

    PrevErrorRollAngle = 0.0;
    PrevErrorPitchAngle = 0.0;
    PrevItermRollAngle = 0.0;
    PrevItermPitchAngle = 0.0;
}

void FC_Inputs(void)
{
    RollRateInputRC = GetReceiverChannelValue(RC_CH1_ROLL);
    PitchRateInputRC = GetReceiverChannelValue(RC_CH2_PITCH);
    FC_InputThrottle = GetReceiverChannelValue(RC_CH3_THROTTLE);
    YawRateInputRC = GetReceiverChannelValue(RC_CH4_YAW);
    GetGyroRateMeasurements(&RollIMU, &PitchIMU, &YawIMU);
    GetAccAngleMeasurements(&AccRollAngle, &AccPitchAngle);
}

void pid_processing_rate(void)
{
    ErrorRollRate = DesiredRollRate - RollIMU;
    ErrorPitchRate = DesiredPichRate - PitchIMU;
    ErrorYawRate = DesiredYawRate - YawIMU;
    // Roll pid
    pid_calc(ErrorRollRate, PRollRate, IRollRate, DRollRate, PrevErrorRollRate, PrevItermRollRate);
    InputRoll = PIDOutput[0];
    PrevErrorRollRate = PIDOutput[1];
    PrevItermRollRate = PIDOutput[2];
    // Pitch pid
    pid_calc(ErrorPitchRate, PPitchRate, IPitchRate, DPitchRate, PrevErrorPitchRate, PrevItermPitchRate);
    InputPitch = PIDOutput[0];
    PrevErrorPitchRate = PIDOutput[1];
    PrevItermPitchRate = PIDOutput[2];
    // Yaw pid
    pid_calc(ErrorYawRate, PYawRate, IYawRate, DYawRate, PrevErrorYawRate, PrevItermYawRate);
    InputYaw = PIDOutput[0];
    PrevErrorYawRate = PIDOutput[1];
    PrevItermYawRate = PIDOutput[2];
}

void pid_processing_angle(void){
    DesiredRollAngle=0.10*(RollRateInputRC-1000) - 50;
    DesiredPitchAngle=0.10*(PitchRateInputRC-1000) - 50;
    DesiredYawRate = (-1.0) * (0.15 * (YawRateInputRC - 1000) - 75); //-1 to invert the range, since at yaw it is backwards, also related to input 1000 = -75 was backwards in practice, this is because to yaw left, we need to give motors 2,4 + power, which is correct in the dynamics relation, but the yaw input when we move the stick to the left will be mapped to - 75 yaw rate and the PID will try to match that and we get reversed movement, this is why we need -1


    ErrorRollAngle = DesiredRollAngle - KalmanRollAngle;
    ErrorPitchAngle = DesiredPitchAngle - KalmanPitchAngle;
#if QUICK_DEBUG_FC!=0
    Serial.print("ErrorRollAngle:");
    Serial.print(ErrorRollAngle);
    Serial.print(",");
    Serial.print(" InputRoll:");
    Serial.print(DesiredRollAngle);
    Serial.print(",");
    Serial.print(" KalmanRoll:");
    Serial.println(KalmanRollAngle);

    Serial.print("ErrorPitchAngle:");
    Serial.print(ErrorPitchAngle);
    Serial.print(",");
    Serial.print(" InputPitch:");
    Serial.print(DesiredPitchAngle);
    Serial.print(",");
    Serial.print(" KalmanPitch:");
    Serial.println(KalmanPitchAngle);
#endif

    // Roll pid
    pid_calc(ErrorRollAngle, PRollAngle, IRollAngle, DRollAngle, PrevErrorRollAngle, PrevItermRollAngle);
    DesiredRollRate = PIDOutput[0];
    PrevErrorRollAngle = PIDOutput[1];
    PrevItermRollAngle = PIDOutput[2];
    // Pitch pid
    pid_calc(ErrorPitchAngle, PPitchAngle, IPitchAngle, DPitchAngle, PrevErrorPitchAngle, PrevItermPitchAngle);
    DesiredPichRate = PIDOutput[0];
    PrevErrorPitchAngle = PIDOutput[1];
    PrevItermPitchAngle = PIDOutput[2];

}

void pid_processing_fc_rate_mode(void){
    //Only needed in rate mode FC, in angle mode we map the inputs there
    DesiredRollRate = 0.15 * (RollRateInputRC - 1000) - 75; // map 1000 to 2000 inputs to -75 to 75 roll rate
    DesiredPichRate = 0.15 * (PitchRateInputRC - 1000) - 75;
    DesiredYawRate = (-1.0) * (0.15 * (YawRateInputRC - 1000) - 75); //-1 to invert the range, since at yaw it is backwards, also related to input 1000 = -75 was backwards in practice, this is because to yaw left, we need to give motors 2,4 + power, which is correct in the dynamics relation, but the yaw input when we move the stick to the left will be mapped to - 75 yaw rate and the PID will try to match that and we get reversed movement, this is why we need -1
    pid_processing_rate();
}

void pid_processing_fc_angle_mode(void){
    pid_processing_angle();
    pid_processing_rate();
}

void QuadcopterFlightDynamics(void)
{
    if (FC_InputThrottle > 1800)  // Instead maybe try to map the throttle to 1000-1800 to get a smooth response
        FC_InputThrottle = 1800;  // Limit throttle to 80 because we need additional throttle for the control (roll,pitch,yaw)
#if QUICK_DEBUG_FC!=0    
        Serial.print("FC ");
        Serial.print("Throttle:");
        Serial.print(FC_InputThrottle);
        Serial.print(",");
        Serial.print(" Roll:");
        Serial.print(InputRoll);
        Serial.print(",");
        Serial.print(" Pitch:");
        Serial.print(InputPitch);
        Serial.print(",");
        Serial.print(" Yaw:");
        Serial.println(InputYaw);
#endif    

    MotorCommands[0] = FC_InputThrottle - InputRoll - InputPitch - InputYaw; // FR - clockwise numerotation here
    MotorCommands[1] = FC_InputThrottle - InputRoll + InputPitch + InputYaw; // RR
    MotorCommands[2] = FC_InputThrottle + InputRoll + InputPitch - InputYaw; // RL
    MotorCommands[3] = FC_InputThrottle + InputRoll - InputPitch + InputYaw; // FL

    MotorCommands[0] = MotorCommands[0] > FC_THROTTLE_MAX ? FC_THROTTLE_MAX : MotorCommands[0];
    MotorCommands[1] = MotorCommands[1] > FC_THROTTLE_MAX ? FC_THROTTLE_MAX : MotorCommands[1];
    MotorCommands[2] = MotorCommands[2] > FC_THROTTLE_MAX ? FC_THROTTLE_MAX : MotorCommands[2];
    MotorCommands[3] = MotorCommands[3] > FC_THROTTLE_MAX ? FC_THROTTLE_MAX : MotorCommands[3];

    MotorCommands[0] = MotorCommands[0] < FC_THROTTLE_IDLE ? FC_THROTTLE_IDLE : MotorCommands[0];
    MotorCommands[1] = MotorCommands[1] < FC_THROTTLE_IDLE ? FC_THROTTLE_IDLE : MotorCommands[1];
    MotorCommands[2] = MotorCommands[2] < FC_THROTTLE_IDLE ? FC_THROTTLE_IDLE : MotorCommands[2];
    MotorCommands[3] = MotorCommands[3] < FC_THROTTLE_IDLE ? FC_THROTTLE_IDLE : MotorCommands[3];

    // Also added here emergency shut down switch
    if (FC_InputThrottle < FC_THROTTLE_CUT_OFF_TRESHOLD || GetAuxSwitchState())
    {
        MotorCommands[0] = FC_THROTTLE_CUT_OFF;
        MotorCommands[1] = FC_THROTTLE_CUT_OFF;
        MotorCommands[2] = FC_THROTTLE_CUT_OFF;
        MotorCommands[3] = FC_THROTTLE_CUT_OFF;
        pid_reset();
    }
    #if QUICK_DEBUG_FC!=0
        Serial.print("FC_MOTORS ");
        Serial.print("Motor_1:");
        Serial.print(MotorCommands[0]);
        Serial.print(",");
        Serial.print(" Motor_2:");
        Serial.print(MotorCommands[1]);
        Serial.print(",");
        Serial.print(" Motor_3:");
        Serial.print(MotorCommands[2]);
        Serial.print(",");
        Serial.print(" Motor_4:");
        Serial.println(MotorCommands[3]);
    #endif
}

void GetFcMotorCommands(float *Motor1Command_FR, float *Motor2Command_RR, float *Motor3Command_RL, float *Motor4Command_FL)
{
    *Motor1Command_FR = MotorCommands[0];
    *Motor2Command_RR = MotorCommands[1];
    *Motor3Command_RL = MotorCommands[2];
    *Motor4Command_FL = MotorCommands[3];
}

void KalmanFilter1D(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    float KalmanGain = 0.0;
    KalmanState = KalmanState + dT * KalmanInput;                                   // Gyro integration happens here
    KalmanUncertainty = KalmanUncertainty + dT * dT * 4.0 * 4.0;                    //(-4,4) angle prediction uncertainty/deviation
    KalmanGain = KalmanUncertainty * (1.0 / (1.0 * KalmanUncertainty + 3.0 * 3.0)); // (-3,3) accelerometer uncertainty
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1.0 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void KalmanFilter1dProcessing(void)
{

    KalmanFilter1D(KalmanRollAngle, KalmanUncertaintyRollAngle, RollIMU, AccRollAngle);
    KalmanRollAngle = Kalman1DOutput[0];
    KalmanUncertaintyRollAngle = Kalman1DOutput[1];
    KalmanFilter1D(KalmanPitchAngle, KalmanUncertaintyPitchAngle, PitchIMU, AccPitchAngle);
    KalmanPitchAngle = Kalman1DOutput[0];
    KalmanUncertaintyPitchAngle = Kalman1DOutput[1];

#if QUICK_DEBUG_FC!=0
    Serial.print("Roll_Angle[°]:");
    Serial.print(KalmanRollAngle);
    Serial.print(",");
    Serial.print(" Pitch_Angle[°]:");
    Serial.println(KalmanPitchAngle);
#endif
}

void FlightControllerProcessing(void)
{
    FC_Inputs();
    KalmanFilter1dProcessing();
    //pid_processing_fc_rate_mode();
    pid_processing_fc_angle_mode();
    QuadcopterFlightDynamics();
}