#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <stdio.h>
#include <arduino.h>
#include "../RadioCommunication/RadioComms.h"
#include "../IMUController/IMUController.h"

//Rate PID vals
#define PCoeffRoll 1.2
#define PCoeffPitch 1.2
#define PCoeffYaw 1.5

#define ICoeffRoll 0.5  //3.5 drifting 2.5 too
#define ICoeffPitch 0.5 //3.5 drifting 2.5 too
#define ICoeffYaw 0.5 // prev 12.0, 6.0 - 12 has drift, 6 less drift, to document and provide graphs for these values

#define DCoeffRoll 0.030
#define DCoeffPitch 0.030
#define DCoeffYaw 0.015 // prev 0 - has drift


//Angle PID vals

#define PCoeffRollAngle 2.5
#define PCoeffPitchAngle 2.5

#define ICoeffRollAngle 0.1
#define ICoeffPitchAngle 0.1

#define DCoeffRollAngle 0.0
#define DCoeffPitchAngle 0.0

//PID constants tuning after test stand testing
/*
-Set1 values (P,I,D): 0.6, 0.6, 0.2 ... ; Response sluggish, high over correction that results in oscilation on roll and pitch, drifting of roll pitch
-Set2 values (P,I,D): 0.8, 0.8, 1.5 ... ; Response feels more sluggish, slightly less over correction but will still result in oscilation, drift still the same, but the drone feels controllable but needs anticipated commands to stabilize from oscilation
-Set3 values (P,I,D): 1.0, 1.0, 1.5 ... ; Response still seems sluggish, maybe slightly swifter, on the over correction we have improvements, oscilations have decreased in amplitude, but still increases over time, along with drift, we have yaw drift visible now.
-Set4 values (P,I,D): 1.2, 1.2, 1.5 ... ; Response seems to have improved a bit, but could be better, the oscilations have decreased in amplitude, and atenuate slowly only at higher motor speeds at 50-60% or more, lower than that and we oscilate. Now I noticed heavy drift on yaw axis.
*/
/*
//Rate PID vals
#define PCoeffRoll    0.6, 0.8, 1.0, 1.2
#define PCoeffPitch   0.6, 0.8, 1.0, 1.2
#define PCoeffYaw     2.0, 1.5, 1.5, 1.5

#define ICoeffRoll    3.5, 2.0, 1.0, 0.5  
#define ICoeffPitch   3.5, 2.0, 1.0, 0.5
#define ICoeffYaw     4.0, 3.0, 2.0, 0.5

#define DCoeffRoll    0.015, 0.025, 0.025, 0.030
#define DCoeffPitch   0.015, 0.025, 0.025, 0.030
#define DCoeffYaw     0.005, 0.01 , 0.01 , 0.015

//Angle PID vals
#define PCoeffRollAngle  2.0, 2.5, 2.5, 2.5
#define PCoeffPitchAngle 2.0, 2.5, 2.5, 2.5

#define ICoeffRollAngle  0.0, 0.1, 0.1, 0.1
#define ICoeffPitchAngle 0.0, 0.1, 0.1, 0.1

#define DCoeffRollAngle  0.0, 0.0, 0.0, 0.0
#define DCoeffPitchAngle 0.0, 0.0, 0.0, 0.0

*/



#define dT 0.004 //Sample time 250 HZ for now, try later 500HZ = 0.002 dT so we have dT <= 10*50 HZ
#define AW_Limit_Int 400.0 //Integral anti wind-up limit = 80,  = 20% Max throttle range = 400, or 400 for input range=2000

#define FC_THROTTLE_MAX 2000.0
#define FC_THROTTLE_MIN 1000.0
#define FC_THROTTLE_IDLE 1200.0 //Keeps motors at idle, prevents stopping of the motor if requested by PID
#define FC_THROTTLE_CUT_OFF 1000.0
#define FC_THROTTLE_CUT_OFF_TRESHOLD 1060.0

#define QUICK_DEBUG_FC 0

extern uint32_t ControlLoopTimer;

extern float DesiredRollRate, DesiredPichRate, DesiredYawRate,FC_InputThrottle;
extern float ErrorRollRate, ErrorPitchRate, ErrorYawRate;
extern float InputRoll, InputPitch, InputYaw;
extern float RollIMU, PitchIMU, YawIMU;
extern float PrevErrorRollRate, PrevErrorPitchRate, PrevErrorYawRate;
extern float PrevItermRoll, PrevItermPitch, PrevItermYaw;
extern float PIDOutput[3];
extern float MotorCommands[4];
extern float AccRollAngle, AccPitchAngle;
extern float RollRateInputRC, PitchRateInputRC, YawRateInputRC;

extern float PRollRate, PPitchRate, PYawRate;
extern float IRollRate, IPitchRate, IYawRate;
extern float DRollRate, DPitchRate, DYawRate;

//Kalman vars

extern float KalmanRollAngle, KalmanUncertaintyRollAngle; //(-2,+2) uncertainty, init angle 0
extern float KalmanPitchangle, KalmanUncertaintyPitchAngle;
extern float Kalman1DOutput[2]; //angle prediction, uncertainty prediction

//Angle mode vars, pid angle, outside loop

extern float DesiredRollAngle,DesiredPitchAngle;
extern float ErrorRollAngle,ErrorPitchAngle;

extern float PrevErrorRollAngle, PrevErrorPitchAngle;
extern float PrevItermRollAngle, PrevItermPitchAngle;

extern float PRollAngle, PPitchAngle; 
extern float IRollAngle, IPitchAngle;
extern float DRollAngle, DPitchAngle;

void pid_calc(float Error, float P, float I, float D, float PrevError, float PrevIterm);
void pid_reset(void);
void pid_processing_rate(void);
void pid_processing_angle(void);
void pid_processing_fc_rate_mode(void);
void pid_processing_fc_angle_mode(void);
void FlightControllerProcessing(void);
void FC_Inputs(void);
void QuadcopterFlightDynamics(void);
void GetFcMotorCommands(float *Motor1Command_FR,float *Motor2Command_RR,float *Motor3Command_RL,float *Motor4Command_FL);
void KalmanFilter1D(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void KalmanFilter1dProcessing(void);
#endif // FLIGHTCONTROLLER_H