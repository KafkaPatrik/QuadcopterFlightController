#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <arduino.h>
#include "../RadioCommunication/RadioComms.h"
#include "../HardwareController/HardwareController.h"
#include "../FlightController/FlightController.h"

#define MC_OUTPUT_PWM_ESC_MOTOR_FR_PIN 2  //soldering mistake, solved in sw, this should have been 1
#define MC_OUTPUT_PWM_ESC_MOTOR_RR_PIN 1
#define MC_OUTPUT_PWM_ESC_MOTOR_RL_PIN 3
#define MC_OUTPUT_PWM_ESC_MOTOR_FL_PIN 4
#define MC_OUTPUT_PWM_FREQ 50
#define MC_OUTPUT_PWM_RESOLUTION 12
#define MC_OUTPUT_PWM_RESOLUTION_SCALING ((double) 4095 / 4000) // full scale range of pwm 12 bits res over 4000 us pwm period
#define MC_OUTPUT_PWM_THROTTLE_MIN_US 200.0 // ESC only seems to accept 5% to 10% duty cycle, similar to what the Receiver outputs on the PWM pin
#define MC_OUTPUT_PWM_THROTTLE_MAX_US 400.0
#define MC_THROTTLE_CUT_OFF 200.0


extern float InputThrottle;
extern boolean InputAux1SwState;
extern float MotorInputs[4];

void MotorControllerInit(void);
void MotorControllerInputs(void);
void MotorControllerProcessing(void);

#endif //MOTORCONTROLLER_H