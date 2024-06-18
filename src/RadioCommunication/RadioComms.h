#ifndef RADIOCOMMS_H
#define RADIOCOMMS_H

#include <PulsePosition.h>
#include <arduino.h>

#define RC_PPM_INPUT_PIN 15
#define RC_CH1_ROLL 0
#define RC_CH2_PITCH 1
#define RC_CH3_THROTTLE 2
#define RC_CH4_YAW 3
#define RC_CH5_AUX_VRA 4 //unused
#define RC_CH6_AUX_VRB 5 //unused
#define RC_CH7_AUX1 6
#define RC_CH8_AUX2 7   //unused, PPM can only transmit up to 8 channels :(
#define RC_CH9_AUX3 8   //unused
#define RC_CH10_AUX4 9  //unused

#define RC_SWITCH_TRESH 1400

extern PulsePositionInput ReceiverInput;
extern float ReceiverChannelValue[10];
extern int ChannelNumber;
extern boolean Aux1SwState;

void RadioCommsInit(void);
void read_receiver(void);
float GetReceiverChannelValue(int ChannelIndex);
int GetNrOfChannels(void);
void PrintRCInputs(void);
void SwitchStateProcessing(void);
boolean GetAuxSwitchState(void);
void RadioCommsProcessing(void);

#endif // RADIOCOMMS_H