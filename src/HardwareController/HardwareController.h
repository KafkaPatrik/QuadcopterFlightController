#ifndef HARDWARECONTROLLER_H
#define HARDWARECONTROLLER_H

#include <arduino.h>


#define HC_OUTPUT_RED_LED_PIN_D 16 //5 before
#define HC_OUTPUT_GREEN_LED_PIN_D 17 //6
#define TURN_LED_BLINK_ON true
#define TURN_LED_BLINK_OFF false
#define HC_LOOP_TIMEBASE 20 //miliseconds
#define HC_SECOND 50 //1 seconds= 20ms * 50 loops

//To do: check LED calls from IMU, check in the final system if we need to call it earlier, in comms for example
//Maybe add some arbitrator

extern int toggleCounter;
extern boolean redLedState;
extern boolean redLedBlinkState;
extern boolean greenLedState;
extern boolean dualLedBlinkState;

void HardwareController_Init(void);
void LedControllerInit(void);
void ToggleRedLedON(void);
void ToggleGreenLedON(void);
void TurnOffLeds(void);
void ToggleRedLed(void);
void SetRedLedToggling(boolean RedLedTogglingState);
void SetDualLedToggling(boolean DualLedTogglingState);

void HardwareControllerProcessing(void);

#endif //HARDWARECONTROLLER_H