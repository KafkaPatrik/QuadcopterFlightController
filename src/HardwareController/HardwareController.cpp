#include "HardwareController.h"
#include "../PowerManagement/PowerManagement.h"
#include "../MotorController/MotorController.h"

int toggleCounter = 0;
boolean redLedState = false;
boolean greenLedState=false;
boolean redLedBlinkState = false;
boolean dualLedBlinkState=false;

void HardwareController_Init(void)
{
    toggleCounter = 0;
    LedControllerInit();
}

void LedControllerInit(void)
{
    pinMode(HC_OUTPUT_RED_LED_PIN_D, OUTPUT);
    pinMode(HC_OUTPUT_GREEN_LED_PIN_D, OUTPUT);
}

void ToggleRedLedON(void)
{
    if(redLedState!=true){
    digitalWrite(HC_OUTPUT_RED_LED_PIN_D, HIGH);
    digitalWrite(HC_OUTPUT_GREEN_LED_PIN_D, LOW);
    redLedState=true;
    greenLedState=false;
    }
}

void ToggleGreenLedON(void)
{
    if(greenLedState!=true && GetBatteryLowVoltageFlag()!=true){
    digitalWrite(HC_OUTPUT_RED_LED_PIN_D, LOW);
    digitalWrite(HC_OUTPUT_GREEN_LED_PIN_D, HIGH);
    redLedState=false;
    greenLedState=true;
    }
}

void TurnOffLeds(void)
{
    digitalWrite(HC_OUTPUT_RED_LED_PIN_D, LOW);
    digitalWrite(HC_OUTPUT_GREEN_LED_PIN_D, LOW);
        redLedState=false;
        greenLedState=false;
}

void ToggleRedLed(void)
{
// Serial.println("entered toggle");
    if (redLedBlinkState)
    {
       // Serial.print("entered toggle condition: ");
       //Serial.println(toggleCounter);
        toggleCounter++;            // set counter to 50 from 10 to real time for testing, it works fine, maybe lower it to 20 or 30 for critical battery level
        if (toggleCounter >= 50) // also works 5 or 10 frequency with 50 ms main loop time //10000 no delay, blink frequency
        {         
           // Serial.print(" entered toggle condition counter: ");
            //Serial.println(toggleCounter);
            if (redLedState)
            {
                TurnOffLeds();
            }
            else
            {
                ToggleRedLedON();
            }
            toggleCounter = 0;
        }
    }
    else 
    {
        ToggleGreenLedON();
        toggleCounter=0;
    }
}

void ToggleDualLed(void)
{
    //Serial.print("Dual blink state: ");
    //Serial.println(dualLedBlinkState);
    if (dualLedBlinkState)
    {
       // Serial.print("Toggle counter ");
       //Serial.println(toggleCounter);
        toggleCounter++;
        if (toggleCounter >= 100)
        {
            if (redLedState)
            {
                ToggleGreenLedON();
                //Serial.println("Switched to Green LED");
            }
            else
            {
                ToggleRedLedON();
                //Serial.println("Switched to Red LED");
            }
            //redLedState = !redLedState; updated calls to track states, no longer needed
            toggleCounter = 0;
        }
    }
    else{
        TurnOffLeds();
        toggleCounter = 0;
    }
}

void SetRedLedToggling(boolean RedLedTogglingState)
{
    redLedBlinkState = RedLedTogglingState;
}

void SetDualLedToggling(boolean DualLedTogglingState)
{
    dualLedBlinkState = DualLedTogglingState;
}

void HardwareControllerProcessing(void)
{
    //Serial.print("redLedBlinkState: ");
    //Serial.print(redLedBlinkState);
    //Serial.print(" dualLedBlinkState: ");
    //Serial.println(dualLedBlinkState);
    if (redLedBlinkState && GetAuxSwitchState()!=true)
    ToggleRedLed();
    if (dualLedBlinkState)
    ToggleDualLed();
}