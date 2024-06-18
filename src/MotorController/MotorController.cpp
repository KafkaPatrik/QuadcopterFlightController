#include "MotorController.h"

float InputThrottle = 0;
boolean InputAux1SwState = false;
float MotorInputs[4] = {0, 0, 0, 0};

void MotorControllerInit(void)
{
    analogWriteFrequency(MC_OUTPUT_PWM_ESC_MOTOR_FR_PIN, MC_OUTPUT_PWM_FREQ);
    analogWriteFrequency(MC_OUTPUT_PWM_ESC_MOTOR_RR_PIN, MC_OUTPUT_PWM_FREQ);
    analogWriteFrequency(MC_OUTPUT_PWM_ESC_MOTOR_RL_PIN, MC_OUTPUT_PWM_FREQ);
    analogWriteFrequency(MC_OUTPUT_PWM_ESC_MOTOR_FL_PIN, MC_OUTPUT_PWM_FREQ);
    analogWriteResolution(MC_OUTPUT_PWM_RESOLUTION);
    delay(250);
    InputThrottle = 0;
    InputAux1SwState = false;
    read_receiver();
    InputThrottle = GetReceiverChannelValue(RC_CH3_THROTTLE);

    // wait for the throttle stick to get in a low position to prevent accidents
    // stub this part when trying to calibrate the ESCs !!
    Serial.println("Please move throttle slightly above lowest position to continue ...");
    SetDualLedToggling(TURN_LED_BLINK_ON);
    while (InputThrottle < 1020 || InputThrottle > 1060)
    {
        read_receiver();
        InputThrottle = GetReceiverChannelValue(RC_CH3_THROTTLE);
        /* //RC debug, if no channels = 0 or -1 usually the cable is not properly in place or something similar
        Serial.print("Throttle Comms: ");
        Serial.println(InputThrottle);
        Serial.print("Number of channels: ");
        Serial.println(GetNrOfChannels());
        */
        HardwareControllerProcessing();
        delay(4);
    }
    SetDualLedToggling(TURN_LED_BLINK_OFF);
    Serial.println("Throttle is in a safe position, comms active, continuing with setup ...");
}
void MotorControllerInputs(void)
{
    // read_receiver(); //Moved to comms
    InputThrottle = GetReceiverChannelValue(RC_CH3_THROTTLE); // To add interfaces from flight controller for all 4 motors
    InputAux1SwState = GetAuxSwitchState();
    GetFcMotorCommands(&MotorInputs[0], &MotorInputs[1], &MotorInputs[2], &MotorInputs[3]);
}
void MotorControllerProcessing(void)
{
    MotorControllerInputs();
    // Serial.print("Aux1 Sw state [bn]: ");
    // Serial.println(InputAux1SwState);
    // Serial.print("Command sent to ESC [us]: ");
    if (InputAux1SwState != true && InputThrottle >= FC_THROTTLE_CUT_OFF_TRESHOLD)
    { // emergency shut down switch
        // Serial.println(MC_OUTPUT_PWM_RESOLUTION_OFFSET*InputThrottle);
        // analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_FL_PIN ,MC_OUTPUT_PWM_RESOLUTION_OFFSET*InputThrottle);
        // Serial.print(MC_OUTPUT_PWM_RESOLUTION_SCALING*map(InputThrottle,1000,2000,200,400));
        // Serial.print("Input Throttle: ");
        // Serial.println(InputThrottle);

        // Printing the values before writing to motors
        /*
        Serial.print("MC ");
        Serial.print("Motor_1:");
        Serial.print(MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[0], 1000, 2000, 200, 400));
        Serial.print(",");
        Serial.print(" Motor_2:");
        Serial.print(MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[1], 1000, 2000, 200, 400));
        Serial.print(",");
        Serial.print(" Motor_3:");
        Serial.print(MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[2], 1000, 2000, 200, 400));
        Serial.print(",");
        Serial.print(" Motor_4:");
        Serial.println(MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[3], 1000, 2000, 200, 400));
        */

        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_FR_PIN, MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[0], 1000, 2000, 200, 400));
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_RR_PIN, MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[1], 1000, 2000, 200, 400));
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_RL_PIN, MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[2], 1000, 2000, 200, 400));
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_FL_PIN, MC_OUTPUT_PWM_RESOLUTION_SCALING * map(MotorInputs[3], 1000, 2000, 200, 400));

        ToggleGreenLedON(); // has conflicts, only keep to test emergency shut down, to find a solution
    }
    else
    {


        // Printing the values before writing to motors

        /*
        Serial.print("MC-CUTOFF ");
        Serial.print("Motor_1:");
        Serial.print(map(MotorInputs[0], 1000, 2000, 200, 400));
        Serial.print(",");
        Serial.print(" Motor_2:");
        Serial.print(map(MotorInputs[1], 1000, 2000, 200, 400));
        Serial.print(",");
        Serial.print(" Motor_3:");
        Serial.print(map(MotorInputs[2], 1000, 2000, 200, 400));
        Serial.print(",");
        Serial.print(" Motor_4:");
        Serial.println(map(MotorInputs[3], 1000, 2000, 200, 400));
        */

        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_FR_PIN, map(MotorInputs[0], 1000, 2000, 200, 400)); // write motor inputs directly after testing, because cutoff is made in flight control
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_RR_PIN, map(MotorInputs[1], 1000, 2000, 200, 400));
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_RL_PIN, map(MotorInputs[2], 1000, 2000, 200, 400));
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_FL_PIN, map(MotorInputs[3], 1000, 2000, 200, 400));


        //Not necessary anymore, added additional emergency switch check in flight control cut-off
        //Fault redundancy, in case of memory curruptior or any possible faults non-software dependent regarding cut off
        //Apply cut-off directly to the ESCs
        /*
        if(MotorInputs[0]!=FC_THROTTLE_CUT_OFF||MotorInputs[1]!=FC_THROTTLE_CUT_OFF||MotorInputs[2]!=FC_THROTTLE_CUT_OFF||MotorInputs[3]!=FC_THROTTLE_CUT_OFF)
        {
            Serial.println("CUT_OFF_FAULT@@@@@@@@");
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_FR_PIN, MC_THROTTLE_CUT_OFF); // write motor inputs directly after testing, because cutoff is made in flight control
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_RR_PIN, MC_THROTTLE_CUT_OFF);
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_RL_PIN, MC_THROTTLE_CUT_OFF);
        analogWrite(MC_OUTPUT_PWM_ESC_MOTOR_FL_PIN, MC_THROTTLE_CUT_OFF);
        }
        */
        // without scaling, needs testing if THROTTLE MIN with scaling or without makes any difference
        // for safety assume MIN is 200 for now
        ToggleRedLedON();
    }
}