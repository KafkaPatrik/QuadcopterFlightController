#include "RadioComms.h"

float ReceiverChannelValue[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;
PulsePositionInput ReceiverInput(RISING);
boolean Aux1SwState=false;

void RadioCommsInit(void)
{
    ReceiverInput.begin(RC_PPM_INPUT_PIN);
}

void read_receiver(void)
{
    ChannelNumber = ReceiverInput.available();
    if (ChannelNumber > 0)
    {
        for (int ChannelIndex = 1; ChannelIndex <= ChannelNumber; ChannelIndex++)
        {
            ReceiverChannelValue[ChannelIndex - 1] = ReceiverInput.read(ChannelIndex);
        }
    }
}

float GetReceiverChannelValue(int ChannelIndex)
{
    return ReceiverChannelValue[ChannelIndex];
}

int GetNrOfChannels()
{
    return ChannelNumber;
}

void PrintRCInputs(void)
{
    read_receiver();
    Serial.print("Number of channels: ");
    Serial.print(GetNrOfChannels());
    Serial.print(" Roll [μs]: ");
    Serial.print(GetReceiverChannelValue(0));
    Serial.print(" Pitch [μs]: ");
    Serial.print(GetReceiverChannelValue(1));
    Serial.print(" Throttle [μs]: ");
    Serial.print(GetReceiverChannelValue(2));
    Serial.print(" Yaw [μs]: ");
    Serial.println(GetReceiverChannelValue(3));
    Serial.print(" Aux 1: ");
    Serial.print(GetReceiverChannelValue(4));
    Serial.print(" Aux 2: ");
    Serial.print(GetReceiverChannelValue(5));
    Serial.print(" Aux 3: ");
    Serial.print(GetReceiverChannelValue(6));
    Serial.print(" Aux 4: ");
    Serial.print(GetReceiverChannelValue(7)); //PPM supports only 8 channels max
    Serial.print(" Aux 5: ");
    Serial.print(GetReceiverChannelValue(8));
    Serial.print(" Aux 6: ");
    Serial.println(GetReceiverChannelValue(9));
    delay(50);
}

void SwitchStateProcessing(void){
    Aux1SwState=GetReceiverChannelValue(RC_CH7_AUX1)<=RC_SWITCH_TRESH?false:true;
}

boolean GetAuxSwitchState(void){
    return Aux1SwState; //After deciding which switches to add, make the function callable with a parameter and return from a vector the sw state value
}
void RadioCommsProcessing(void){
    read_receiver();
    SwitchStateProcessing();
}