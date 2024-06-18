#include "PowerManagement.h"
#include "../HardwareController/HardwareController.h"

float DividerVoltage = 0.0;
float BatteryVoltage = 0.0;
boolean BatteryLowVoltageFlag = false;

//Simulation vars
int SimulationLoopCounter=0;

void PowerManagementInit(void)
{
    pinMode(PM_INPUT_BATTERY_VOLTAGE_DIVIDED_PIN_A,INPUT);
    DividerVoltage = 0.0;
    BatteryVoltage = 0.0;
    BatteryLowVoltageFlag = false;
}

void ReadDividerVoltage(void)
{
    //Serial.print("AnalogReadVolts: ");
    //Serial.println(analogRead(PM_INPUT_BATTERY_VOLTAGE_DIVIDED_PIN_A));
    DividerVoltage = (float)analogRead(PM_INPUT_BATTERY_VOLTAGE_DIVIDED_PIN_A) * VOLTS - VOLTAGE_DIV_OFFSET;
}

void CalculateBatteryVoltage(void)
{
    BatteryVoltage = DividerVoltage / VOLTAGE_DIV_RATIO;
    //BatteryVoltage=SimulateBatteryVoltage();
    //Serial.print("Battery voltage: ");
    //Serial.println(BatteryVoltage);
}


void PrintVoltageProcessData(void){
    Serial.print("Voltage div ratio: ");
    Serial.print(VOLTAGE_DIV_RATIO);
    Serial.print(" To volts constant: ");
    Serial.print(VOLTS,6);
    Serial.print(" VoltageDivider [V]: ");
    Serial.print(DividerVoltage);
    Serial.print(" BatteryVoltage [V]: ");
    Serial.println(BatteryVoltage,6);
}

void BatteryVoltageMonitor(void){
    ReadDividerVoltage();
    CalculateBatteryVoltage();
    //Serial.println("voltage monitor");
    if(BatteryVoltage <= BatteryLowVoltageWarn){
        //Serial.println("voltage monitor: low voltage condition");
        BatteryLowVoltageFlag=true;
        SetRedLedToggling(TURN_LED_BLINK_ON);
    } else {
        SetRedLedToggling(TURN_LED_BLINK_OFF);
        BatteryLowVoltageFlag=false;
    }

}

float SimulateBatteryVoltage(void){
    
    if(SimulationLoopCounter<=50*HC_SECOND){
    SimulationLoopCounter++;
    }else
    SimulationLoopCounter=0;
                                                                //different values for real time, set to 100/2 x second and 50x
    if (SimulationLoopCounter % (50*HC_SECOND) < 25*HC_SECOND){ //200-100 * 50 ms loop time we get 5 s blink 5 s off. (voltage above warning level and below ), 500 250 for 20 ms
        return 11.5;
    } else {
        return 12.0;
    }
}

boolean GetBatteryLowVoltageFlag(void){
    return BatteryLowVoltageFlag;
}

void PowerManagementProcessing(void){
    BatteryVoltageMonitor(); //Needs pin connected to divider
}