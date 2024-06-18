#ifndef POWERMANAGEMENT_H
#define POWERMANAGEMENT_H

#include <arduino.h>

// Pin measures the voltage from a divider, so that we do not drive the pin more than 3.3V as per datasheet
#define PM_INPUT_BATTERY_VOLTAGE_DIVIDED_PIN_A 14

// Voltage divider resistor values in ohm, do not change unless you also change the resistors in the circuit
#define R1 2000
#define R2 330
#define VOLTAGE_DIV_OFFSET 0.0 // Offset of the voltage divider, 0.01 - 0.03 usually, to be determined when mounted to LiPo
#define VOLTAGE_DIV_RATIO ((double) R2 / ((double)(R1 + R2)))
#define VOLTS ((double) 3.3 / 1023.0)

// 3S LiPo Safe operating voltage range:

#define BatteryVoltageLiPoMax 12.6
     #define BatteryLowVoltageWarn 11.5 //11.7 or 5.9 for 5V testing @@@@@ 11.9 for LIVE TEST ONLY , 11.5 seems to be a good value to stop flying
     #define BatteryLowVoltageCritical 11.4 // 3.8 each cell, maybe implement later an interface for Hw control to make faster blinking for this case
#define BatteryVoltageLiPoMin 11.1 // 3.7 each cell, this is the voltage we should aim to not exceed at the end of flight, any lower and we might damage the battery

// example 5V input, 5V x VOLTAGE_DIV_RATIO = 0.7 V at divider => DividerVoltage=0.7V,
// then we want to transform back to 5V so we get the battery voltage, since teensy pins allow max 3.3v input
// then we have DividerVoltage/VOLTAGE_DIV_RATIO => 5V

extern float DividerVoltage;
extern float BatteryVoltage;
extern boolean BatteryLowVoltageFlag;

void PowerManagementInit(void);
void ReadDividerVoltage(void);
void CalculateBatteryVoltage(void);
void PrintVoltageProcessData(void);
void BatteryVoltageMonitor(void);
float SimulateBatteryVoltage(void);
boolean GetBatteryLowVoltageFlag(void);
void PowerManagementProcessing(void);

#endif // POWERMANAGEMENT_H