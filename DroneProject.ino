#define ENABLE_ARDUINO_IDE_SUPPORT

#ifdef ENABLE_ARDUINO_IDE_SUPPORT
#include "src/CustomLibs.h"
#else
#include "RadioComms.h"
#include "IMUController.h"
#include "HardwareController.h"
#include "PowerManagement.h"
#include "MotorController.h"
#include "FlightController.h"
#endif

void setup()
{
  HardwareController_Init();
  ToggleRedLedON();
  Serial.begin(57600);
  delay(5000);//serial initialize time https://forum.pjrc.com/index.php?threads/teensy-4-0-serial-monitor-issues.58726/
  PowerManagementInit();
  RadioCommsInit();
  MotorControllerInit();
  IMU_Init();
  ToggleGreenLedON();
  ControlLoopTimer=micros();
}

void loop()
{
  RadioCommsProcessing();
  IMUProcessing();
  FlightControllerProcessing();
  MotorControllerProcessing();
  PowerManagementProcessing();
  HardwareControllerProcessing();
 
  while(micros()- ControlLoopTimer <4000); //250 HZ loop, 4us loop time, needed for PID dT
  ControlLoopTimer=micros();
}
