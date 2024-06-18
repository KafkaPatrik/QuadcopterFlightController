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
  MotorControllerInit();//at this state maybe add alternating leds to signal this
  IMU_Init();// maybe move it before blinking or make a separate blinking sequence for calibration
  ToggleGreenLedON();
  ControlLoopTimer=micros();
}

void loop()
{
  RadioCommsProcessing();
  IMUProcessing();
  //PrintAccData();
  //PrintAccOrientationAngles();
  FlightControllerProcessing();
  MotorControllerProcessing();
  PowerManagementProcessing();
  HardwareControllerProcessing();
 
  while(micros()- ControlLoopTimer <4000); //250 HZ loop, 4us loop time, needed for PID dT
  ControlLoopTimer=micros();


//Working MotorController, VoltageMonitor, HardwareController
  //MotorControllerProcessing();
  //BatteryVoltageMonitor();
  //HardwareControllerProcessing();
  //delay(20); //50HZ loop
//delay(5);
// PowerManagement - 
  //BatteryVoltageMonitor();
  //PrintVoltageProcessData();
  //HardwareControllerProcessing();
  //delay(50); // blinking dependent on loop time, update it when final loop time is determined
// IMU
  //PrintGyroData();
  //PrintAccData();
  //PrintAccOrientationAngles();
  //delay(50);
// RC
  //read_receiver();
  //PrintRCInputs();
  //delay(50);
}
