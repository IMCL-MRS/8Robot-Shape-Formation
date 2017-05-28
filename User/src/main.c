#include "stm32f10x.h"
#include "halDriver.h"
#include "vTasks.h"
#include "apps.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEMO //LOCATION //MAG //EXAMPLE
int main(void) {
  //////////////////////////////////////////////////////////////////////////////
  //1. hardware init
  //////////////////////////////////////////////////////////////////////////////
  SystemInit();    //STM32 cpu clock init -> 72MHZ
  halRCCInit();    //STM32 PeriphClock init
  //////////////////////////////////////////////////////////////////////////////
  //2. Periph devices init
  /////////////////////    //a. Led init
  WheelSpeedSensorInit();  //b. motor feedback speed sensor init 
  MotorInit();         /////////////////////////////////////////////////////////
  LedInit();               //c. motor driver init
  UartInit();              //d. uart Init, communicate with CC2530. ***it must be initialized, even uart is not used!!!***
  halMCUWaitMs(100);
  MPUSensorInit();         //e. MPU sensor Init -> gyro accel mag sensor
  RFInit();                //f. RF Init. via UART communicate with CC2530, config RF
  LightSensorInit();       //g. Light Sensor Init
  BeepInit();  
  //建立任务 
  //Motor task
  xTaskCreate( vMotorControlTask,    ( signed portCHAR * ) "MOTOR",  configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+10, NULL );  

#if defined(DEMO) 
  // Demo
  xTaskCreate( vBCastInfoTask,      ( signed portCHAR * ) "BCast",    configMINIMAL_STACK_SIZE*16, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( vDemoOneTask,        ( signed portCHAR * ) "SNAKE",    configMINIMAL_STACK_SIZE*5,  NULL, tskIDLE_PRIORITY+1, NULL );        
#elif defined(MAG)
  // Debug mag
  xTaskCreate( vMagTask,            ( signed portCHAR * ) "MAG",      configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL );
  xTaskCreate( vBCastMagTask,       ( signed portCHAR * ) "BCastMag", configMINIMAL_STACK_SIZE*16, NULL, tskIDLE_PRIORITY, NULL );  
#elif defined(LOCATION)
  // Debug location
  xTaskCreate( vLocalizationTask,   ( signed portCHAR * ) "MPU READ", configMINIMAL_STACK_SIZE*5, NULL, tskIDLE_PRIORITY+4, NULL );  
  xTaskCreate( vLocationTask,       ( signed portCHAR * ) "LOCATION", configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY+3, NULL );
  // Example
#elif defined(EXAMPLE)
  xTaskCreate( vExampleTask,        ( signed portCHAR * ) "Example",  configMINIMAL_STACK_SIZE*16, NULL, tskIDLE_PRIORITY+5, NULL );  
  xTaskCreate( vStackCheck,         ( signed portCHAR * ) "STACK",    configMINIMAL_STACK_SIZE,  NULL, tskIDLE_PRIORITY,   NULL );      
#endif
  
  //启动OS
  vTaskStartScheduler();  
  return 0;
}

