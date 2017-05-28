#include "vMotorControlTask.h"

void vMotorControlTask( void *pvParameters ) {
  MotorRunEnable(true);
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while(1){
	/* We want this task to execute exactly every 10 milliseconds. */
	vTaskDelayUntil( &xLastWakeTime, ( 10 / portTICK_RATE_MS ) );
    ControlWheelSpeedAlg(GetWheelGivenSpeed() , ReadWheelSpeed());
  }
}

