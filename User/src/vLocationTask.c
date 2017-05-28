#include "vLocationTask.h"
#include "robot.h"
#include "vInfoList.h"
#include <math.h>

typeCoordinate location;
float angle2North = 0;
float R2X = 0;
void vLocationTask( void *pvParameters ) {
  halt(1);
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  while(1) {
	location = GetCoordinate();
        //front direction parallel to X axis
	angle2North = ReadMagSensorAngle2North(); 
	R2X = CalibrateNorth2X();
	vTaskDelay(500);
  }
}

