#include "vBCastInfoTask.h"
#include "apps.h"
#include "vInfoList.h"

#define SAMPLE   10
#define G        1

extern int isPickup;
float array[10];

void vAccelatorTask( void *pvParameters ){
  static typeMPUSensor accelValue;
  int i = 0;
  static float sValue = 0.1;
  
  while(1){	
//	SetLeftWheelGivenSpeed(30);
//	SetRightWheelGivenSpeed(30);
	isPickup = 0;
    vTaskDelay(MPU_SAMPLE_PERIOD);
	static float zValue = 0;
    static float value = 0;
    float sum = 0;
	for(i = 0;i < 10; i++){
	   accelValue = halReadMPUSensor();
	   zValue = -accelValue.accelZ;
	   sum += (zValue - G)*(zValue - G);	   
	} 
	value = sqrt(sum/10.0);
	if(value >= sValue){
	  isPickup = 1;
	}	
	asm("NOP");
  } 
}

//  accelValue.accelZ = 0;
//	for(i = 0;i < 10; i++){
//	  accelValue = halReadMPUSensor();
//	  zValue = -accelValue.accelZ;
//	  sum += (zValue - 1)*(zValue - 1);	   
//	} 
//	zValue = sqrt(sum/10.0);
//	if(zValue >= TMEAN){
//	  float endX = bCastInfo[activeRb[rbCount1 -1] - 1].rpos.locationX;
//	  float endY = bCastInfo[activeRb[rbCount1 -1] - 1].rpos.locationY;
//	  rotateFastTo(endX,endY,FASTSPEED,ROTATE_ACCURATE);  
//	  x = endX;
//	  y = endY;
//	}
