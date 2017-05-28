#include "FreeRTOS.h"
#include "task.h"
#include "robot.h"
#include "behaviors.h"
#include "AStar.h"
#include "vDemoCtl.h"

#define G      1
#define SAMPLE 10
#define sValue 0.1

extern volatile u8 stage;

void snakeStart(float x, float y){
    rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
    gotoWayPoint(x,y,LINESPEED,PHASE_ONE);  //leader should run by itself
}

void goFastStraight(float x, float y){
   rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
   gotoWayPoint(x,y,FASTSPEED,PHASE_ONE);  //leader should run by itself
}

void snakeRun(float x, float y, int ptNum){  
  stage = ptNum;
  vTaskDelay(500);
  runDrawCircle(x,y,FASTSPEED,PHASE_ONE,ptNum);  //leader should run by itself
}

float readAaccelZ(){
    typeMPUSensor accelValue = halReadMPUSensor();
	float zValue = accelValue.accelZ;
	return zValue;
}

int isPickedUp(){
  static typeMPUSensor accelValue;  
  static float zValue = 0;
  static float value = 0;
  float sum = 0;
  u8 i = 0;
  for(i = 0;i < 10; i++){
	accelValue = halReadMPUSensor();
	zValue = -accelValue.accelZ;
	sum += (zValue - G)*(zValue - G);	   
  } 
  value = sqrt(sum/10);
  if(value >= sValue){
	return 1;
  }
  return 0;
}

void drawCircle(){
    SetLeftWheelGivenSpeed(20);
	SetRightWheelGivenSpeed(34);
	vTaskDelay(5000);
}
