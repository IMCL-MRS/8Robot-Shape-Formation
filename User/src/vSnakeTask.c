
#include "vInfoList.h"
#include "neighbours.h"
#include "behaviors.h"
#include "AStar.h"
#include "fc.h"
#include "snake.h"
#include "robot.h"

#define NEIGHBOR_ROUND_PERIOD		400
#define BEHAVIOR_TASK_PERIOD		50

extern rbNode bCastInfo[ROBOTS];	
extern u8 activeRb[ROBOTS];
extern wayPoint wayArr[STAR];
extern volatile u8 rbType;
extern wayPoint endWayPoint;
extern wayPoint obsStart;
extern wayPoint obsEnd;
extern int turnPoint;
extern uint32 rbCount1;
extern const typeCoordinate posEnd;
extern wayPoint endWayPoint;

extern volatile u8 isActive;
volatile u8 imReady = 0;
volatile u8 isLeaderOK = 0;
volatile u8 imFoundPath = 0;
volatile u8 isStop = 0;
volatile u8 stage = 0;
extern u8 isPickup;

//const typeCoordinate posP1  = {0.3500,0.5000};
//const typeCoordinate posP2  = {1.0000,0.5000};
//const typeCoordinate posP3  = {1.0000,0.0755};
//const typeCoordinate posP4  = {0.3800,0.0755};

const typeCoordinate posP1  = {1.4000,0.7000};
const typeCoordinate posP2  = {0.4000,0.7000};
const typeCoordinate posP3  = {0.4000,0.0755};
const typeCoordinate posP4  = {1.4000,0.0755};

void snakeForm(float x, float y){
  u8 i = 0;
  static u8 leader = 0;
  static u8 initiator = 1;
  
//  initiator = electInitiator();
//  while(!initiator){
//	initiator = electInitiator();
//	halt(0.5);
//  }
  
  isActive = 1;
  halt(0.5);  
  while(!groupReady()){
    halt(0.5);
  }  
  
  initiator = 1; 
  halt(1);
  if(rbID == initiator){
	halt(1);
	leader = electLeader(initiator,x,y);
	while(!leader){
	  leader =  electLeader(initiator,x,y);
	  halt(0.5);
	}
	isLeaderOK = 1;
	halt(0.5);        
	if(rbCount1 >= 2){
//          u8 i = 0;
//          u8 count = 0;
	  while(1){
//           count = 0;
//           for(i = 2; i <= GROBOTS; i++){
//             if(isReady(i)){
//                count++;
//             }
//           }
//           if(count == (GROBOTS - 1)){
//                imReady = 1;	  
//                vTaskDelay(100);
//                break;
//           }
            
            if (isReady(2) && isReady(3) && isReady(4) && isReady(5) && isReady(6) && isReady(7) && isReady(8)) {
                imReady = 1;	  
                vTaskDelay(100);
                break;
            }
           vTaskDelay(500);
	  }
	}else{
	  imReady = 1;
	  halt(0.5);
	}
  }else{  //not initiator
	halt(1);
	while(bCastInfo[initiator - 1].isLeaderOK != 1){	  
	  halt(0.5);
	}
//	SetLedStatus(LED_GREEN, LED_ON);
	
	while(!followerInit(initiator)){
	  halt(0.5);
	}
//	halSetLedStatus(LED_RED, LED_ON);
	imReady = 1;
//	halt(1);
	vTaskDelay(1000);
	
	while(1){
	  if(isReady(initiator)){
		imReady = 1;
//		halt(0.5);
		break;
	  }
	  vTaskDelay(500);
//	  halt(0.5);
	}
  }
  
  halSetLedStatus(LED_YELLOW, LED_ON);
  //all robots will go here.
  imReady = 1;
  halt(0.5);
  if(imReady){	
      for(i = 1; i < rbCount1; i++){
        if(activeRb[i] == rbID){
            halt(i*4 + 3);
        }
      }
  }  
}

void snakeBeginForm(){
  if((activeRb[0]) == rbID){
	snakeStart(posP2.x + 0.12,posP2.y);
	//drawCircle();
	stage = 3;
  }else if((activeRb[1]) == rbID){
	snakeStart(posP2.x + 0.25,posP2.y);
	halt(0.5);
	stage = 3;
  }else if((activeRb[2]) == rbID){
	snakeStart(posP2.x + 0.38,posP2.y);
	halt(0.5);
	stage = 3;
  }else{
	snakeStart(posP2.x + 0.51,posP2.y);
	halt(0.5);
	stage = 3;
  }
}

void runCircle(){
  if((stage == 1)){
    snakeRun(posP1.x,posP1.y,1);
    if(!isPickup){
      stage = 2;
      vTaskDelay(500);
    }
  }else if(stage == 2){
    snakeRun(posP2.x,posP2.y,2);
    if(!isPickup){
      stage = 3;
      vTaskDelay(500);
    }	  
  }else if(stage == 3){
    snakeRun(posP3.x,posP3.y,3);
    if(!isPickup){
      stage = 4;
      vTaskDelay(500);
    }	  
  }else if(stage == 4){
    snakeRun(posP4.x,posP4.y,4);
    if(!isPickup){
      stage = 1;
      vTaskDelay(500);
    }
  }
}

void accelatorTest(){ 
  static typeMPUSensor accelValue;
  static float zValue = 0;
  while(1){
    accelValue = halReadMPUSensor();
	zValue = -accelValue.accelZ;
	halt(zValue);
  }
}

void pickUpCheck(){
   typeCoordinate posOne = GetCoordinate();
   SetLeftWheelGivenSpeed(30);
   SetRightWheelGivenSpeed(30);
   vTaskDelay(400);
   typeCoordinate posTwo = GetCoordinate();
   if(getDistance2(posOne.x, posOne.y, posTwo.x, posTwo.y) >= 0.15){
	  float endX = posP1.x;
	  float endY = posP2.y;
	  rotateFastTo(endX,endY,FASTSPEED,ROTATE_ACCURATE); 
	  gotoWayPoint(endX,endY,FASTSPEED,PHASE_ONE);
	}
}

void circle(){
    snakeRun(posP1.x,posP1.y,1);
	snakeRun(posP2.x,posP2.y,2);	
	snakeRun(posP3.x,posP3.y,3);
	snakeRun(posP4.x,posP4.y,4);
}

void testPoint(){
   rotateFastTo(posP1.x,posP1.y,ANGLESPEED,ROTATE_ACCURATE);
   gotoWayPoint(posP1.x,posP1.y,FASTSPEED,PHASE_ONE);  //leader should run by itself
}

void robot1Snake(){
  while (1) {
    if (statusReady(0)) {
      imReady = 0;
      vTaskDelay(100);
      break;
    }
    vTaskDelay(500);
  }
}

void robotSnake(){
  while (1) {
    imReady = 0;
    if (isReady0(activeRb[0])) {
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
}

void sychronize(){
  if(rbID == activeRb[0]){
	robot1Snake();
  }else{
	robotSnake();
  }
  halt(1);
  beepSing();
}

void recoverShape(){
  shapeForm();
}

void hdTest(){
   const typeCoordinate posS1 = {1.5,0.28};
   snakeStart(posS1.x, posS1.y);
   halt(1000);
//  while(1){
//  ControlRobotRotate(180,15);
//  halt(1);
//  ControlRobotRotate(-180,15);
//  halt(1);
//  }
}

void vSnakeTask( void *pvParameters ){ 
  int i = 0;
/*****************Formation Control************/      
  sychronize();
/******************Snake Eating ****************/
  while(1){
	sychronize();  
	snakeBeginForm();
	for(i = 0;i < 4; i++){
	  	runCircle(); 
	}
	  shapeForm();
  }
}
