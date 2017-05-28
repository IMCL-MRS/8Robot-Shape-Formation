#include "vInfoList.h"
#include "neighbours.h"
#include "behaviors.h"
#include "AStar.h"

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


const typeCoordinate posP1  = {1.1503,0.3800};
const typeCoordinate posP2  = {1.1503,0.3800};
const typeCoordinate posP3  = {1.1503,0.3800};
const typeCoordinate posP4  = {1.1503,0.3800};

void snakeForm(){
    
}

void vSnakeTask( void *pvParameters ){
  while(1){
	snakeForm();
	halt(5000);
  }
}