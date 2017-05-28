#include "apps.h"
#include "neighbours.h"
#include "behaviors.h"
#include "vDemoCtl.h"

extern rbNode bCastInfo[ROBOTS];
extern volatile u8 nbrList[ROBOTS];
volatile u8 rbType = 0;
extern volatile u8 isActive;

//#define xSemaphoreCreateMutex() xQueueCreateMutex( queueQUEUE_TYPE_MUTEX )
static xSemaphoreHandle neighborsMutex;
static uint32 neighborPeriod = NEIGHBOR_PERIOD_DEFAULT;

xTaskHandle xHandle_Dist = NULL;

//static uint32 obstacle_timeout = OBSTACLE_TIMEOUT_ROUNDS;
//boolean rbIsPoweroff(uint8 ID){
//  if()
//	return TRUE;
//  else
//	return FALSE;
//}

volatile u8 activeRb[ROBOTS] = {0};
volatile uint32 rbCount1 = 0;
volatile uint32 rbCount2 = 1;

void updateActiveRobotList2(u8 leader){
  u8 i = 0;
  for(i = 1; i < ROBOTS; i++){
	if((bCastInfo[i].isActive == 1) && (bCastInfo[i].nodeID != 0)){
	  if(bCastInfo[i].nodeID == leader){
		activeRb[0] = leader;
	  }else{
	  	activeRb[rbCount1++] = bCastInfo[i].nodeID;
	  }
	}
  }
  asm("NOP");
}

void updateActiveRobotList1(){
  u8 i = 0;
  u8 k = 0;
  for(i = 0; i < ROBOTS; i++){
	if((bCastInfo[i].isActive == 1) && (bCastInfo[i].nodeID != 0)){
	  	activeRb[k++] = bCastInfo[i].nodeID;
	}
  }
  rbCount1 = k;
}

int getActiveRbNum(){
  return rbCount1;
}

void nbrRemoveNbr(uint8 ID) {
	bCastInfo[ID - 1].nodeID = ROBOT_ID_NULL;
}

void nbrDataSet(u8 ID, u8 type){
  bCastInfo[ID - 1].type = type;
  rbType = type;
}

/*
 * @brief Get neighbors mutex with specified max delay.
 *
 * @returns void
 */
signed neighborsGetMutexDelay(unsigned long delay) {
	return xSemaphoreTake(neighborsMutex, delay);
}

/*
 * @brief Get neighbors mutex.
 *
 * @returns void
 */
void neighborsGetMutex(void) {
	xSemaphoreTake(neighborsMutex, portMAX_DELAY);
}

/*
 * @brief Put neighbors mutex.
 *
 * @returns void
 */
void neighborsPutMutex(void) {
	xSemaphoreGive(neighborsMutex);
}

/*
 * @brief Set neighbor period, neighbor timeout, and obstacle timeout proportional to argument.
 *
 * @param neighbor_period_arg the neighbor period length in rounds
 * @returns void
 */
static void neighborsSetPeriod(uint32 neighbor_period_arg) {
	neighborPeriod = neighbor_period_arg;
//	obstacle_timeout = OBSTACLE_TIMEOUT_ROUNDS;
}

void clearNodeMsg(){
  u8 i = 0;
  for(i = 0; i < ROBOTS; i++){
	bCastInfo[i].nodeID = 0;
	bCastInfo[i].angle2n = 0;
	bCastInfo[i].isActive = 0;
	bCastInfo[i].isReady = 0;
	bCastInfo[i].type = 0;
	bCastInfo[i].rpos.locationX = 0;
	bCastInfo[i].rpos.locationY = 0;
  }
}

bool followerInit(u8 initor){
  u8 i = 0;
  u8 k = 0;
  
  for(i = 0; i < GROBOTS; i++){
      if(bCastInfo[initor - 1].nbrList[i]){
        activeRb[k++] = bCastInfo[initor - 1].nbrList[i];
      }
  }
  //memcpy((u8 *)(&nbrList),(u8 *)(&bCastInfo[initor - 1].nbrList),sizeof(nbrList));
  
  rbCount1 = k;
#ifdef VISIT_DEMO
  if(rbCount1 < GROBOTS){
	return 0;
  }
#endif
  return rbCount1;
}

bool robotsReady(){
  bool allReady = groupReady();
  while(!allReady){
	if((rbID - 1) == 0){
	  if(bCastInfo[0].nodeID == 1){
		bCastInfo[0].isActive = TRUE;
		isActive = 1;
	  }else{
		bCastInfo[0].isActive = FALSE;
		isActive = 0;
	  }
	}else if(bCastInfo[rbID - 1].nodeID == rbID){
	  bCastInfo[rbID - 1].isActive = TRUE;
	  isActive = 1;
	}else{
	  bCastInfo[rbID - 1].isActive = FALSE;
	  isActive = 0;
	}  
	allReady = groupReady();
	halt(1);
  }
  return TRUE;
}

//u8 neighborsInit1(){
//  static u8 leader = 0;
//  uint8 navigationMode = MODE_IDLE;
//  
//  bool rbsReady = robotsReady();
//  while(!rbsReady){
//	halt(1);
//	rbsReady = robotsReady();
//  }
//  
//  leader = electLeader();
//  if((leader <= 0) || (leader > ROBOTS))
//	return 0;
//  
//  //updateActiveRobotList1();
//  
//  if(leader != 0){
//	if(leader == rbID){
//	  navigationMode = LEADER;
//	}else{
//	  navigationMode = FOLLOWER;
//	}
//	
//	switch (navigationMode) {
//	case MODE_IDLE: {
//	  //		ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
//	  nbrDataSet(rbID, 0);
//	  break;
//	}
//	case LEADER: {
//	  //		ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
//	  nbrDataSet(rbID, 1);
//	  break;
//	}
//	case FOLLOWER: {
//	  //		ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
//	  nbrDataSet(rbID, 2);
//	  break;
//	}
//	case OBJECT: {
//	  //		ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
//	  nbrDataSet(rbID, 3);
//	  break;
//	}
//	case OBJECT_FOUND: {
//	  //		ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
//	  nbrDataSet(rbID, 4);
//	  break;
//	}
//	case GRIPPED: {
//	  //		ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
//	  nbrDataSet(rbID, 5);
//	  break;
//	}
//	default:{
//	}
//	
//	}//end switch
//  } //end if
//  return leader;
//}

#if 0
void neighborsTask(void* parameters) {
  uint8 i;
  static u8 leader = 0;
  uint8 navigationMode = MODE_IDLE;
  portTickType lastWakeTime, currentTime;
  lastWakeTime = xTaskGetTickCount();
  while(1){
//	neighborsGetMutex();
	currentTime = xTaskGetTickCount();
//    vTaskDelay(NBR_LEADER);
	for(i = 0; i < ROBOTS; i++){
	  if(i == 0){
		if(bCastInfo[i].nodeID == 1){
			bCastInfo[i].isActive = TRUE;
		    isActive = 1;
		}
		else{
		    bCastInfo[i].isActive = FALSE;
			removeNode(0);
		}
	  }else if(bCastInfo[i].nodeID == i+1){
		bCastInfo[i].isActive = TRUE;
		isActive = 1;
	  }else{
		bCastInfo[i].isActive = FALSE;
		removeNode(bCastInfo[i].nodeID);
	  }
	}
	
	//formerLeader = leader;
	leader = electLeader();
	updateActiveRobotList1();
	if(leader != 0){
	  if(leader == rbID){
		navigationMode = LEADER;
	  }else{
		navigationMode = FOLLOWER;
	  }
	  
	  switch (navigationMode) {
	  case MODE_IDLE: {
		//		ledsSetPattern(LED_ALL, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_SLOW);
		nbrDataSet(rbID, 0);
		break;
	  }
	  case LEADER: {
		//		ledsSetPattern(LED_RED, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		nbrDataSet(rbID, 1);
		break;
	  }
	  case FOLLOWER: {
		//		ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		nbrDataSet(rbID, 2);
		break;
	  }
	  case OBJECT: {
		//		ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		nbrDataSet(rbID, 3);
		break;
	  }
	  case OBJECT_FOUND: {
		//		ledsSetPattern(LED_BLUE, LED_PATTERN_PULSE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		nbrDataSet(rbID, 4);
		break;
	  }
	  case GRIPPED: {
		//		ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		nbrDataSet(rbID, 5);
		break;
	  }
	  default:{
	  }
	  
	  }//end switch
	} //end if
	asm("NOP");
	
//	neighborsPutMutex();
	currentTime = xTaskGetTickCount();
	if (currentTime < (lastWakeTime + neighborPeriod)) {
		vTaskDelayUntil(&lastWakeTime, neighborPeriod);
	}
  } /*end while*/
}
#endif

//void neighborsDist( void* parameters){
//  portTickType lastWakeTime, currentTime;
//  lastWakeTime = xTaskGetTickCount();
//  while(1){
////	vTaskDelay(NBRDIST_PEROID);
//	if(!bCastInfo[electLeader()-1].isStop){
//	  u8 i = 0;
//	  float x1,y1,x2, y2, dist;
//	  if( (getActiveRbNum() >= 2)){
//		for(i = 0; i < getActiveRbNum() - 1; i++){
//		  x1 = bCastInfo[i].rpos.locationX;
//		  y1 = bCastInfo[i].rpos.locationY;
//		  x2 = bCastInfo[i + 1].rpos.locationX;
//		  y2 = bCastInfo[i + 1].rpos.locationY;
//		  dist = getDistance2(x1,y1,x2,y2);
//		  
//		  while(dist >= 0.4){
//			halt(2);
//			x1 = bCastInfo[i].rpos.locationX;
//			y1 = bCastInfo[i].rpos.locationY;
//			x2 = bCastInfo[i + 1].rpos.locationX;
//			y2 = bCastInfo[i + 1].rpos.locationY;
//			dist = getDistance2(x1,y1,x2,y2);
//		  }
//		}
//	  }
//	}
//	currentTime = xTaskGetTickCount();
//	if (currentTime < (lastWakeTime + neighborPeriod)) {
//	  vTaskDelayUntil(&lastWakeTime, neighborPeriod);
//	}
//  }
//}

/*
 * @brief Initialize neighbors and start neighbors task.
 *
 * Initializes neighbor period, neighbor timeout, obstacle timeout. Initialize neighborData
 * Sets message length.
 * Semaphore implementing neighborsMutex created.
 * @param neighbor_period_arg the neighbor period in rounds
 * @returns void
 */
void neighborsInit(uint32 neighbor_period_arg) {
  neighborsSetPeriod(neighbor_period_arg);
  neighborsMutex = xSemaphoreCreateMutex();
  xTaskCreate(neighborsTask, "neighbors",    configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY + 1,NULL );
//  xTaskCreate(neighborsDist, "keepDistance", configMINIMAL_STACK_SIZE*4, NULL, KEEP_DISTANCE_TASK_PRIORITY,&xHandle_Dist );
}
