#include "FreeRTOS.h"
#include "task.h"
#include "robot.h"
#include "behaviors.h"
#include "AStar.h"
#include "snake.h"
#include "vDemoCtl.h"


extern const typeCoordinate posEnd;

extern rbNode bCastInfo[ROBOTS];
extern u8 activeRb[ROBOTS];
extern uint32 rbCount1;
volatile u8 isActive = 0;
volatile u8 nbrList[ROBOTS] = {0};

u8 isPickup = 0;
extern u8 stage;

rbDist rbList[ROBOTS] = {0};
//static xSemaphoreHandle neighborsMutex;

/******** Defines ********/
#define __PI 3.14159
#define TMEAN   0.8   //accelator threshold

/* Angles */
#define MILLIRAD_HALF_PI			1571
#define MILLIRAD_PI					(MILLIRAD_HALF_PI * 2)
#define MILLIRAD_2PI				(MILLIRAD_HALF_PI * 4)

#define MILLIRAD_DEG_0				0
#define MILLIRAD_DEG_15				(MILLIRAD_HALF_PI / 6)
#define MILLIRAD_DEG_20				(MILLIRAD_PI / 9)
#define MILLIRAD_DEG_30				(MILLIRAD_HALF_PI / 3)
#define MILLIRAD_DEG_45				(MILLIRAD_HALF_PI / 2)
#define MILLIRAD_DEG_60				(MILLIRAD_PI / 3)
#define MILLIRAD_DEG_90				MILLIRAD_HALF_PI
#define MILLIRAD_DEG_180			MILLIRAD_PI
#define MILLIRAD_DEG_270			(MILLIRAD_PI + MILLIRAD_HALF_PI)
#define MILLIRAD_DEG_360			MILLIRAD_2PI


int16 atan2MilliRad(int32 y, int32 x) {
  boolean x_fold = FALSE;
  boolean y_fold = FALSE;
  boolean deg_45_fold = FALSE;
  uint32 val, denom, ux, uy;
  
  /* argument folding */
  if (x < 0) {
	x_fold = TRUE;
	x = -x;
  }
  if (y < 0) {
	y_fold = TRUE;
	y = -y;
  }
  if (y > x) {
	int32 tmp;
	deg_45_fold = TRUE;
	tmp = x;
	x = y;
	y = tmp;
  }
  
  uy = (uint32)y;
  ux = (uint32)x;
  
  /* atan2 approximation for 0 <= y/x <= 1
  basic formula is
  (60937 * y * x) /
  ((61 * x * x) + (17 * y * y))
  we check the size of ux to know how much to truncate args so we don't overflow
  ux and uy must be 8-bit numbers to prevent overflow
  x > y so we only need to check x
  */
  while (ux & ~0x000000FF) {
	ux = ux >> 1;
	uy = uy >> 1;
  }
  val = (60937L * uy * ux);
  denom = (61L * ux * ux) + (17L * uy * uy);
  
  if (denom != 0) {
	val = val / denom;
	/* argument unfolding */
	if (deg_45_fold) {
	  val = MILLIRAD_DEG_90 - val;
	}
	if (x_fold) {
	  val = MILLIRAD_PI - val;
	}
	if (y_fold && (val > 0)) {
	  val = MILLIRAD_2PI - val;
	}
  }
  else {
	// denom = 0 iff x = y = 0, but then function is undefined.  Return 0.
	val = 0;
  }
  
  return((int16)val);
}

typeCoordinate getNowPos(float tarX, float tarY){
  static typeCoordinate nowPos;
  nowPos.x = bCastInfo[rbID - 1].rpos.locationX;
  nowPos.y = bCastInfo[rbID - 1].rpos.locationY;  

  u8 x = isnan(nowPos.x);
  u8 y = isnan(nowPos.y);
  while(x || y){
	halSetLedStatus(LED_BLUE, LED_ON);
	beepSing();
	vTaskDelay(5);	
	ControlRobotRotate(30, 10);
	rotateFastTo(tarX,tarY,10,ROTATE_ACCURATE);
	nowPos = GetCoordinate();
//	nowPos.x = bCastInfo[rbID - 1].rpos.locationX;
//	nowPos.y = bCastInfo[rbID - 1].rpos.locationY;  
	x = isnan(nowPos.x);
	y = isnan(nowPos.y);	
  }
  halSetLedStatus(LED_BLUE, LED_OFF);  
  return nowPos;
}


void bubbleSort(rbDist list[], int n){
  int c, d;
  rbDist t;
  for (c = 0 ; c < ( n - 1 ); c++){
    for (d = 0 ; d < n - c - 1; d++){
      if (list[d].dist > list[d+1].dist){
        /* Swapping */
        t         = list[d];
        list[d]   = list[d+1];
        list[d+1] = t;
      }
    }
  }
}

uint8 removeNode(u8 ID){
  bCastInfo[ID - 1].nodeID = 0;
  bCastInfo[ID - 1].angle2n = 0;
  bCastInfo[ID - 1].isActive = 0;
  bCastInfo[ID - 1].type = 0;
  bCastInfo[ID - 1].rpos.locationX = 0;
  bCastInfo[ID - 1].rpos.locationY = 0;
  return 0;
}

uint8 clearNbrMsg(){
  uint8 i = 0;
  for(i = 0; i < ROBOTS; i++){
	bCastInfo[i].nodeID = 0;
	bCastInfo[i].angle2n = 0;
	bCastInfo[i].isActive = 0;
	bCastInfo[i].type = 0;
	bCastInfo[i].rpos.locationX = 0;
	bCastInfo[i].rpos.locationY = 0;
  }
  return 0;
}

bool groupReady(){
  u8 i = 0;
  for(i = 0; i < GROBOTS; i++){
      if(!bCastInfo[i].isActive){
        return FALSE;
      }
  }
  
  return TRUE;
}

int allAreReady(){
  int i = 0;
  int count = 0;
  if(rbID == activeRb[0]){
	for(i = 0; i < rbCount1; i++){
	  if(bCastInfo[i].isReady == 0){
		count++;
	  }
	}
  }
  if(count == rbCount1)
	return 1;
  return 0;
}

u8 electInitiator(){
  u8 count = 0;
  u8 i = 0;
  while(1){
	halt(1);
	for(i = 0; i < ROBOTS; i++){
	  if(bCastInfo[i].isActive == 1){
		return bCastInfo[i].nodeID;
	  }
	}
	
	count++;
	if(count == INIT_TIME){
//	  if(allAreReady()){
//		beepSing();
//	  }
	  isActive = 1;
	  halt(0.5);
	  break;
	}
  }
  return rbID;
}

extern const typeCoordinate posP1;

u8 electLeader(u8 initor,float x, float y){
  u8 i = 0 ;
  static u8 k = 0;
  for(i = 0; i < GROBOTS; i++){  
	if(bCastInfo[i].nodeID != 0){
	  rbList[k].id = i + 1;
//	  rbList[k].dist = getDistance2(bCastInfo[i].rpos.locationX,bCastInfo[i].rpos.locationY,posEnd.x, posEnd.y);
	  rbList[k].dist = getDistance2(bCastInfo[i].rpos.locationX,bCastInfo[i].rpos.locationY, x - 0.3, y + 0.5);
	  k++;
	}
  }
  
  rbCount1 = k;  
//  if(allAreReady()){
//	beepSing();
//  }
  if(rbCount1 == GROBOTS){
	beepSing();
  }	
#ifdef VISIT_DEMO
  if(rbCount1 < ROBOTS){
	return 0;
  }
#endif
  bubbleSort(rbList,rbCount1);    
  
  for(i = 0;i < rbCount1; i++){
	if(rbList[i].id)
	  nbrList[i] = rbList[i].id;
  }
  halt(1);
  
  for(i = 0;i < rbCount1; i++){
	if(bCastInfo[initor - 1].nbrList[i]){
	  activeRb[i] = bCastInfo[initor - 1].nbrList[i];
	}
  }
  
  halt(1);
  return activeRb[0];
}

/*elect the leader which its ID is not 0 and has the lowest ID*/
uint8 electLeader1(){
  uint8 i = 0;
  int currID = 0;
  int lowestID = 0;
  for(i = 0;i < ROBOTS; i++){
	currID = bCastInfo[i].nodeID;
	if(currID == 0) continue; 
	if(lowestID == 0){
	  lowestID = currID;
	}else if(currID < lowestID){
	  lowestID = currID;
	}
  }
  return lowestID;
}

int reachPoint(float x, float y, float speed){
  typeCoordinate nowPos = GetCoordinate();
  if((nowPos.x >= x) && (nowPos.y >= y))
	return 1;
  else 
	return 0;
}

bool reachWayPoint(float x, float y, int rgFlag){
    float range  = 0;
	typeCoordinate nowp = GetCoordinate();
	float dist = getDistance2(nowp.x, nowp.y, x,y);
	if(rgFlag == LITTLE_RANGE)
	  range = MIN_RANGE;
	else if(rgFlag == BIG_RANGE)
	  range = MAX_RANGE;
	else
	  range = MIN_RANGE;
	
	if( dist < range)
	  return true;
	else
	  return false;
}

bool isNearWall(float x, float y){
  if(x <= 0.05 || y < 0 || x >= 1.35 || y > 0.85)
	return TRUE;
  else
	return FALSE;
}

int slowDown(float x, float y,int index,float gap){
  float speed = 20;
  float cell = 5;
  float speedL = 0;
  float speedR = 0;
  int i = 0;
//  if(gap == LONG_GAP){
//	speed = 25;
//  }
//  
//  if(gap == SHORT_GAP){
//	speed = 15;
//  }
  int side = whichSide(x, y);
  if (side == RIGHT_HAND) { // right side
	speedL = speed;
	speedR = speed + cell;
  } else if (side == LEFT_HAND) { // left side
	speedL = speed + cell;
	speedR = speed;
  } else {
	speedL = speedR = speed;
  }
  SetLeftWheelGivenSpeed(speedL);
  SetRightWheelGivenSpeed(speedR);
  vTaskDelay(200);
  typeCoordinate nowp = GetCoordinate();
  float dist = getDistance2(nowp.x, nowp.y, x, y);
  if( dist < MAX_RANGE)
	return 1;
  if(dist <= 0.12){
	for(i = 0; i < 3; i++){  //Turn right for 90 degrees.
	  SetLeftWheelGivenSpeed(20);
	  SetRightWheelGivenSpeed(35);
	  vTaskDelay(2000);
	  SleepUntilFrontSafe(index,FRONT_SIGHT_LENGTH);
	}
	stage += 1;
	vTaskDelay(500);
	return stage;
  }
  return 0;
}

//如果后面的机器人离自己太远，则减速行驶
int nbrSlowDown(int index,float x, float y){
  int maxIndex = index;
  static u8 backRb = 0;
  static float x1=0,y1=0,x2=0,y2=0;
  static float dist = 0;
  if( (rbCount1 >= 2)){
	backRb = activeRb[maxIndex + 1];	
	x1 = bCastInfo[rbID - 1].rpos.locationX;
	y1 = bCastInfo[rbID - 1].rpos.locationY;
	x2 = bCastInfo[backRb - 1].rpos.locationX;
	y2 = bCastInfo[backRb - 1].rpos.locationY;
	dist = getDistance2(x1,y1,x2,y2);
	  
//	while(dist >= LONG_GAP){
//	  if(slowDown(x,y,LONG_GAP)){
//		return 1;		
//	  }
//	  x1 = bCastInfo[rbID - 1].rpos.locationX;
//	  y1 = bCastInfo[rbID - 1].rpos.locationY;
//	  x2 = bCastInfo[backRb - 1].rpos.locationX;
//	  y2 = bCastInfo[backRb - 1].rpos.locationY;
//	  dist = getDistance2(x1,y1,x2,y2);
//	}
//	
	if(dist >= LONG_GAP){
	  halt(2);
	}
	
	while(dist >= SHORT_GAP){
	  if(slowDown(x,y,index,SHORT_GAP)){
		return 1;
	  }
	  x1 = bCastInfo[rbID - 1].rpos.locationX;
	  y1 = bCastInfo[rbID - 1].rpos.locationY;
	  x2 = bCastInfo[backRb - 1].rpos.locationX;
	  y2 = bCastInfo[backRb - 1].rpos.locationY;
	  dist = getDistance2(x1,y1,x2,y2);
	}
  }
  return 0;
}

//如果离自己最近的一个机器人的距离都大于30cm，就等2秒钟
void nbrNearRangeBalance(){
  int i = 0;
  float length = 0.25;  
  float distance = 0;
  int sum = 0;
  typeCoordinate nowPos = GetCoordinate();
  if(rbCount1 >= 2){
	for(i = 1; i <= rbCount1; i++){
	  int count = 0;
	  float x = bCastInfo[i - 1].rpos.locationX;
	  float y = bCastInfo[i - 1].rpos.locationY;
	  distance = getDistance2(x,y,nowPos.x,nowPos.y);	
	  if(distance > length){
		count = 1;
	  }
	  sum += count;
	}
	
	if(sum >= rbCount1)
	  halt(2);
  }
}

void nbrRangeBalance(){
  u8 i = 0;
  static u8 backRb = 0;
  static float x1=0,y1=0,x2=0,y2=0;
  static float dist = 0;
  if(!bCastInfo[activeRb[0] - 1].isStop){
	if( (rbCount1 >= 2)){
	  for(i = 0; i < rbCount1 - 1; i++){
		if(rbID == activeRb[i]){
		  backRb = activeRb[i+1];
		  break;
		}
	  }
	  
	  x1 = bCastInfo[rbID - 1].rpos.locationX;
	  y1 = bCastInfo[rbID - 1].rpos.locationY;
	  x2 = bCastInfo[backRb - 1].rpos.locationX;
	  y2 = bCastInfo[backRb - 1].rpos.locationY;
	  dist = getDistance2(x1,y1,x2,y2);
	  while(dist >= 0.40){
		halt(2);
		x1 = bCastInfo[rbID - 1].rpos.locationX;
		y1 = bCastInfo[rbID - 1].rpos.locationY;
		x2 = bCastInfo[backRb - 1].rpos.locationX;
		y2 = bCastInfo[backRb - 1].rpos.locationY;
		dist = getDistance2(x1,y1,x2,y2);
	  }
	}
  }
}

u8 getCurrentIndex(){
  static u8 maxIndex = 0;
  u8 i = 0;
  for(i = 0; i < rbCount1; i++){
	if(rbID == activeRb[i]){
	  maxIndex = i;
	}
  }
  return maxIndex;
}

void gotoWayPoint(float x, float y, float speed,int phase) {
  int cell = MAX_MOTORDETA;
  float speedL = speed;
  float speedR = speed;
  
  if(GoalInFront(x,y,phase) == true)
	return;
    
//  if(isNearWall(x,y)){
//	SetLeftWheelGivenSpeed(10);
//	SetRightWheelGivenSpeed(8);
//	return;
//  }
  
  u8 maxIndex = getCurrentIndex();  
//  typeCoordinate nowp = GetCoordinate();
  typeCoordinate nowp = getNowPos(x,y);
  while (1) {		
	float dist = getDistance2(nowp.x, nowp.y, x, y);	
	if (dist < MIN_RANGE) break;		
	
//	if(rbID != activeRb[rbCount1 -1]){ //not the last robot
//	  nbrRangeBalance();
//	}
	
	while((dist <= 0.06)){
	  //rotateTo(x,y,ANGLESPEED,ROTATE_ACCURATE); 
	  rotateTo(x,y,FASTSPEED,ROTATE_ACCURATE); 
	  while(SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH));
	  SetLeftWheelGivenSpeed(30);
	  SetRightWheelGivenSpeed(30);
	  vTaskDelay(300);
	  if((GoalInFront(x,y,phase) == true))
		return;
	}
	
	 while(SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH));
	 whichSide(x, y);
	  SetLeftWheelGivenSpeed(speed);
	  SetRightWheelGivenSpeed(speed);
	  vTaskDelay(400);
	int side = whichSide(x, y);
	if (side == RIGHT_HAND) { // right side
	  speedL = speed;
	  speedR = speed + cell;
	} else if (side == LEFT_HAND) { // left side
	  speedL = speed + cell;
	  speedR = speed;
	} else {
	  speedL = speedR = speed;
	}
	
	while(SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH));
        while(SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH));
	
	SetLeftWheelGivenSpeed(speedL);
	SetRightWheelGivenSpeed(speedR);
	vTaskDelay(400);
	
//	nowp = GetCoordinate();
	nowp = getNowPos(x,y);
  }
}


int readAccelZ(){  	
	typeMPUSensor accelValue;
	accelValue.accelZ = 0;
	float zValue = 0;
	u8 i = 0;
	int sum = 0;
	for(i = 0;i < 10; i++){
	  u8 sTemp = 0;
	  accelValue = halReadMPUSensor();
	  zValue = -accelValue.accelZ;
	  if(zValue < ACCEL_Z){
		sTemp = 1;		
	  }
	  	sum += sTemp;
	  vTaskDelay(10);
	}
	if(sum < ACCEL_TIMES)
	  return 0;
	else
	  return 1;
}

void solvePickUp(){
  if(readAccelZ()){
	isPickup = 1;
	//halt(50000);
	if(rbCount1 >= 2){
	  if(rbID != activeRb[rbCount1-1]){
		float endX = bCastInfo[activeRb[rbCount1 -1] - 1].rpos.locationX;
		float endY = bCastInfo[activeRb[rbCount1 -1] - 1].rpos.locationY;
		rotateFastTo(endX,endY,FASTSPEED,ROTATE_ACCURATE);  
		gotoWayPoint(endX,endY,FASTSPEED,PHASE_ONE);		  
		//setting stage can be done here.
		stage = bCastInfo[activeRb[rbCount1 -1] - 1].type;
		vTaskDelay(20);
		return;
	  }else{
		float endX = bCastInfo[activeRb[rbCount1 - 2] - 1].rpos.locationX;
		float endY = bCastInfo[activeRb[rbCount1 - 2] - 1].rpos.locationY;	 
		rotateFastTo(endX,endY,FASTSPEED,ROTATE_ACCURATE);  
		gotoWayPoint(endX,endY,FASTSPEED,PHASE_ONE);	  		  
		//setting stage can be done here.
		stage = bCastInfo[activeRb[rbCount1 -1] - 1].type;
		vTaskDelay(20);
		return;
	  }
	}
  }
}


void runDrawCircle(float x, float y, float speed,int phase,int ptNum) {
  int cell = MAX_MOTORDETA;
  float speedL = speed;
  float speedR = speed;
  
  isPickup = 0;
  if(GoalInFront(x,y,phase) == true)
	return;
  
  u8 maxIndex = getCurrentIndex();
//  typeCoordinate nowp = GetCoordinate();
  typeCoordinate nowp = getNowPos(x,y);
  while (1){		
	float dist = getDistance2(nowp.x, nowp.y, x, y);	
	if (dist < MIN_RANGE) break;	
	
	if(rbID != activeRb[rbCount1 -1]){ //not the last robot
	  if(nbrSlowDown(maxIndex,x,y))
		return;
	}
	
	while((dist <= 0.12)){  //turn right for 90 degree
	  u8 i = 0;
	  for(i = 0; i < 3; i++){
		SetLeftWheelGivenSpeed(20);
		SetRightWheelGivenSpeed(35);
		vTaskDelay(2000);
		SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH);
	  }	  
	  return;
	}
	
	while(SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH));
	whichSide(x, y);
	SetLeftWheelGivenSpeed(speed);
	SetRightWheelGivenSpeed(speed);
	vTaskDelay(400);
	solvePickUp();
	int side = whichSide(x, y);
	if (side == RIGHT_HAND) { // right side
	  speedL = speed;
	  speedR = speed + cell;
	} else if (side == LEFT_HAND) { // left side
	  speedL = speed + cell;
	  speedR = speed;
	} else {
	  speedL = speedR = speed;
	}
	
	while(SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH));
	while(SleepUntilFrontSafe(maxIndex,FRONT_SIGHT_LENGTH));
	
	SetLeftWheelGivenSpeed(speedL);
	SetRightWheelGivenSpeed(speedR);
	vTaskDelay(400);	
	//nowp = GetCoordinate();  //0.3172,0.5369
	nowp = getNowPos(x,y);
	solvePickUp();
  }
  
  if(!isPickup){
	if(ptNum == 4){
	  stage = 1;
	}else{
	  stage += 1;
	}
  }
}

//void gotoWayPoint1(float x, float y, float speed, int fRbID,int phase) {
//  int cell = MAX_MOTORDETA;
//  float speedL = speed;
//  float speedR = speed;
//  
//  if(GoalInFront1(x,y,phase) == true)
//	return;
//  
//  if(isNearWall(x,y)){
//	SetLeftWheelGivenSpeed(0);
//	SetRightWheelGivenSpeed(0);
//	return;
//  }
//  
//  typeCoordinate nowp = GetCoordinate();
//  while (1) {	
//	float dist = getDistance2(nowp.x, nowp.y, x, y);	
//	if (dist < MAX_RANGE) break;
//	
//	if(rbID != activeRb[rbCount1 -1]){
//	  nbrRangeBalance();
//	}
//	while((dist <= 0.06)){
//	  rotateTo(x,y,ANGLESPEED,ROTATE_ACCURATE); 
////	  rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
//	  while(SleepUntilSafe(fRbID,FRONT_SIGHT_LENGTH));
//	  SetLeftWheelGivenSpeed(speed);
//	  SetRightWheelGivenSpeed(speed);
//	  vTaskDelay(300);
//	  if((GoalInFront1(x,y,phase) == true))
//		return;
//	}
//	 while(SleepUntilSafe(fRbID,FRONT_SIGHT_LENGTH));
//	//while(rotateToSafe(x,y,FRONT_SIGHT_LENGTH,CLOCKWISE)){
//	  SetLeftWheelGivenSpeed(speed);
//	  SetRightWheelGivenSpeed(speed);
//	  vTaskDelay(500);
//	//}
//	int side = whichSide(x, y);
//	if (side == RIGHT_HAND) { // right side
//	  speedL = speed;
//	  speedR = speed + cell;
//	} else if (side == LEFT_HAND) { // left side
//	  speedL = speed + cell;
//	  speedR = speed;
//	} else {
//	  speedL = speedR = speed;
//	}
//	
//	while(SleepUntilSafe(fRbID,FRONT_SIGHT_LENGTH));
//	while(rotateToSafe(x,y,FRONT_SIGHT_LENGTH,CLOCKWISE));
//	
//	SetLeftWheelGivenSpeed(speedL);
//	SetRightWheelGivenSpeed(speedR);
//	vTaskDelay(500);
//	
//	nowp = GetCoordinate();
//  }
//}

void leaderRun(int i, int j){
  float x = (xStart + (j)*100.0)/1000;
  float y = (yStart - (i)*100.0)/1000;
  rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
  gotoWayPoint1(x,y,FASTSPEED,-1,PHASE_ONE);  //leader should run by itself
}

void runSelf(int i, int j){
//  u8 k = 0;
//  u8 frontRb = 0;
//  if(rbID == activeRb[0])
//	frontRb = -1;
//  for(k = 1;k < rbCount1; k++){
//	if(rbID == activeRb[k]){
//	  frontRb = activeRb[k - 1];
//	  break;
//	}
//  } 
  
  float x = (xStart + (j)*100.0)/1000;
  float y = (yStart - (i)*100.0)/1000;
  
  rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);  
  gotoWayPoint(x,y,FASTSPEED,PHASE_ONE);  //leader should run by itself
}

void followLeader() {
  u8 i = 0;
  static u8 frontRb = 0;
  if(bCastInfo[rbID - 1].type == FOLLOWER){
	for(i = 1;i < rbCount1; i++){
	  if(rbID == activeRb[i]){
		frontRb = activeRb[i - 1];
		break;
	  }
	}
  }
  float x = bCastInfo[frontRb - 1].rpos.locationX;
  float y = bCastInfo[frontRb - 1].rpos.locationY;
  
  rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
  gotoWayPoint(x,y,FASTSPEED,PHASE_ONE);
}

void gotoTarget(int i, int j){
  float x = (xStart + j*100.0)/1000;
  float y = (yStart - i*100.0)/1000;
  rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
  gotoWayPoint(x,y,FASTSPEED,PHASE_ONE);
}

