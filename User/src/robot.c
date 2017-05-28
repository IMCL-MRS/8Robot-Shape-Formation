#include "robot.h"
#include "vInfoList.h"
#include <task.h>

extern rbNode bCastInfo[ROBOTS];
extern u8 activeRb[ROBOTS];

//0 - 360
float CalibrateNorth2X(void){
  float nAngle = ReadMagSensorAngle2North();
  if((nAngle > FRONT2X) && (nAngle < 180)){
    nAngle -= FRONT2X;
  }else{
    nAngle += 360 - FRONT2X;
  }
  return nAngle;
}

//0 - 360
//float CalibrateNorth2X(void){
//  float nAngle = ReadMagSensorAngle2North();
//  float southAngle = -270 + _Y2NORTH;
//  if((nAngle >= -southAngle) && (nAngle <= 180)){
//      nAngle -= southAngle;
//  }else{
//      nAngle += 360 - southAngle;
//  }
//  return nAngle;
//}

// 0~-180, 0~180
//float CalibrateNorth2_Y(void){
//  float nAngle = ReadMagSensorAngle2North();
//  float southAngle = 180 - _Y2NORTH;
//  if((nAngle >= -southAngle) && (nAngle <= 180)){
//    nAngle -= _Y2NORTH;
//  }else{
//    nAngle += 180 + southAngle;
//  }
//  return nAngle;
//}


//float CalibrateNorth2_Y(void){
//  typeMagSensor magSensor;
//  float edgeLong=0;
//  float compX, compY;
//  float angleReturn;
//  
//  magSensor =  ReadMagSensor();
//  compX = magSensor.magX-MAG_SENSOR_X;
//  compY = magSensor.magY-MAG_SENSOR_Y;
//  
//  edgeLong= sqrt(compX*compX + compY*compY); 
//  
//  angleReturn = (180.0f/3.14) * (acos(compY/edgeLong));  
//  
//  if (compX < 0){    
//	angleReturn=0-angleReturn + _Y2NORTH;
//  }   
//  
//  if (compX > 0){    
//	angleReturn += _Y2NORTH;
//	if(angleReturn > 180){
//	  angleReturn -= 360;
//	}
//  }  
//  
//  return angleReturn;
//}

//angle -180 to 180
float ReadMagSensorAngle2North(void) {
  typeMagSensor magSensor;
  float edgeLong=0;
  float compX, compY;
  float angleReturn;
  
  magSensor =  ReadMagSensor();
  compX = magSensor.magX-MAG_SENSOR_X;
  compY = magSensor.magY-MAG_SENSOR_Y;
  
  edgeLong= sqrt(compX*compX + compY*compY);
  
  angleReturn = (180.0f/3.14) * (acos(compY/edgeLong));  
  
  if (compX < 0){    
	angleReturn=0-angleReturn;
  }   

  return angleReturn;
}

////////////////////////////////////////////////////////////////////////////////
//calculate coordinate
////////////////////////////////////////////////////////////////////////////////
typeCoordinate GetCoordinate(void) {
  float disB1, disB2;
  static typeCoordinate coordinateC;
  
  disB1 = GetDistanse2B1();
  disB2 = GetDistanse2B2();
  
  coordinateC.y = (disB1*disB1 - disB2*disB2 + DISTANSE_B1_2_B2*DISTANSE_B1_2_B2)/(2*DISTANSE_B1_2_B2);
  coordinateC.x = sqrt(disB1*disB1 - DISTANSE_B1_2_B2*DISTANSE_B1_2_B2 - coordinateC.y*coordinateC.y);
  //check weather x axis is negative or not.
//  float xPos = disB1*disB1 - DISTANSE_B1_2_B2*DISTANSE_B1_2_B2 - coordinateC.y*coordinateC.y;
//  if(xPos >= 0){
//	coordinateC.x = sqrt(xPos);
//  }
  
  return coordinateC;
}


#if 0
///////////////////////////////
/////////////////////////////////////////////////
//angle: range from -180 to 180 
////////////////////////////////////////////////////////////////////////////////
void ControlRobotRotate(float angle, float speed) {
  float givenAngle;
  float timeInterval;
  if (FloatAbs(speed) > MAX_ROTATE_SPEED) {
	if (speed<0) {
	  speed = 0 - MAX_ROTATE_SPEED;
	}
	else {
	  speed = MAX_ROTATE_SPEED;
	}
  }
  if (angle<0) {
	givenAngle = -angle/180*PI;
	SetLeftWheelGivenSpeed(speed);
	SetRightWheelGivenSpeed(0-speed);
  }
  else {
	givenAngle = angle/180*PI;
	SetLeftWheelGivenSpeed(0-speed);
	SetRightWheelGivenSpeed(speed);
  }
    
  //timeInterval = 1000*givenAngle*(WHEEL_L_R_DISTANCE)/(2*speed); //(L = n*PI*r/180)
  //vTaskDelay((unsigned long)timeInterval);
  
  //////////////////////////////////////////////////////////////////////////////
  //1¡ã=0.0174532922222222»¡¶È
  //d=60mm
  //1¡ã½Ç¶ÈÏàµ±ÓÚÂÖ×Ó×ß0.534070742mm
  //µç»ú·´À¡Ã¿ºÁÃ×¶ÔÓ¦µÄÂö³åÊý570.4113257715211¸öÂö³å, 4±¶Æµ 
  //1¡ã=1218.56¸öÂö³å
  //²î1µ½2¶ÈµÄÑù×Ó
  //////////////////////////////////////////////////////////////////////////////
  int32_t angle2Pulse; 
  extern int64_t pulseAccL, pulseAccR;
    
  angle2Pulse = (int32_t)(angle*114100/400);  //Èç¹ûÓÐÎó²îÐÞ¸Ä121856/400Õâ¸öÊÇÊý¾Ý£¬Îó²îÀ´×ÔÓÚ²âÁ¿Îó²î
     
  pulseAccL = 0;
  pulseAccR = 0;
  
  if (angle2Pulse >0){
    while(pulseAccR < angle2Pulse) {
      vTaskDelay(5);
    }
  }
  else {
    while(pulseAccR > angle2Pulse) {
      vTaskDelay(5);
    }

  }
  
 
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
}
#endif

#if 1
void ControlRobotRotate(float angle, float speed) {
  float givenAngle;
  float timeInterval;
  if (FloatAbs(speed) > MAX_ROTATE_SPEED) {
	if (speed<0) {
	  speed = 0 - MAX_ROTATE_SPEED;
	}
	else {
	  speed = MAX_ROTATE_SPEED;
	}
  }
  if (angle<0) {
	givenAngle = -angle/180*PI;
	SetLeftWheelGivenSpeed(speed);
	SetRightWheelGivenSpeed(0-speed);
  }
  else {
	givenAngle = angle/180*PI;
	SetLeftWheelGivenSpeed(0-speed);
	SetRightWheelGivenSpeed(speed);
  }
  timeInterval = 1000*givenAngle*(WHEEL_L_R_DISTANCE)/(2*speed); //(L = n*PI*r/180)
  vTaskDelay((unsigned long)timeInterval);
  
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
}
#endif
////////////////////////////////////////////////////////////////////////////////
//angle: control Robot go from current position to given position
////////////////////////////////////////////////////////////////////////////////
void ControlRobotFollowLine(float x0, float y0, float x1, float y1, float speed){
  
}
////////////////////////////////////////////////////////////////////////////////
//angle: control Robot go from current position to given position
////////////////////////////////////////////////////////////////////////////////
void ControlRobotFollowCircle(float x, float y, float speed){
  
}

////////////////////////////////////////////////////////////////////////////////
//angle: given two coordinates to calculate line dir
////////////////////////////////////////////////////////////////////////////////
float GetLineDirection(float x0, float y0, float x1, float y1) {
  float angle;
  float eLong;
  float comX, comY;
  
  comX = x1-x0;
  comY = y1-y0;
  
  eLong = sqrt(comX*comX + comY*comY);
  angle = (180/M_PI) * acos(FloatAbs(comX)/eLong);  //0 to 90 degree
  
  if ((comX>=0) && (comY>=0)) {     //1
	angle = angle+90;
  }
  if ((comX<=0) && (comY>=0)) {     //2 
	angle = 90-angle-180;
  }
  if ((comX<=0) && (comY<=0)) {     //3
	angle = angle-90;
  }
  if ((comX>=0) && (comY<=0)) {     //4
	angle = 90 - angle;
  }
  return angle;
}

//0 - 360 degree
float GetLineDirectionX(float x0, float y0, float x1, float y1) {
  float angle;
  float eLong;
  float comX, comY;
  
  comX = x1-x0;
  comY = y1-y0;
  
  eLong = sqrt(comX*comX + comY*comY);
  angle = (180/M_PI) * acos(FloatAbs(comX)/eLong);  //0 to 90 degree
  
  if ((comX>=0) && (comY>=0)) {     //1
	angle = angle;
  }
  if ((comX<=0) && (comY>=0)) {     //2 
	angle = 180 - angle;
  }
  if ((comX<=0) && (comY<=0)) {     //3
	angle = 180 + angle;
  }
  if ((comX>=0) && (comY<=0)) {     //4
	angle = 360 - angle;
  }
  return angle;
}


bool RobotInRange(float x0, float y0, float x1, float y1) {
  if ((FloatAbs(x1-x0)>0.03f) || (FloatAbs(y1-y0)>0.03f)){
	return false;
  }
  return true;
}

void northRotate(float x, float y, float speed){
  static typeCoordinate coordinate;
  float lineDir;
  float robotDir;
  
  //1. calculate coordinate
  coordinate = GetCoordinate();
  
  //2. calculate direction of line
  lineDir = GetLineDirection(coordinate.x, coordinate.y, x, y);
  //3. turn dir to line
  robotDir = ReadMagSensorAngle2North();
  
  if ((lineDir-robotDir)<-180) {
	ControlRobotRotate(lineDir-robotDir+360-20, 5);
  }
  else if ((lineDir-robotDir)>180) {
	ControlRobotRotate(lineDir-robotDir-360+20, 5);
  }
  else {
	ControlRobotRotate(lineDir-robotDir, 5);
  }
}

////////////////////////////////////////////////////////////////////////////////
//angle: control Robot go from current position to given position
////////////////////////////////////////////////////////////////////////////////
void ControlRobot2Position(float x, float y, float speed){
  static typeCoordinate coordinate;
  float speedL, speedR;
  //float calY;
  float lineDir;
  float robotDir;
  
  speedL = speed;
  speedR = speed;
  
  //1. calculate coordinate
  coordinate = GetCoordinate();
  //2. y=kx+b, calculate line para k and b
  float k, b;
  k = (y-coordinate.y)/(x-coordinate.x);
  b = y-k*x;
  //3. calculate direction of line
  lineDir = GetLineDirection(coordinate.x, coordinate.y, x, y);
  //4. turn dir to line
  robotDir = ReadMagSensorAngle2North();
  float turnangle = lineDir - robotDir;
  while (turnangle < 0) turnangle += 360;
  while (turnangle >= 360) turnangle -= 360;
  if (turnangle > 180) turnangle = 360 - turnangle;
  ControlRobotRotate(turnangle, 5);
  /*
  if ((lineDir-robotDir)<-180) {
  ControlRobotRotate(lineDir-robotDir+360-20, 5);
}
  else if ((lineDir-robotDir)>180) {
  ControlRobotRotate(lineDir-robotDir-360+20, 5);
}
  else {
  ControlRobotRotate(lineDir-robotDir, 5);
}
  */
  
  SetLeftWheelGivenSpeed(1);
  SetRightWheelGivenSpeed(1);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(3);
  SetRightWheelGivenSpeed(3);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(5);
  SetRightWheelGivenSpeed(5);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(10);
  SetRightWheelGivenSpeed(10);
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(15);
  SetRightWheelGivenSpeed(15); 
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(20);
  SetRightWheelGivenSpeed(20);
  vTaskDelay(1000);
  
  coordinate = GetCoordinate();
  while (RobotInRange(coordinate.x,coordinate.y, x, y)==false){
	/*
	if (coordinate.y>(k*coordinate.x+b)) {
	speedL = 25;
	speedR = 20;      
  }
	else {
	speedL = 20;
	speedR = 25;
  }
	*/
	if (coordinate.y>(k*coordinate.x+b)+0.01) {
	  speedL = 23;
	  speedR = 20;
	}
	else if (coordinate.y+0.0<(k*coordinate.x+b)) {
	  speedL = 20;
	  speedR = 23;
	}
	else {
	  robotDir = ReadMagSensorAngle2North();
	  if ((lineDir-robotDir)>20){
		speedL = 20;
		speedR = 23;
	  }
	  else if ((lineDir-robotDir)<-20){
		speedL = 23;
		speedR = 20;
	  }
	  
	  else{
		speedL = 20;
		speedR = 20;
	  }
	}
	
	SetLeftWheelGivenSpeed(speedL);
	SetRightWheelGivenSpeed(speedR);
	vTaskDelay(500);
	coordinate = GetCoordinate();
  }
  
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  vTaskDelay(100000);
  
  asm("NOP");
}

////////////////////////////////////////////////////////////////////////////////
//1. check robot stability 
//2. get robot's direction
////////////////////////////////////////////////////////////////////////////////
void RobotPrepare(void) {
  //1. make robot stop
  typeWheelSpeed speed;
  typeMPUSensor mpuAccel;
  u8 retryTimes=0;
  static typeCoordinate startCoordinate, endCoordinate;
  float diffX, diffY;
  float angle;
  
  MotorRunEnable(true);
  speed.left  = 0;
  speed.right = 0;
  SetWheelGivenSpeed(speed);
  speed = GetWheelSpeed();
  while ((speed.left > 1) || (speed.right > 1)); {
	vTaskDelay(50);
	speed = GetWheelSpeed();
  }
  MotorRunEnable(false);
  //2. read MPU accel sensor, check robot's stalibility
  mpuAccel.accelX = 20;
  mpuAccel.accelY = 20;
  
  while ( retryTimes< ROBOT_STABLE_RETRY_THR ){
	mpuAccel = ReadMPUSensor();
	if ((FloatAbs(mpuAccel.accelX)<ROBOT_STABLE_ACCEL_X_THR) && (FloatAbs(mpuAccel.accelY)<ROBOT_STABLE_ACCEL_X_THR)){ 
	  retryTimes++;
	}
	else {
	  retryTimes = 0;
	}
	vTaskDelay(50);
  }
  //3. rotate to find x min compass 
  startCoordinate = GetCoordinate();
  
  MotorRunEnable(true);
  speed.left  = 10;
  speed.right = 10;
  SetWheelGivenSpeed(speed);
  vTaskDelay(4000);
  speed.left  = 0;
  speed.right = 0;
  SetWheelGivenSpeed(speed);
  
  endCoordinate = GetCoordinate();
  diffX = endCoordinate.x - startCoordinate.x;
  diffY = endCoordinate.y - startCoordinate.y;
  
  angle =(180/3.14)*acos(diffX / sqrt(diffY*diffY + diffX*diffX));
  
  if (diffY < 0) {
	angle = 0-angle;
  }
  SetRobotAngle(angle);
  
  //4. 
}

float robotAngle=0;
void SetRobotAngle(float angle) {
  robotAngle = angle;
}

float GetRobotAngle(void) {
  return robotAngle;
}

float getDistance2(float Ax, float Ay, float Bx, float By) {
  return sqrt( (Ax-Bx)*(Ax-Bx) + (Ay-By)*(Ay-By));
}

void rotateTo(float x, float y, float speed,int flag) {
  typeCoordinate start = GetCoordinate();
//  float lineDir = GetLineDirection(start.x, start.y, x, y);
//  float robotDir = CalibrateNorth2_Y();
  float lineDir = GetLineDirectionX(start.x, start.y, x, y);
  float robotDir = CalibrateNorth2X();
  float turnangle = lineDir - robotDir;
  if (turnangle < -180) turnangle += 360 ;
  if (turnangle > 180) turnangle -= 360;
  if(flag == ROTATE_LARGER_15)
  {
	if(FloatAbs(turnangle) <= 15)    
	  return;
  }
  
  if(flag == ROTATE_LARGER_25)  //used for runStraight
  {
	if(FloatAbs(turnangle) <= 25)   
	  return;
  }
  
  ControlRobotRotate(turnangle, speed);
}

void rotateFastTo(float x, float y, float speed,int flag) {
  rotateTo(x,y,FASTSPEED,ROTATE_ACCURATE);
  rotateTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
}

//void rotateToNorthAngle(float tar, float speed) {
//  float ang = tar - CalibrateNorth2_Y();
//  if (ang < -180) ang += 360;
//  if (ang > 180) ang -= 360;
//  ControlRobotRotate(ang, speed);
//}

void rotateToNorthAngle(float tar, float speed) {
  float ang = tar - CalibrateNorth2X();
  if (ang < -180) ang += 360 ;
  if (ang > 180) ang -= 360;
  ControlRobotRotate(ang, speed);
}

void halt(float time){
  SetLeftWheelGivenSpeed(0);
  SetRightWheelGivenSpeed(0);
  vTaskDelay((int)(time*1000));
}

// return which side C is in related to AB
int whichSideByPoint(float ax, float ay, float bx, float by, float cx, float cy) {
  float eps = (ax-bx)*(cx-bx) + (cy-by)*(ay-by);
  if (eps < 0) return -2; // C is beyond AB's vertical line
  eps = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
  if (eps < -MIN_RANGE) return -1; // right side
  if (eps > MIN_RANGE) return 1; // left side
  return 0; // on the line
}

int whichSide(float x, float y){
  typeCoordinate start = GetCoordinate();
//  typeCoordinate start = getNowPos();
//  float lineDir = GetLineDirection(start.x, start.y, x, y);
//  float robotDir = CalibrateNorth2_Y();
  float lineDir = GetLineDirectionX(start.x, start.y, x, y);
  float robotDir = CalibrateNorth2X();
  float turnangle = lineDir - robotDir;
  if (turnangle < -180) turnangle += 360;
  if (turnangle > 180) turnangle -= 360;
  
  if(FloatAbs(turnangle) <= 25){
	if(turnangle > 0)
	  return 1; //right side
	if(turnangle < 0)
	  return -1;  //left side
  }
  
  if(FloatAbs(turnangle) > 25){
	ControlRobotRotate(turnangle, MAX_ROTATE_SPEED);
  }
  return 0;  // on the line
}

float r2rAngle(float lineDir, float myDir){
  float turnangle = lineDir - myDir;
  if (turnangle < -180) turnangle += 360;
  if (turnangle > 180) turnangle -= 360;
  return FloatAbs(turnangle);
}

////////////////////////////////////////////////////////////////////////////////
//check collision between robots
////////////////////////////////////////////////////////////////////////////////
bool ObjectInFront(float dir,typeCoordinate coordinate,float *interAngle,float length,int *leftOrRight) {
  float myDir;
  typeCoordinate myCoordinate;
  float disRobot2Robot;
  float lineDir;
  float leftAngle,rightAngle,leftAngle1,rightAngle1;
  
  myDir        = ReadMagSensorAngle2North();
  myCoordinate = GetCoordinate();
  
  disRobot2Robot = sqrt((myCoordinate.x-coordinate.x)*(myCoordinate.x-coordinate.x)+(myCoordinate.y-coordinate.y)*(myCoordinate.y-coordinate.y));
  lineDir = GetLineDirection(myCoordinate.x, myCoordinate.y, coordinate.x, coordinate.y);
  
  *interAngle = r2rAngle(lineDir,myDir);
  
  if (disRobot2Robot < length) {
	leftAngle  = myDir - FOV;   //when -180 < myDir < -180-FOV below -180,eg: -230;
	rightAngle = myDir + FOV;
	if(leftAngle < -180){
	  leftAngle = -180;
	  rightAngle1 = 180;
	  leftAngle1  = 180 - (FOV - (180 + myDir));
	}
	
	if(rightAngle > 180){
	  rightAngle = 180;
	  leftAngle1 = -180;
	  rightAngle1 = -180 + (FOV - (180-myDir));
	}
	
	if ((lineDir > leftAngle) && (lineDir < rightAngle)) {
	  if((leftAngle < lineDir) && leftAngle <= myDir){
		*leftOrRight = leftHand;
	  }
	  
	  if((myDir <= lineDir) && lineDir < rightAngle){
		*leftOrRight = rightHand;
	  }
	  return true;
	}
	
	if ((lineDir >= leftAngle1) && (lineDir <= rightAngle1)) {
	  if((leftAngle < lineDir) && leftAngle <= myDir){
		*leftOrRight = leftHand;
	  }
	  
	  if((myDir <= lineDir) && lineDir < rightAngle){
		*leftOrRight = rightHand;
	  }
	  return true;
	}
  }
  return false;                              
}

bool GoalInFront1(float x, float y,int phase){
  typeCoordinate nowp = GetCoordinate();
  float dist = getDistance2(nowp.x, nowp.y, x,y);
  if( dist < MAX_RANGE)
	return true;
  else
	return false;
}

bool GoalInFront(float x, float y,int phase){
  typeCoordinate nowp = GetCoordinate();
  float dist = getDistance2(nowp.x, nowp.y, x,y);
  if( dist < MIN_RANGE)
	return true;
  else
	return false;
}

u8 rotateToSafe(float x,float y){
    u8 flag = 0;
    u8 i = 0;
	float dir;
    typeCoordinate c;
	for (i = 0; i < ROBOTS; i++){
	  /*if check itself,then return*/
	  //add code here
	  if((i+1) == rbID)
		continue;
	  if (bCastInfo[i].nodeID == (i+1)) {
		//get my information
		dir = bCastInfo[i].angle2n;
        c.x  = bCastInfo[i].rpos.locationX;
        c.y  = bCastInfo[i].rpos.locationY;
		
		float interAngle = 0;
		int leftOrRight = 0;
		if (ObjectInFront(dir, c, &interAngle,FRONT_SIGHT_LENGTH,&leftOrRight) == true){
		  flag++;
//		  halSetLedStatus(LED_BLUE, LED_ON);  
		  if((leftOrRight == -1)){
			  ControlRobotRotate(-(interAngle+FOV+EYEANGLE), FASTSPEED);
		  }else if((leftOrRight == 1)){
			  ControlRobotRotate(-(FOV-interAngle+EYEANGLE), FASTSPEED);
		  }else{
			 ControlRobotRotate(-(FOV + EYEANGLE), FASTSPEED);
		  }
		}
	  }
   } /* end of for*/
   if( flag == 3){  //have 3 robots in front of it. 
	  halt(2);
	  rotateTo(x,y,ANGLESPEED,ROTATE_ACCURATE);	
   }
   return flag;
}

extern rbNode bCastInfo[ROBOTS];

u8 rotateToSafe1(float x,float y,float length,int check){
  u8 flag = 0;
  u8 i = 0;
  float dir;
  typeCoordinate c;
  for (i = 0; i < ROBOTS; i++){
	/*if check itself,then return*/
	//add code here
	if((i+1) == rbID)
	  continue;
	if (bCastInfo[i].nodeID == (i+1)) {
	  //get my information	  
	  dir = bCastInfo[i].angle2n;
//    s.left   = bCastInfo[i].speedL;
//    s.right  = bCastInfo[i].speedR;
	  c.x  = bCastInfo[i].rpos.locationX;
	  c.y  = bCastInfo[i].rpos.locationY;
	  
	  float interAngle = 0;
	  int leftOrRight = 0;
	  if (ObjectInFront(dir, c, &interAngle,length,&leftOrRight) == true){
		flag = bCastInfo[i].nodeID;
		halSetLedStatus(LED_BLUE, LED_ON);  
		if(check == 1){
		  if((leftOrRight == -1)){
			ControlRobotRotate(-(interAngle+FOV+EYEANGLE), MAX_ROTATE_SPEED);
		  }else if((leftOrRight == 1)){
			ControlRobotRotate(-(FOV-interAngle+EYEANGLE), MAX_ROTATE_SPEED);
		  }else{
			ControlRobotRotate(-(FOV + EYEANGLE), MAX_ROTATE_SPEED);
		  }
		}
		
		if(check == -1){
		  if((leftOrRight == -1)){
			ControlRobotRotate((interAngle+FOV+EYEANGLE), MAX_ROTATE_SPEED);
		  }else if((leftOrRight == 1)){
			ControlRobotRotate((FOV-interAngle+EYEANGLE), MAX_ROTATE_SPEED);
		  }else{
			ControlRobotRotate((FOV + EYEANGLE), MAX_ROTATE_SPEED);
		  }
		}
	  }
	}
  } /* end of for*/
  return flag;
}

u8 sleepSafe(float length){
  u8 flag = 0;
  u8 i = 0;
  float dir;
  typeCoordinate c;
  for (i = 0; i < ROBOTS; i++){
	/*if check itself,then return*/
	//add code here
	if((i+1) == rbID)
	  continue;
	if (bCastInfo[i].nodeID == (i+1)) {
	  //get my information
	  dir = bCastInfo[i].angle2n;
	  c.x  = bCastInfo[i].rpos.locationX;
	  c.y  = bCastInfo[i].rpos.locationY;
	  
	  float interAngle = 0;
	  int leftOrRight = 0;
	  if (ObjectInFront(dir, c, &interAngle,length,&leftOrRight) == true){
		flag++;
		halSetLedStatus(LED_BLUE, LED_ON);  
		halt(5);		  
	  }
	}
  } /* end of for*/   
  return flag;
}

void ControlRobotgo2Position(float x, float y, float speed,int flag) {
  //float speedL = speed;
  //float speedR = speed;
  typeCoordinate start = GetCoordinate();
  if(GoalInFront(x,y,0) == true)
	return;
  SetLeftWheelGivenSpeed(1);
  SetRightWheelGivenSpeed(1);
  vTaskDelay(500);
  
  if(GoalInFront(x,y,0) == true)
	return;
  SetLeftWheelGivenSpeed(3);
  SetRightWheelGivenSpeed(3);
  vTaskDelay(500);
  
  if(GoalInFront(x,y,0) == true)
	return;
  rotateToSafe(x,y);
  SetLeftWheelGivenSpeed(5);
  SetRightWheelGivenSpeed(5);
  vTaskDelay(500);
  
  if(GoalInFront(x,y,0) == true)
	return;
  rotateToSafe(x,y);
  SetLeftWheelGivenSpeed(10);
  SetRightWheelGivenSpeed(10);
  vTaskDelay(300);
  
  if(GoalInFront(x,y,0) == true)
	return;
  rotateToSafe(x,y);
  SetLeftWheelGivenSpeed(15);
  SetRightWheelGivenSpeed(15); 
  vTaskDelay(200);
  
  if(GoalInFront(x,y,0) == true)
	return;
  rotateToSafe(x,y);
  SetLeftWheelGivenSpeed(20);
  SetRightWheelGivenSpeed(20);
  rotateToSafe(x,y);
  if(GoalInFront(x,y,0) == true)
	return;
  u8 i = 0;
  while (1) {	
	for(i = 0;i < CHECK; i++){
	  rotateToSafe(x,y);  
	  SetLeftWheelGivenSpeed(30);
	  SetRightWheelGivenSpeed(30);
	  vTaskDelay(300);
	  if(GoalInFront(x,y,0) == true)
		return;
	}
    rotateTo(x,y,FASTSPEED,ROTATE_LARGER_15);	
	//go to target point
	SetLeftWheelGivenSpeed(30);
	SetRightWheelGivenSpeed(30);
	vTaskDelay(300);
	if(GoalInFront(x,y,0) == true)
	  return;
 }  /* end of while */
} /*end of ControlRobotgo2Position*/

void specialRun(float x, float y,float speed) {
  rotateTo(x,y,ANGLESPEED,ROTATE_ACCURATE); 
  
  int cell = 6;
  float speedL = speed;
  float speedR = speed;
  
  if(GoalInFront(x,y,0) == true)
	return;
  
  typeCoordinate nowp = getNowPos(x,y);
  while (1) {		
	float dist = getDistance2(nowp.x, nowp.y, x, y);	
	if (dist < MIN_RANGE) break;		
	
	while((dist <= 0.06)){
	  rotateTo(x,y,FASTSPEED,ROTATE_ACCURATE); 
	  rotateToSafe(x,y);
	  SetLeftWheelGivenSpeed(speedL);
	  SetRightWheelGivenSpeed(speedR);
	  vTaskDelay(300);
	  if((GoalInFront(x,y,0) == true))
		return;
	}
	
	rotateToSafe(x,y);
	whichSide(x, y);
	SetLeftWheelGivenSpeed(speed);
	SetRightWheelGivenSpeed(speed);
	vTaskDelay(300);
	 if((GoalInFront(x,y,0) == true))
		return;
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
	
	rotateToSafe(x,y);
	
	SetLeftWheelGivenSpeed(30);
	SetRightWheelGivenSpeed(30);
	vTaskDelay(300);
	if((GoalInFront(x,y,0) == true))
	  return;
	nowp = getNowPos(x,y);
  }
} /*end of ControlRobotgo2Position*/


#if 0
void runStraight(float x, float y, float speed) {
  typeCoordinate start = GetCoordinate();
  if(GoalInFront(x,y) == true)
	return;
  sleepSafe(FRONT_SIGHT_LENGTH);  
  typeCoordinate nowp;
  int tempTime = 500;
  int  nearTime = 0;
  while (1) {	
	//	for(i = 0;i < 1; i++){
	while(1){
	  if(sleepSafe(FRONT_SIGHT_LENGTH) == 0)
		break;
	}
	SetLeftWheelGivenSpeed(speed);
	SetRightWheelGivenSpeed(speed);
	vTaskDelay(tempTime - nearTime);
	if(GoalInFront(x,y) == true)
	  return;
	//	}
	nowp =  GetCoordinate();
	if(nowp.x >= x)
	  return;
	float dist = getDistance2(nowp.x, nowp.y, x, y);
	if(dist >= 0.06)
	{
	  rotateTo(x,y,ANGLESPEED,0);	
	}
	else {
	  rotateTo(x,y,ANGLESPEED,1);	
	  //	  nearTime = 200;
	}
  }  /* end of while */
} /*end of ControlRobotgo2Position*/
#endif

#if 0
void runStraight(float x, float y, float speed, int flag) {
  float speedL = speed;
  float speedR = speed;
  typeCoordinate start = GetCoordinate();
  
  typeCoordinate nowp = GetCoordinate();
  while (1) {	
	int side = whichSide(start.x,start.y, x, y, nowp.x, nowp.y);
	float dist = getDistance2(nowp.x, nowp.y, x, y);	
	
	while(1){
	  if(sleepSafe(FRONT_SIGHT_LENGTH) == 0)
		break;
	}
	while(dist <= 0.10){
	  rotateTo(x,y,ANGLESPEED,1); 
	  SetLeftWheelGivenSpeed(speedL);
	  SetRightWheelGivenSpeed(speedR);
	  vTaskDelay(300);
	  if(GoalInFront(x,y) == true)
		return;
	}
	if (side == -2 || dist < EPS) break;
	if (side == -1) { // right side
	  speedL = speed;
	  speedR = speed+5;
	} else if (side == 1) { // left side
	  speedL = speed+5;
	  speedR = speed;
	} else {
	  speedL = speedR = speed;
	}
	SetLeftWheelGivenSpeed(speedL);
	SetRightWheelGivenSpeed(speedR);
	vTaskDelay(300);
	
	nowp = GetCoordinate();
  }
#ifdef DEBUG
  halt(2);
#endif
}
#endif

#if 0
void runStraight(float x, float y, float speed) {
  float speedL = speed;
  float speedR = speed;
  typeCoordinate start = GetCoordinate();
  SetLeftWheelGivenSpeed(1);
  SetRightWheelGivenSpeed(1);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(3);
  SetRightWheelGivenSpeed(3);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(5);
  SetRightWheelGivenSpeed(5);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(10);
  SetRightWheelGivenSpeed(10);
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(15);
  SetRightWheelGivenSpeed(15); 
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(20);
  SetRightWheelGivenSpeed(20);
  typeCoordinate nowp = GetCoordinate();
  while (1) {
	sleepSafe(FRONT_SIGHT_LENGTH);  
	int side = whichSide(start.x,start.y, x, y, nowp.x, nowp.y);
	float dist = getDistance2(nowp.x, nowp.y, x, y);
	if (dist >= 0.08){
	  rotateTo(x,y,ANGLESPEED,2); 
	  //#ifdef CONFIG_ROBOT1	  
	  //	  	rotateTo(posM.x,posM1.y,ANGLESPEED,2); 
	  //#elif defined(CONFIG_ROBOT2)
	  //		rotateTo(posM.x,posM2.y,ANGLESPEED,2); 
	  //#elif defined(CONFIG_ROBOT3)
	  //		rotateTo(posM.x,posM3.y,ANGLESPEED,2);	
	  //#elif defined(CONFIG_ROBOT4)
	  //		rotateTo(posM.x,posM4.y,ANGLESPEED,2);
	  //#endif		
	}
	if (side == -2 || dist < EPS) break;
	if (side == -1) { // right side
	  speedL = speed;
	  speedR = speed+3;
	} else if (side == 1) { // left side
	  speedL = speed+3;
	  speedR = speed;
	} else {
	  speedL = speedR = speed;
	}
	SetLeftWheelGivenSpeed(speedL);
	SetRightWheelGivenSpeed(speedR);
	vTaskDelay(300);
	nowp = GetCoordinate();
  }
#ifdef DEBUG
  halt(2);
#endif
}
#endif

#if 0
void goStraight(float x, float y, float speed, int flag) {
  int cell = 8;
  if(cell == 3)
	cell = 3;
  float speedL = speed;
  float speedR = speed;
  typeCoordinate start = GetCoordinate();
  SetLeftWheelGivenSpeed(1);
  SetRightWheelGivenSpeed(1);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(3);
  SetRightWheelGivenSpeed(3);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(5);
  SetRightWheelGivenSpeed(5);
  vTaskDelay(500);
  SetLeftWheelGivenSpeed(10);
  SetRightWheelGivenSpeed(10);
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(15);
  SetRightWheelGivenSpeed(15); 
  vTaskDelay(300);
  SetLeftWheelGivenSpeed(20);
  SetRightWheelGivenSpeed(20);
  typeCoordinate nowp = GetCoordinate();
  while (1) {	
	int side = whichSide(start.x,start.y, x, y, nowp.x, nowp.y);
	float dist = getDistance2(nowp.x, nowp.y, x, y);	
	//    rotateToSafe(x,y,FRONT_SIGHT_LENGTH,flag); 
	if(flag == 3)
	{
	  while(1){
		if(sleepSafe(FRONT_SIGHT_LENGTH) == 0)
		  break;
	  }
	}
	while(dist <= 0.06){
	  rotateTo(x,y,ANGLESPEED,1); 
	  SetLeftWheelGivenSpeed(speedL);
	  SetRightWheelGivenSpeed(speedR);
	  vTaskDelay(300);
	  if(GoalInFront(x,y) == true)
		return;
	}
	if (side == -2 || dist < EPS) break;
	if (side == -1) { // right side
	  speedL = speed;
	  speedR = speed+cell;
	} else if (side == 1) { // left side
	  speedL = speed+cell;
	  speedR = speed;
	} else {
	  speedL = speedR = speed;
	}
	SetLeftWheelGivenSpeed(speedL);
	SetRightWheelGivenSpeed(speedR);
	vTaskDelay(300);
	
	nowp = GetCoordinate();
  }
#ifdef DEBUG
  halt(2);
#endif
}
#endif

int SleepUntilSafe(u8 index, float length){
  u8 flag = 0;
  u8 i = 0;
  float dir;
  typeCoordinate c;
  for (i = 0; i < ROBOTS; i++){
	/*if check itself,then return*/
	//add code here
	if((i+1) == rbID)
	  continue;
	if (bCastInfo[i].nodeID == (i+1)) {
	  //get my information	  
	  dir = bCastInfo[i].angle2n;
	  c.x  = bCastInfo[i].rpos.locationX;
	  c.y  = bCastInfo[i].rpos.locationY;
	  
	  float interAngle = 0;
	  int leftOrRight = 0;
	  if (ObjectInFront(dir, c, &interAngle,length,&leftOrRight) == true){		
		flag++;
		halt(flag*4);
		//halSetLedStatus(LED_BLUE, LED_ON);  	
	  }
	}
  } /* end of for*/
  return flag;
}

//å¦‚æžœå‰é¢æœ‰ä¸ªæœºå™¨äººåœ¨è·ç¦»è‡ªå·±13mmèŒƒå›´å†…ï¼Œåˆ™æš‚åœè¡Œé©¶ 2'
int SleepUntilFrontSafe(u8 index, float length){
  u8 maxIndex = index;
  u8 flag = 0;
  typeCoordinate nowp = GetCoordinate();
  typeCoordinate front;
  if(maxIndex == 0)
	return 0;
  u8 frontR = activeRb[maxIndex - 1];	
  front.x  = bCastInfo[frontR - 1].rpos.locationX;
  front.y  = bCastInfo[frontR - 1].rpos.locationY;
  float dist = getDistance2(nowp.x, nowp.y, front.x, front.y);
  if(dist < (length)){
	flag++;
	halt(flag*2 + 3);
  }
  return flag;
}

int SleepUntilSafe2(int frontR, float length){
  u8 flag = 0;
  typeCoordinate nowp = GetCoordinate();
  typeCoordinate front;
#ifdef VISIT_DEMO	  
  /**************for visitor demo******************/	  
  if(activeRb[2] == rbID){
	if(bCastInfo[activeRb[1] - 1].isStop){
	  return flag;
	}
  }
  /**************for visitor demo******************/	  
#endif	
	if((frontR >= 1) && (frontR < ROBOTS))
	{
	  front.x  = bCastInfo[frontR - 1].rpos.locationX;
	  front.y  = bCastInfo[frontR - 1].rpos.locationY;
	  
	  float dist = getDistance2(nowp.x, nowp.y, front.x, front.y);
	  if(dist < (length + 0.04)){
		flag++;
		//halSetLedStatus(LED_BLUE, LED_ON); 
	  }
	}
  halt(flag*4);	
  return flag;
}

int reachLine(float x, float y, float speed){
  typeCoordinate nowPos = GetCoordinate();
  if(nowPos.x >= x)
	return 1;
  else 
	return 0;
}

u8 IsinActive(u8 id){
  return bCastInfo[id-1].isActive;
}

int otherIsinActive(u8 id){
  int i = 0;
  static int tick = 0;
  switch(id){
  case 1:
	for(i = 2; i<= ROBOTS; i++){
	  if(!bCastInfo[i-1].isActive){
		tick++;
	  }
	}
	break;
  case 2:
	for(i = 1; i<= ROBOTS; i++){
	  if(i == 2) continue;
	  if(!bCastInfo[i-1].isActive){
		tick++;
	  }
	}
	break;
  case 3:
	for(i = 1; i<= ROBOTS; i++){
	  if(i == 3) continue;
	  if(!bCastInfo[i-1].isActive){
		tick++;
	  }
	}
	break;
  case 4:
	for(i = 1; i<= ROBOTS; i++){
	  if(i == 4) continue;
	  if(!bCastInfo[i-1].isActive){
		tick++;
	  }
	}
  }
  return tick;
}

u8 TargetPosIsDead(float x, float y, u8 id){
	int i = 0;
	for(i = 1; i <= ROBOTS; i++){
	  if(i == id) continue;
	  float dist = getDistance2(x,y, bCastInfo[i].rpos.locationX , bCastInfo[i].rpos.locationY);
	  if(dist <= MIN_RANGE )
		return i;
	}
	return 0;
}

//void goStraight(float x, float y, float speed, int fRbID,int phase) {
//  int deta = 0;
//  int i = 0;
//  int cell = MAX_MOTORDETA;
//  float speedL = speed;
//  float speedR = speed;
//  
//  if(phase == 2){
//	cell = MIN_MOTORDETA;
//	deta = LONG_LINE;
//  }
//  
//  typeCoordinate start = GetCoordinate();
//  if(GoalInFront(x,y,phase) == true)
//	return;
//  
//  typeCoordinate nowp = GetCoordinate();
//  while (1) {	
//	float dist = getDistance2(nowp.x, nowp.y, x, y);	
//	if (dist < MIN_RANGE) break;
//	
//	while(TargetPosIsDead(x,y,rbID)){
//	  halt(1);
//	}
//	
//	while(((phase == 1) || (phase == 3)) && (dist <= 0.06)){
//	  rotateTo(x,y,ANGLESPEED,ROTATE_ACCURATE); 
//	  SetLeftWheelGivenSpeed(speed);
//	  SetRightWheelGivenSpeed(speed);
//	  vTaskDelay(300);
//	  if((GoalInFront(x,y,phase) == true))
//		return;
//	}
//	
//	if(phase == 2){
//	  while(SleepUntilSafe(fRbID,FRONT_SIGHT_LENGTH));
//	  if(((reachLine(x,y,speed)) || dist < MIN_RANGE))
//		return;
//	}
//	
//	if(phase == 1){
//	  static int obsID = 0;
//	  while((obsID = rotateToSafe(x,y,FRONT_SIGHT_LENGTH,CLOCKWISE))){
//		if(IsinActive(obsID) != 0){
//		  SetLeftWheelGivenSpeed(speed);
//		  SetRightWheelGivenSpeed(speed);
//		  vTaskDelay(500);
//		}else{
//		  typeCoordinate now = GetCoordinate();
//		  float dist = getDistance2(now.x, now.y,bCastInfo[obsID-1].rpos.locationX,bCastInfo[obsID-1].rpos.locationY); 
//		  if((bCastInfo[obsID-1].rpos.locationY > now.y) || (dist < 0.1f)){
//			rotateToNorthAngle(0, MAX_ROTATE_SPEED);
//		  }else if((bCastInfo[obsID-1].rpos.locationY < now.y) || (dist < 0.1)){
//			rotateToNorthAngle(180, MAX_ROTATE_SPEED);
//		  }
//		  SetLeftWheelGivenSpeed(speed);
//		  SetRightWheelGivenSpeed(speed);
//		  for(i = 0; i < GOBACK_TIMES; i++){
//			vTaskDelay(500);
//			SetLeftWheelGivenSpeed(speed);
//			SetRightWheelGivenSpeed(speed);
//			rotateToSafe(x,y,FRONT_SIGHT_LENGTH,CLOCKWISE);
//		  }
//		  break;
//		}
//	  }
//	}
//	int side = whichSide(x + deta, y);
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
//	while(rotateToSafe(x,y,FRONT_SIGHT_LENGTH,CLOCKWISE));
//	
//	SetLeftWheelGivenSpeed(speedL);
//	SetRightWheelGivenSpeed(speedR);
//	vTaskDelay(500);	
//	nowp = GetCoordinate();
//  }
//}

void gotoLeftDelta(){
  rotateToNorthAngle(190,FASTSPEED);
  SetLeftWheelGivenSpeed(30);
  SetRightWheelGivenSpeed(35);
  vTaskDelay(3000);
}

void gotoRightDelta(){
  rotateToNorthAngle(20,FASTSPEED);
  SetLeftWheelGivenSpeed(30);
  SetRightWheelGivenSpeed(35);
  vTaskDelay(3000);
}

void goForward(){
  rotateToNorthAngle(5,FASTSPEED);
  SetLeftWheelGivenSpeed(35);
  SetRightWheelGivenSpeed(35);
  vTaskDelay(10000);
}  //trick is not very good

void gotoRightStep2(){
  rotateToNorthAngle(300,FASTSPEED);
  SetLeftWheelGivenSpeed(30);
  SetRightWheelGivenSpeed(25);
  vTaskDelay(3000);
}