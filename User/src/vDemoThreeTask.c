#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include "colAvoidance.h"
#include <math.h>

#ifdef DEMO3
//const typeCoordinate posA1 = {0.7860,0.4000};
//const typeCoordinate posA2 = {0.6320,0.3880};
//const typeCoordinate posA3 = {0.4900,0.3880};
//const typeCoordinate posA4 = {0.3240,0.3987};
//
//const typeCoordinate posM  = {2.1003,0.4000};
//
//const typeCoordinate posM1  = {1.1003,0.4000};
//const typeCoordinate posM2  = {1.1003,0.4000};
//const typeCoordinate posM3  = {1.1003,0.4000};
//const typeCoordinate posM4  = {1.1003,0.4000};
//
//const typeCoordinate posC1 = {1.3300,0.5966};
//const typeCoordinate posC4 = {1.3300,0.4648};
//const typeCoordinate posC3 = {1.3300,0.3554};
//const typeCoordinate posC2 = {1.3300,0.2270};

const typeCoordinate posA1 = {0.9300,0.4180};
const typeCoordinate posA2 = {0.7250,0.4180};
const typeCoordinate posA3 = {0.5300,0.4180};
const typeCoordinate posA4 = {0.3400,0.4080};

const typeCoordinate posM  = {2.3412,0.4180};

const typeCoordinate posM1 = {1.3378,0.4333};
const typeCoordinate posM2 = {1.3136,0.4318};
const typeCoordinate posM3 = {1.3240,0.4200};
const typeCoordinate posM4 = {1.3327,0.4083};

const typeCoordinate posC1 = {1.4562,0.7400};
const typeCoordinate posC4 = {1.4562,0.5600};
const typeCoordinate posC3 = {1.4562,0.3600};
const typeCoordinate posC2 = {1.4562,0.2000};

u8 imready = 0;

/*robot1 formation control*/
void robot1(){
  imready = 0;
  rotateTo(posA1.x,posA1.y,ANGLESPEED,1);
  ControlRobotgo2Position(posA1.x,posA1.y,FASTSPEED,1);
  rotateTo(posM1.x,posM1.y,ANGLESPEED,1);
#ifdef DEBUG
  imready = 1;
#else
  while (1) {
	if (isReady(2) && isReady(3) && isReady(4)) {
	  imready = 1;
	  break;
	}
	halt(0.3);
  }
#endif
  if (imready) {
	imready = 1;
	halt(1);
	rotateTo(posM1.x,posM1.y,ANGLESPEED,1);
    runStraight(posM1.x,posM1.y,LINESPEED);
	//ControlRobotgo2Position(posM1.x,posM1.y,LINESPEED,-1);
	rotateTo(posC1.x,posC1.y,ANGLESPEED,1);
	ControlRobotgo2Position(posC1.x,posC1.y,FASTSPEED,-1);
	rotateToNorthAngle(-90, ANGLESPEED);
  }
  return;
}

void robot2(){
  imready = 0;
  rotateTo(posA2.x,posA2.y,ANGLESPEED,1);
  ControlRobotgo2Position(posA2.x,posA2.y,FASTSPEED,1);
  rotateTo(posM2.x,posM2.y,ANGLESPEED,1);
#ifdef DEBUG
  imready = 1;
  halt(2);
#else
  imready = 1;
  while (1) {
	if (isReady(1)) {
	  imready = 1;
	  break;
	}
	halt(0.3);
  }
  halt(6);
#endif
  if (imready) {
	rotateTo(posM2.x,posM2.y,ANGLESPEED,1);
	runStraight(posM2.x,posM2.y,LINESPEED);
//	ControlRobotgo2Position(posM2.x,posM2.y,LINESPEED,-1);
	rotateTo(posC2.x,posC2.y,ANGLESPEED,1);
	ControlRobotgo2Position(posC2.x,posC2.y,FASTSPEED,1);
	rotateToNorthAngle(-90, ANGLESPEED);
  }
}

void robot3(){
  imready = 0;
  rotateTo(posA3.x,posA3.y,ANGLESPEED,1);
  ControlRobotgo2Position(posA3.x,posA3.y,FASTSPEED,1);
  rotateTo(posM3.x,posM3.y,ANGLESPEED,1);
#ifdef DEBUG
  imready = 1;
  halt(2);
#else
  imready = 1;
  while (1) {
	if (isReady(1)) {
	  imready = 1;
	  break;
	}
	halt(0.3);
  }
  halt(11);
#endif
  if (imready) {
	rotateTo(posM3.x,posM3.y,ANGLESPEED,1);
	runStraight(posM3.x,posM3.y,LINESPEED);
//	ControlRobotgo2Position(posM3.x,posM3.y,LINESPEED,-1);
	rotateTo(posC3.x,posC3.y,ANGLESPEED,1);
	
	rotateTo(posC3.x,posC3.y,ANGLESPEED,1);
	ControlRobotgo2Position(posC3.x,posC3.y,FASTSPEED,1);
	rotateToNorthAngle(-90, ANGLESPEED);
  }
}

void robot4(){
  imready = 0;
  rotateTo(posA4.x,posA4.y,ANGLESPEED,1);
  ControlRobotgo2Position(posA4.x,posA4.y,FASTSPEED,1);
  rotateTo(posM4.x,posM4.y,ANGLESPEED,1);
#ifdef DEBUG
  imready = 1;
  halt(2);
#else
  imready = 1;
  while (1) {
	if (isReady(1)) {
	  imready = 1;
	  break;
	}
	halt(0.3);
  }
  halt(16);
#endif
  if (imready) {
	rotateTo(posM4.x,posM4.y,ANGLESPEED,1);
	runStraight(posM4.x,posM4.y,LINESPEED);
//	ControlRobotgo2Position(posM4.x,posM4.y,LINESPEED,-1);
	rotateTo(posC4.x,posC4.y,ANGLESPEED,1);
	
	rotateTo(posC4.x,posC4.y,ANGLESPEED,1);
	ControlRobotgo2Position(posC4.x,posC4.y,FASTSPEED,-1);
	rotateToNorthAngle(-90, ANGLESPEED);
  }
}

void vDemoThreeTask( void *pvParameters ){
  halt(3);
  while (1) {
#ifdef CONFIG_ROBOT1
	robot1();
#elif defined(CONFIG_ROBOT2)
	robot2();
#elif defined(CONFIG_ROBOT3)
	robot3();
#elif defined(CONFIG_ROBOT4)
	robot4();
#endif
	halt(1000);
  }
}
#endif
