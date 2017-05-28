
#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include "colAvoidance.h"
#include <math.h>

#ifdef DEMO4
//const typeCoordinate posA1 = {0.8200,0.3800};
//const typeCoordinate posA2 = {0.6700,0.3800};
//const typeCoordinate posA3 = {0.5200,0.3800};
//const typeCoordinate posA4 = {0.3700,0.3800};

const typeCoordinate posM1  = {1.1503,0.3800};
const typeCoordinate posM2  = {1.1503,0.3800};
const typeCoordinate posM3  = {1.1503,0.3800};
const typeCoordinate posM4  = {1.1503,0.3800};

const typeCoordinate posC1 = {1.3300,0.6000};
const typeCoordinate posC4 = {1.3300,0.4500};
const typeCoordinate posC3 = {1.3300,0.3000};
const typeCoordinate posC2 = {1.3300,0.1500};

//const typeCoordinate posA1 = {0.9300,0.4000};
//const typeCoordinate posA2 = {0.7250,0.4000};
const typeCoordinate posA3 = {0.5300,0.4000};
//const typeCoordinate posA4 = {0.3700,0.4000};
//
//const typeCoordinate posM  = {2.3412,0.4000};
//
//const typeCoordinate posM1 = {1.3378,0.4000};
//const typeCoordinate posM2 = {1.3300,0.4000};
//const typeCoordinate posM3 = {1.4562,0.4000};
//const typeCoordinate posM4 = {1.2300,0.4100};
//
//const typeCoordinate posC1 = {1.4562,0.6700};
//const typeCoordinate posC4 = {1.4562,0.4067};
//const typeCoordinate posC3 = {1.4562,0.2966};
//const typeCoordinate posC2 = {1.4562,0.1100};

const typeCoordinate posA1 = {0.3,0.4};
const typeCoordinate posA2 = {0.4,0.4};
const typeCoordinate posA4 = {0.5,0.4};

u8 imready = 0;
u8 isActive = 1;

void SetInitState(){
  imready = 0;
  isActive = 1;
}

void behSetReady(){
  imready = 1;
}

void behSetinActive(){
  isActive = 0;
}

void behSetActive(){
  isActive = 1;
}
/*robot1 formation control*/
void robot1(){
  SetInitState();
  rotateFastTo(posA1.x,posA1.y,ANGLESPEED,ROTATE_ACCURATE);
  goStraight(posA1.x,posA1.y,FASTSPEED,FRONT_RBID,PHASE_ONE);
  rotateFastTo(posM1.x,posM1.y,ANGLESPEED,ROTATE_ACCURATE);
  behSetinActive();
#ifdef DEBUG
  imready = 1;
#else
  while (1) {
	if (isReady(2) && /*isReady(3) &&*/ isReady(4)) {
	  imready = 1;
	  break;
	}
	halt(0.3);
  }
#endif
  if (imready) {
	imready = 1;
	halt(1);
	rotateFastTo(posM1.x,posM1.y,ANGLESPEED,ROTATE_ACCURATE);
    goStraight(posM1.x,posM1.y,LINESPEED,FRONT_RBID,PHASE_TWO);
	rotateFastTo(posC1.x,posC1.y,ANGLESPEED,ROTATE_ACCURATE);
	goStraight(posC1.x,posC1.y,FASTSPEED,FRONT_RBID,PHASE_THREE);
	rotateToNorthAngle(-90, MAX_ROTATE_SPEED);
  }
  return;
}

void robot2(){
  SetInitState();
  rotateFastTo(posA2.x,posA2.y,ANGLESPEED,ROTATE_ACCURATE);
  goStraight(posA2.x,posA2.y,FASTSPEED,FRONT_RBID,PHASE_ONE);
  rotateFastTo(posM2.x,posM2.y,ANGLESPEED,ROTATE_ACCURATE);
  behSetinActive();
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
	rotateFastTo(posM2.x,posM2.y,ANGLESPEED,ROTATE_ACCURATE);
	goStraight(posM2.x,posM2.y,LINESPEED,FRONT_RBID,PHASE_TWO); //flag, phase
	rotateFastTo(posC2.x,posC2.y,ANGLESPEED,ROTATE_ACCURATE);
	goStraight(posC2.x,posC2.y,FASTSPEED,FRONT_RBID,PHASE_THREE);
	rotateToNorthAngle(-90, MAX_ROTATE_SPEED);
  }
}

void robot3(){
  SetInitState();
  rotateFastTo(posA3.x,posA3.y,ANGLESPEED,ROTATE_ACCURATE);
  goStraight(posA3.x,posA3.y,FASTSPEED,FRONT_RBID,PHASE_ONE);
  rotateFastTo(posM3.x,posM3.y,ANGLESPEED,ROTATE_ACCURATE);
  behSetinActive();
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
	rotateFastTo(posM3.x,posM3.y,ANGLESPEED,ROTATE_ACCURATE);
	goStraight(posM3.x,posM3.y,LINESPEED,FRONT_RBID,PHASE_TWO);
	rotateFastTo(posC3.x,posC3.y,ANGLESPEED,ROTATE_ACCURATE);
	goStraight(posC3.x,posC3.y,FASTSPEED,FRONT_RBID,PHASE_THREE);
	rotateToNorthAngle(-90, MAX_ROTATE_SPEED);
  }
}

void robot4(){
  SetInitState();
  rotateFastTo(posA4.x,posA4.y,ANGLESPEED,ROTATE_ACCURATE);
  goStraight(posA4.x,posA4.y,FASTSPEED,FRONT_RBID,PHASE_ONE);
  rotateFastTo(posM4.x,posM4.y,ANGLESPEED,ROTATE_ACCURATE);
  behSetinActive();
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
	rotateFastTo(posM4.x,posM4.y,ANGLESPEED,ROTATE_ACCURATE);
	goStraight(posM4.x,posM4.y,LINESPEED,FRONT_RBID,PHASE_TWO);
	rotateFastTo(posC4.x,posC4.y,ANGLESPEED,ROTATE_ACCURATE);
	goStraight(posC4.x,posC4.y,FASTSPEED,FRONT_RBID,PHASE_THREE);
	rotateToNorthAngle(-90, MAX_ROTATE_SPEED);
  }
}

void vDemoFourTask( void *pvParameters ){
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

