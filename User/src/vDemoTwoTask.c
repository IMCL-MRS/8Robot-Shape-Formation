#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include "colAvoidance.h"
#include <math.h>

#ifdef DEMO1

#ifdef CONFIG_ROBOT1
const int rbID = 1;
#elif defined(CONFIG_ROBOT2)
const int rbID = 2;
#elif defined(CONFIG_ROBOT3)
const int rbID = 3;
#elif defined(CONFIG_ROBOT4)
const int rbID = 4;
#endif

const typeCoordinate posA1 = {0.7860,0.4000};
const typeCoordinate posA2 = {0.6320,0.3980};
const typeCoordinate posA3 = {0.4900,0.3880};
const typeCoordinate posA4 = {0.3140,0.3987};

const typeCoordinate posM1  = {1.1003,0.4000};
const typeCoordinate posM2  = {1.1003,0.4000};
const typeCoordinate posM3  = {1.0903,0.3987};
const typeCoordinate posM4  = {1.0903,0.3987};

const typeCoordinate posC1 = {1.3450,0.2000};
const typeCoordinate posC3 = {1.3360,0.3373};
const typeCoordinate posC4 = {1.3150,0.5400};
const typeCoordinate posC2 = {1.3000,0.6313};

u8 imready = 0;

float GetPosX();
float GetPosY();


float getDistance2(float Ax, float Ay, float Bx, float By) {
  return sqrt( (Ax-Bx)*(Ax-Bx) + (Ay-By)*(Ay-By));
}

void broardCastInfo(u8 isReady){
  static volatile rbNode rbInfo;
  rbInfo.nodeID = rbID;
  //init the location
  rbInfo.rpos.locationX = GetPosX();
  rbInfo.rpos.locationY = GetPosY();
  //init the north angel
  static u8 txBuf[10] = {0}; //info package
  //send the package
  txBuf[0] = rbInfo.nodeID;	
  memcpy(txBuf+1,(u8 *)(&(rbInfo.rpos.locationX)),1);
  memcpy(txBuf+2,((u8 *)(&(rbInfo.rpos.locationX))+1),1);
  memcpy(txBuf+3,((u8 *)(&(rbInfo.rpos.locationX))+2),1);
  memcpy(txBuf+4,((u8 *)(&(rbInfo.rpos.locationX))+3),1);
  memcpy(txBuf+5,(u8 *)(&(rbInfo.rpos.locationY)),1);
  memcpy(txBuf+6,((u8 *)(&(rbInfo.rpos.locationY))+1),1);
  memcpy(txBuf+7,((u8 *)(&(rbInfo.rpos.locationY))+2),1);
  memcpy(txBuf+8,((u8 *)(&(rbInfo.rpos.locationY))+3),1);
  txBuf[9] = isReady;
  RFTxPacket(RF_BROADCAST_INFO, txBuf, 10);
}

void boardCastInfos(){
  int i = 0;
  for (i = 0; i < SENDTIMES; ++ i) {
    broardCastInfo(1);
  }
}


void robot1(){
  imready = 0;
  rotateTo(posA1.x,posA1.y,ANGLESPEED);
  ControlRobotgo2Position(posA1.x,posA1.y,LINESPEED);
  rotateTo(posM1.x,posM1.y,ANGLESPEED);
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
    boardCastInfos();
    halt(1);
    rotateTo(posM1.x,posM1.y,ANGLESPEED);
    ControlRobotgo2Position(posM1.x,posM1.y,LINESPEED);
    rotateTo(posC1.x,posC1.y,ANGLESPEED);
    ControlRobotgo2Position(posC1.x,posC1.y,LINESPEED);
    rotateToNorthAngle(-80, ANGLESPEED);
  }
}

void robot2(){
  imready = 0;
  rotateTo(posA2.x,posA2.y,ANGLESPEED);
  ControlRobotgo2Position(posA2.x,posA2.y,LINESPEED);
  rotateTo(posM2.x,posM2.y,ANGLESPEED);
#ifdef DEBUG
  imready = 1;
  halt(2);
#else
  boardCastInfos();
  while (1) {
    if (isReady(1)) {
      imready = 1;
      break;
    }
    halt(0.3);
  }
  halt(8);
#endif
  if (imready) {
    rotateTo(posM2.x,posM2.y,ANGLESPEED);
    ControlRobotgo2Position(posM2.x,posM2.y,LINESPEED);
    rotateTo(posC2.x,posC2.y,ANGLESPEED);
    ControlRobotgo2Position(posC2.x,posC2.y,LINESPEED);
    rotateToNorthAngle(-90, ANGLESPEED);
  }
}

void robot3(){
  imready = 0;
  rotateTo(posA3.x,posA3.y,ANGLESPEED);
  ControlRobotgo2Position(posA3.x,posA3.y,LINESPEED);
  rotateTo(posM3.x,posM3.y,ANGLESPEED);
#ifdef DEBUG
  imready = 1;
  halt(2);
#else
  boardCastInfos();
  while (1) {
    if (isReady(1)) {
      imready = 1;
      break;
    }
    halt(0.3);
  }
  halt(15);
#endif
  
  if (imready) {
    rotateTo(posM3.x,posM3.y,ANGLESPEED);
    ControlRobotgo2Position(posM3.x,posM3.y,LINESPEED);
    rotateTo(posC3.x,posC3.y,ANGLESPEED);
    ControlRobotgo2Position(posC3.x,posC3.y,LINESPEED);
    rotateToNorthAngle(-90, ANGLESPEED);
  }
}

void robot4(){
  imready = 0;
  rotateTo(posA4.x,posA4.y,ANGLESPEED);
  ControlRobotgo2Position(posA4.x,posA4.y,LINESPEED);
  rotateToNorthAngle(103,ANGLESPEED);
#ifdef DEBUG
  imready = 1;
  halt(2);
#else
  boardCastInfos();
  while (1) {
    if (isReady(1)) {
      imready = 1;
      break;
    }
    halt(0.3);
  }
  halt(22);
#endif
  if (imready) {
    ControlRobotgo2Position(posM4.x,posM4.y,LINESPEED);
    rotateTo(posC4.x,posC4.y,ANGLESPEED);
    ControlRobotgo2Position(posC4.x,posC4.y,LINESPEED);
    rotateToNorthAngle(-80, ANGLESPEED);
  }
}

void vDemoOneTask( void *pvParameters ){
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