#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include "fc.h"
#include "snake.h"
#include <math.h>

extern volatile u8 imReady;
extern u8 activeRb[ROBOTS];

const typeCoordinate posS1 = {1.7,0.25};

const typeCoordinate posA8 = {1.6, 0.30};
const typeCoordinate posA7 = {1.4, 0.30};
const typeCoordinate posA6 = {1.2, 0.30};
const typeCoordinate posA5 = {1.0, 0.30};
const typeCoordinate posA4 = {0.80,0.30};
const typeCoordinate posA3 = {0.60,0.30};
const typeCoordinate posA2 = {0.40,0.30};
const typeCoordinate posA1 = {0.18,0.30};

const typeCoordinate posR1 = {0.62,0.085};
const typeCoordinate posR2 = {0.62,0.314};
const typeCoordinate posR3 = {0.63,0.538};
const typeCoordinate posR4 = {0.90,0.509};
const typeCoordinate posR5 = {0.90,0.092};
const typeCoordinate posR6 = {1.16,0.092};
const typeCoordinate posR7 = {1.16,0.289};
const typeCoordinate posR8 = {1.16,0.518};

const typeCoordinate posR1_2 = {0.90,0.092};
const typeCoordinate posR2_2 = {0.62,0.085};
const typeCoordinate posR3_2 = {0.62,0.314};
const typeCoordinate posR4_2 = {0.63,0.538};
const typeCoordinate posR5_2 = {1.16,0.092};
const typeCoordinate posR6_2 = {1.16,0.289};
const typeCoordinate posR7_2 = {1.16,0.518};
const typeCoordinate posR8_2 = {0.90,0.509};


const typeCoordinate posT1 = {0.56,0.050};
const typeCoordinate posT2 = {0.71,0.283};
const typeCoordinate posT3 = {0.81,0.45};
const typeCoordinate posT4 = {0.96,0.669};
const typeCoordinate posT5 = {0.93,0.050};
const typeCoordinate posT6 = {1.23,0.050};
const typeCoordinate posT7 = {1.14,0.258};
const typeCoordinate posT8 = {1.05,0.45};

const typeCoordinate posT1_2 = {0.93,0.050};
const typeCoordinate posT2_2 = {0.56,0.050};
const typeCoordinate posT3_2 = {0.71,0.283};
const typeCoordinate posT4_2 = {0.81,0.45};
const typeCoordinate posT5_2 = {1.23,0.050};
const typeCoordinate posT6_2 = {1.14,0.258};
const typeCoordinate posT7_2 = {1.05,0.45};
const typeCoordinate posT8_2 = {0.96,0.669};

const typeCoordinate posH1 = {0.65,0.165};
const typeCoordinate posH2 = {0.58,0.398};
const typeCoordinate posH3 = {0.65,0.593};
const typeCoordinate posH4 = {0.93,0.686};
const typeCoordinate posH5 = {0.93,0.069};
const typeCoordinate posH6 = {1.13,0.15};
const typeCoordinate posH7 = {1.2,0.318};
const typeCoordinate posH8 = {1.13,0.543};

const typeCoordinate posD1 = {0.7,0.1};
const typeCoordinate posD2 = {1.2,0.1};
const typeCoordinate posD3 = {1.2,0.7};
const typeCoordinate posD4 = {0.7,0.7};

//#define DEBUG
void bugTest(){
  gotoLeftDelta();
  rotateFastTo(posA1.x,posA1.y,ANGLESPEED,ROTATE_ACCURATE);
  ControlRobotgo2Position(posA1.x,posA1.y,FASTSPEED,0);
  halt(1000);
}

void point2Point(float x, float y){
  rotateFastTo(x,y,ANGLESPEED,ROTATE_ACCURATE);
  ControlRobotgo2Position(x,y,FASTSPEED,0);
}

u8 statusReady(u8 status){
  if ((stReady(activeRb[1],status)) && (stReady(activeRb[2],status))
      &&(stReady(activeRb[3],status))&&(stReady(activeRb[4],status))
        &&(stReady(activeRb[5],status)) &&(stReady(activeRb[6],status)) 
          &&(stReady(activeRb[7],status))) {  
            return 1;
          }else{
            return 0;
          }
}

/*robot1 formation control*/
void robot1Step1(){
  imReady = 0;
  vTaskDelay(500);
  //point2Point(posA1.x,posA1.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot1Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    if (statusReady(1)) {
	  imReady = 1;	  
	  break;
	}
    vTaskDelay(500);
  }
#endif
  while(!stReady(activeRb[0],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[0],1)) {
    beepSing();
    halt(1);
    gotoRightStep2();//hksp
    point2Point(posR1.x,posR1.y);
  }
  halt(2);
  return;
}

void robot1Step3(){  
  //3 formation	  
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else	
  for(;;) {
    if (statusReady(2)) {
	  imReady = 2;
	  break;
	}
    vTaskDelay(500);
  }
#endif
  while(!stReady(activeRb[0],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  
  if (stReady(activeRb[0],2)) {
    beepSing();
    halt(1);
    point2Point(posT1.x,posT1.y);
  }  
  halt(2);
  return;
}

void robot1Step4(u8 flag){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    if (statusReady(3)) {
	  imReady = 3;
	  break;
	}
    vTaskDelay(500);
  }
#endif
  while(!stReady(activeRb[0],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  
  if (stReady(activeRb[0],3)) {
    beepSing();
    halt(1);
    if (flag) {
      rotateToNorthAngle(270,FASTSPEED);
      SetLeftWheelGivenSpeed(30);
      SetRightWheelGivenSpeed(30);
      vTaskDelay(3000);
    }
    point2Point(posH1.x,posH1.y);
  }   
  halt(2);
  return;
}

void robot1Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    if (statusReady(4)) {
	  imReady = 4;
	  break;
	}
    vTaskDelay(500);
  }
#endif
  while(!stReady(activeRb[0],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  
  if (stReady(activeRb[0],4)) {
    beepSing();
    halt(1);
    point2Point(posT1_2.x,posT1_2.y);
  }
  return;
}

void robot1Step6(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    if (statusReady(sig)) {
	  imReady = sig;
	  break;
	}
    vTaskDelay(500);
  }
#endif
  while(!stReady(activeRb[0],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  
  if (stReady(activeRb[0],sig)) {
    beepSing();
    halt(1);
    point2Point(posR1_2.x,posR1_2.y);
  }
  return;
}

void robot1Step7(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    if (statusReady(sig)) {
	  imReady = sig;
	  break;
	}
    vTaskDelay(500);
  }
#endif
  while(!stReady(activeRb[0],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  
  if (stReady(activeRb[0],sig)) {
    beepSing();
    //sundy
    //	halt(1);
    //gotoLeftDelta();
    point2Point(posA4.x,posA4.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
  return;
}

void robot1Step8(){  
  //5 formation	
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    if (statusReady(7)) {
	  imReady = 7;
	  break;
	}
    vTaskDelay(500);
  }
#endif
  while(!stReady(activeRb[0],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  
  if (stReady(activeRb[0],7)) {
    beepSing();
    //sundy
    //	halt(1);
    //gotoLeftDelta();
      rotateToNorthAngle(185,FASTSPEED);
      SetLeftWheelGivenSpeed(30);
      SetRightWheelGivenSpeed(30);
      vTaskDelay(3000);
    point2Point(posA1.x,posA1.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
  return;
}

/*robot2 formation control*/
void robot2Step1(){
  imReady = 0;
  vTaskDelay(500);
  //  point2Point(posA2.x,posA2.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot2Step2(){  
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    imReady = 1;	
    if (stReady(activeRb[0],1)) {
      imReady = 1;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[1],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[1],1)) {
    halt(2); //hksp
    point2Point(posR2.x,posR2.y);
  }
//  imReady = 2;
//  halt(2);
  return;
}

void robot2Step3(){  
  //3 formation
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  for(;;) {
    imReady = 2;
    if (stReady(activeRb[0],2)) {
      imReady = 2;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[1],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  
  if (stReady(activeRb[1],2)) {
    point2Point(posT2.x,posT2.y);
  }

//  imReady = 3;
//  halt(2);
  return;
}

void robot2Step4(){  
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    imReady = 3;
    if (stReady(activeRb[0],3)) {
      imReady = 3;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[1],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  if (stReady(activeRb[1],3)) {  
    point2Point(posH2.x,posH2.y);
  }

//  imReady = 4;
//  halt(2);
  return;
}

void robot2Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    imReady = 4;
    if (stReady(activeRb[0],4)) {
      imReady = 4;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[1],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  if (stReady(activeRb[1],4)) {
    halt(1);
    point2Point(posT2_2.x,posT2_2.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 5;
//  halt(2);
  return;
}

void robot2Step6(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[1],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[1],sig)) {
    halt(1);
    point2Point(posR2_2.x,posR2_2.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 6;
//  halt(2);
  return;
}

void robot2Step7(u8 sig){
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[1],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[1],sig)) {
    halt(1);
    point2Point(posA2.x,posA2.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 7;
//  halt(2);
  return;
}

void robot2Step8(){  
  //5 formation	
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    imReady = 7;
    if (stReady(activeRb[0],7)) {
      imReady = 7;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[1],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  if (stReady(activeRb[1],7)) {
    halt(1);
    point2Point(posA2.x,posA2.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 0;
//  halt(2);
  return;
}

/*robot3 formation control*/
void robot3Step1(){
  imReady = 0;
  vTaskDelay(500);
  //  point2Point(posA3.x,posA3.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot3Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    imReady = 1;
    if (stReady(activeRb[0],1)) {
      imReady = 1;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[2],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[2],1)) {
    //goForward();
    point2Point(posR3.x,posR3.y);
  }
  
//  imReady = 2;
//  halt(2);
  return;
}

void robot3Step3(){
  //3 formation
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  for(;;) {
    imReady = 2;
    if (stReady(activeRb[0],2)) {
      imReady = 2;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[2],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  
  if (stReady(activeRb[2],2)) {
    point2Point(posT3.x,posT3.y);
  }
  
//  imReady = 3;
//  halt(2);
  return;
}

void robot3Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    imReady = 3;
    if (stReady(activeRb[0],3)) {
      imReady = 3;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[2],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  
  if (stReady(activeRb[2],3)) {
    point2Point(posH3.x,posH3.y);
  }
  
//  imReady = 4;
//  halt(2);
  return;
}

void robot3Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    imReady = 4;
    if (stReady(activeRb[0],4)) {
      imReady = 4;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[2],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  if (stReady(activeRb[2],4)) {
    halt(1);
    point2Point(posT3_2.x,posT3_2.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 5;
//  halt(2);
  return;
}

void robot3Step6(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[2],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[2],sig)) {
    halt(1);
    point2Point(posR3_2.x,posR3_2.y);
  }
//  imReady = 6;
//  halt(2);
  return;
}

void robot3Step7(u8 sig){
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[2],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[2],sig)) {
    halt(1);
    //gotoRightDelta();
    point2Point(posA3.x,posA3.y);
  }
//  imReady = 7;
//  halt(2);
  return;
}

void robot3Step8(){  
  //5 formation	
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    imReady = 7;
    if (stReady(activeRb[0],7)) {
      imReady = 7;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[2],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  if (stReady(activeRb[2],7)) {
    halt(1);
    //gotoRightDelta();
    point2Point(posA3.x,posA3.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 0;
//  halt(2);
  return;
}

/*robot4 formation control*/
void robot4Step1(){
  imReady = 0;
  //  point2Point(posA4.x,posA4.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot4Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    imReady = 1;
    if (stReady(activeRb[0],1)) {
      imReady = 1;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[3],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[3],1)) {
    point2Point(posR4.x,posR4.y);
  }
  
//  imReady = 2;
//  halt(2);
  return;
}

void robot4Step3(){
  //3 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  for(;;) {
    imReady = 2;
    if (stReady(activeRb[0],2)) {
      imReady = 2;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[3],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  if (stReady(activeRb[3],2)) {
    point2Point(posT4.x,posT4.y);
  }
  
//  imReady = 3;
//  halt(2);
  return;
}

void robot4Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    imReady = 3;
    if (stReady(activeRb[0],3)) {
      imReady = 3;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[3],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  if (stReady(activeRb[3],3)) {
    point2Point(posH4.x,posH4.y);
  }
  
//  imReady = 4;
//  halt(2);
  return;
}

void robot4Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    imReady = 4;
    if (stReady(activeRb[0],4)) {
      imReady = 4;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[3],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  if (stReady(activeRb[3],4)) {
    halt(1);
    point2Point(posT4_2.x,posT4_2.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 5;
//  halt(2);
  return;
}

void robot4Step6(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[3],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[3],sig)) {
    halt(1);
    point2Point(posR4_2.x,posR4_2.y);
  }
//  imReady = 6;
//  halt(2);
  return;
}

void robot4Step7(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[3],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[3],sig)) {
    halt(1);
    //sundy
    //gotoLeftDelta();
    point2Point(posA1.x,posA1.y);
  }
//  imReady = 7;
//  halt(2);
  return;
}

void robot4Step8(){  
  //5 formation	
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    imReady = 7;
    if (stReady(activeRb[0],7)) {
      imReady = 7;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[3],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  if (stReady(activeRb[3],7)) {
    halt(1);
    //sundy
    //gotoLeftDelta();
    point2Point(posA4.x,posA4.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 0;
//  halt(2);
  return;
}

/*robot5 formation control*/
void robot5Step1(){
  imReady = 0;
  //  point2Point(posA5.x,posA5.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot5Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    imReady = 1;
    if (stReady(activeRb[0],1)) {
      imReady = 1;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[4],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[4],1)) {
    point2Point(posR5.x,posR5.y);
  }
  
//  imReady = 2;
//  halt(2);
  return;
}

void robot5Step3(){
  //3 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  for(;;) {
    imReady = 2;
    if (stReady(activeRb[0],2)) {
      imReady = 2;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[4],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  if (stReady(activeRb[4],2)) {
    point2Point(posT5.x,posT5.y);
  }
  
//  imReady = 3;
//  halt(2);
  return;
}

void robot5Step4(u8 flag){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    imReady = 3;
    if (stReady(activeRb[0],3)) {
      imReady = 3;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[4],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  if (stReady(activeRb[4],3)) {
    if (flag){
        rotateToNorthAngle(300,FASTSPEED);
        SetLeftWheelGivenSpeed(30);
        SetRightWheelGivenSpeed(30);
        vTaskDelay(3000);
    }
    point2Point(posH5.x,posH5.y);
  }
  
//  imReady = 4;
//  halt(2);
  return;
}

void robot5Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    imReady = 4;
    if (stReady(activeRb[0],4)) {
      imReady = 4;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[4],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  if (stReady(activeRb[4],4)) {
    halt(1);
    point2Point(posT5_2.x,posT5_2.y);
  }
//  imReady = 5;
//  halt(2);
  return;
}

void robot5Step6(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[4],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[4],sig)) {
    halt(1);
    point2Point(posR5_2.x,posR5_2.y);
  }
//  imReady = 6;
//  halt(2);
  return;
}

void robot5Step7(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[4],sig)){
    vTaskDelay(500);
    imReady = 6;
  }
  if (stReady(activeRb[4],sig)) {
    halt(1);
    point2Point(posA8.x,posA8.y);
  }
//  imReady = 7;
//  halt(2);
  return;
}

void robot5Step8(){  
  //5 formation	
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    imReady = 7;
    if (stReady(activeRb[0],7)) {
      imReady = 7;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[4],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  if (stReady(activeRb[4],7)) {
    halt(1);
    point2Point(posA5.x,posA5.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 0;
//  halt(2);
  return;
}

/*robot6 formation control*/
void robot6Step1(){
  imReady = 0;
  //  point2Point(posA6.x,posA6.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot6Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    imReady = 1;
    if (stReady(activeRb[0],1)) {
      imReady = 1;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[5],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[5],1)) {
    point2Point(posR6.x,posR6.y);
  }
  
//  imReady = 2;
//  halt(2);
  return;
}

void robot6Step3(){
  //3 formation	
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  for(;;) {
    imReady = 2;
    if (stReady(activeRb[0],2)) {
      imReady = 2;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[5],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  if (stReady(activeRb[5],2)) {
    point2Point(posT6.x,posT6.y);
  }
  
//  imReady = 3;
//  halt(2);
  return;
}

void robot6Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    imReady = 3;
    if (stReady(activeRb[0],3)) {
      imReady = 3;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[5],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  if (stReady(activeRb[5],3)) {
    point2Point(posH6.x,posH6.y);
  }
  
//  imReady = 4;
//  halt(2);
  return;
}

void robot6Step5(){  
  //5 formation	
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    imReady = 4;
    if (stReady(activeRb[0],4)) {
      imReady = 4;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[5],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  if (stReady(activeRb[5],4)) {
    halt(1);
    point2Point(posT6_2.x,posT6_2.y);
  }
  
//  imReady = 5;
//  halt(2);
  return;
}

void robot6Step6(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[5],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[5],sig)) {
    halt(1);
    point2Point(posR6_2.x,posR6_2.y);
  }
  
//  imReady = 6;
//  halt(2);
  return;
}

void robot6Step7(u8 sig){  
  //5 formation	
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[5],sig)){
    vTaskDelay(500);
    imReady = 6;
  }
  if (stReady(activeRb[5],sig)) {
    //gotoRightDelta();
    //gotoRightDelta();
    point2Point(posA6.x,posA6.y);
  }
//  imReady = 7;
//  halt(2);
  return;
}

void robot6Step8(){  
  //5 formation	
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    imReady = 7;
    if (stReady(activeRb[0],7)) {
      imReady = 7;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[5],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  if (stReady(activeRb[5],7)) {
    //gotoRightDelta();
    //gotoRightDelta();
    point2Point(posA6.x,posA6.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 0;
//  halt(2);
  return;
}


/*robot7 formation control*/
void robot7Step1(){
  imReady = 0;
  //  point2Point(posA7.x,posA7.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot7Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    imReady = 1;
    if (stReady(activeRb[0],1)) {
      imReady = 1;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[6],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[6],1)) {
    point2Point(posR7.x,posR7.y);
  }
  
//  imReady = 2;
//  halt(2);
  return;
}

void robot7Step3(){
  //3 formation 
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  for(;;) {
    imReady = 2;
    if (stReady(activeRb[0],2)) {
      imReady = 2;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[6],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  if (stReady(activeRb[6],2)) {
    point2Point(posT7.x,posT7.y);
  }
  
//  imReady = 3;
//  halt(2);
  return;
}

void robot7Step4(){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    imReady = 3;
    if (stReady(activeRb[0],3)) {
      imReady = 3;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[6],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  if (stReady(activeRb[6],3)) {
    point2Point(posH7.x,posH7.y);
  }
  
//  imReady = 4;
//  halt(2);
  return;
}

void robot7Step5(){  
  //5 formation 
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    imReady = 4;
    if (stReady(activeRb[0],4)) {
      imReady = 4;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[6],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  if (stReady(activeRb[6],4)) {
    halt(1);
    point2Point(posT7_2.x,posT7_2.y);
  }
  
//  imReady = 5;
//  halt(2);
  return;
}

void robot7Step6(u8 sig){  
  //5 formation 
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[6],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[6],sig)) {
    halt(1);
    point2Point(posR7_2.x,posR7_2.y);
  }
//  imReady = 6;
//  halt(2);
  return;
}

void robot7Step7(u8 sig){  
  //5 formation 
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[6],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[6],sig)) {
    //gotoRightDelta();
    //gotoRightDelta();
    rotateToNorthAngle(5,FASTSPEED);
    SetLeftWheelGivenSpeed(30);
    SetRightWheelGivenSpeed(30);
    vTaskDelay(3000);
    point2Point(posA7.x,posA7.y);
  }
//  imReady = 7;
//  halt(2);
  return;
}

void robot7Step8(){  
  //5 formation 
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    imReady = 7;
    if (stReady(activeRb[0],7)) {
      imReady = 7;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[6],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  if (stReady(activeRb[6],7)) {
    //gotoRightDelta();
    //gotoRightDelta();
    point2Point(posA7.x,posA7.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 0;
//  halt(2);
  return;
}


/*robot8 formation control*/
void robot8Step1(){
  imReady = 0;
  //  point2Point(posA8.x,posA8.y);
  rotateToNorthAngle(0,FASTSPEED);
  halt(2);
  return;
}

void robot8Step2(){
  //2 formation
#ifdef DEBUG
  imReady = 1;
  halt(2);
#else
  for(;;) {
    imReady = 1;
    if (stReady(activeRb[0],1)) {
      imReady = 1;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[7],1)){
    vTaskDelay(500);
    imReady = 1;
  }
  
  if (stReady(activeRb[7],1)) {
    point2Point(posR8.x,posR8.y);
  }
  
//  imReady = 2;
//  halt(2);
  return;
}

void robot8Step3(){
  //3 formation 
#ifdef DEBUG
  imReady = 2;
  halt(2);
#else
  for(;;) {
    imReady = 2;
    if (stReady(activeRb[0],2)) {
      imReady = 2;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[7],2)){
    vTaskDelay(500);
    imReady = 2;
  }
  if (stReady(activeRb[7],2)) {
    point2Point(posT8.x,posT8.y);
  }
  
//  imReady = 3;
//  halt(2);
  return;
}

void robot8Step4(u8 flag){
  //4 formation
#ifdef DEBUG
  imReady = 3;
  halt(2);
#else
  for(;;) {
    imReady = 3;
    if (stReady(activeRb[0],3)) {
      imReady = 3;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[7],3)){
    vTaskDelay(500);
    imReady = 3;
  }
  if (stReady(activeRb[7],3)) {
    if (flag) {
        rotateToNorthAngle(5,FASTSPEED);
        SetLeftWheelGivenSpeed(30);
        SetRightWheelGivenSpeed(30);
        vTaskDelay(3000);
    }
    point2Point(posH8.x,posH8.y);
  }
  
//  imReady = 4;
//  halt(2);
  return;
}

void robot8Step5(){  
  //5 formation 
#ifdef DEBUG
  imReady = 4;
  halt(2);
#else
  for(;;) {
    imReady = 4;
    if (stReady(activeRb[0],4)) {
      imReady = 4;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[7],4)){
    vTaskDelay(500);
    imReady = 4;
  }
  if (stReady(activeRb[7],4)) {
    halt(1);
    point2Point(posT8_2.x,posT8_2.y);
  }
  
//  imReady = 5;
//  halt(2);
  return;
}

void robot8Step6(u8 sig){  
  //5 formation 
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[7],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[7],sig)) {
    halt(1);
    point2Point(posR8_2.x,posR8_2.y);
  }
//  imReady = 6;
//  halt(2);
  return;
}

void robot8Step7(u8 sig){  
  //5 formation 
#ifdef DEBUG
  imReady = sig;
  halt(2);
#else
  for(;;) {
    imReady = sig;
    if (stReady(activeRb[0],sig)) {
      imReady = sig;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[7],sig)){
    vTaskDelay(500);
    imReady = sig;
  }
  if (stReady(activeRb[7],sig)) {
    //goForward();    
    point2Point(posA5.x,posA5.y);
  }
//  imReady = 7;
//  halt(2);
  return;
}

void robot8Step8(){  
  //5 formation 
#ifdef DEBUG
  imReady = 7;
  halt(2);
#else
  for(;;) {
    imReady = 7;
    if (stReady(activeRb[0],7)) {
      imReady = 7;
      halt(1);
      break;
    }
    vTaskDelay(500);
  }
  halt(2);
#endif
  while(!stReady(activeRb[7],7)){
    vTaskDelay(500);
    imReady = 7;
  }
  if (stReady(activeRb[7],7)) {
    //goForward();  
      rotateToNorthAngle(5,FASTSPEED);
      SetLeftWheelGivenSpeed(30);
      SetRightWheelGivenSpeed(35);
      vTaskDelay(8000);
    point2Point(posA8.x,posA8.y);
    rotateToNorthAngle(0,FASTSPEED);
  }
//  imReady = 0;
//  halt(2);
  return;
}

void robot1()
{
  robot1Step1(); //方向统一
  robot1Step2(); //square
  halt(2);
  robot1Step3(); //trangle
  halt(2);
  robot1Step4(0); //circle
  halt(2);
  robot1Step5(); //trangle
  halt(2);
  robot1Step6(9); //square
  halt(2);
  robot1Step7(18); // lines
  halt(2);
}

void robot2()
{
  robot2Step1();
  robot2Step2();
  halt(2);
  robot2Step3();
  halt(2);
  robot2Step4();
  halt(2);
  robot2Step5();
  halt(2);
  robot2Step6(9);
  halt(2);
  robot2Step7(18);
  halt(2);
}

void robot3()
{
  robot3Step1();
  robot3Step2();
  halt(2);
  robot3Step3();
  halt(2);
  robot3Step4();
  halt(2);
  robot3Step5();
  halt(2);
  robot3Step6(9);
  halt(2);
  robot3Step7(18);
  halt(2);
}

void robot4()
{
  robot4Step1();
  robot4Step2();
  halt(2);
  robot4Step3();
  halt(2);
  robot4Step4();
  halt(2);
  robot4Step5();
  halt(2);
  robot4Step6(9);
  halt(2);
  robot4Step7(18);
  halt(2);
}

void robot5()
{
  robot5Step1();
  robot5Step2();
  halt(2);
  robot5Step3();
  halt(2);
  robot5Step4(0);
  halt(2);
  robot5Step5();
  halt(2);
  robot5Step6(9);
  halt(2);
  robot5Step7(18);
  halt(2);
}

void robot6()
{
  robot6Step1();
  robot6Step2();
  halt(2);
  robot6Step3();
  halt(2);
  robot6Step4();
  halt(2);
  robot6Step5();
  halt(2);
  robot6Step6(9);
  halt(2);
  robot6Step7(18);
  halt(2);
}

void robot7(){
  robot7Step1();
  robot7Step2();
  halt(2);
  robot7Step3();
  halt(2);
  robot7Step4();
  halt(2);
  robot7Step5();
  halt(2);
  robot7Step6(9);
  halt(2);
  robot7Step7(18);
  halt(2);
}

void robot8(){
  robot8Step1();
  robot8Step2();
  halt(2);
  robot8Step3();
  halt(2);
  robot8Step4(0);
  halt(2);
  robot8Step5();
  halt(2);
  robot8Step6(9);
  halt(2);
  robot8Step7(18);
  halt(2);
}

void robot1_2()
{
  robot1Step6(10);
  halt(2);
  robot1Step5();
  halt(2);
  robot1Step4(1);
  halt(2);
  robot1Step3();
  halt(2);
  robot1Step2();
  halt(2);
  robot1Step8();
}

void robot2_2()
{
  robot2Step6(10);
  halt(2);
  robot2Step5();
  halt(2);
  robot2Step4();
  halt(2);
  robot2Step3();
  halt(2);
  robot2Step2();
  halt(2);
  robot2Step8();
}

void robot3_2()
{
  robot3Step6(10);
  halt(2);
  robot3Step5();
  halt(2);
  robot3Step4();
  halt(2);
  robot3Step3();
  halt(2);
  robot3Step2();
  halt(2);
  robot3Step8();
}

void robot4_2()
{
  robot4Step6(10);
  halt(2);
  robot4Step5();
  halt(2);
  robot4Step4();
  halt(2);
  robot4Step3();
  halt(2);
  robot4Step2();
  halt(2);
  robot4Step8();
}

void robot5_2()
{
  robot5Step6(10);
  halt(2);
  robot5Step5();
  halt(2);
  robot5Step4(1);
  halt(2);
  robot5Step3();
  halt(2);
  robot5Step2();
  halt(2);
  robot5Step8();
}

void robot6_2()
{
  robot6Step6(10);
  halt(2);
  robot6Step5();
  halt(2);
  robot6Step4();
  halt(2);
  robot6Step3();
  halt(2);
  robot6Step2();
  halt(2);
  robot6Step8();
}

void robot7_2()
{
  robot7Step6(10);
  halt(2);
  robot7Step5();
  halt(2);
  robot7Step4();
  halt(2);
  robot7Step3();
  halt(2);
  robot7Step2();
  halt(2);
  robot7Step8();
}

void robot8_2(){
  robot8Step6(10);
  halt(2);
  robot8Step5();
  halt(2);
  robot8Step4(1);
  halt(2);
  robot8Step3();
  halt(2);
  robot8Step2();
  halt(2);
  robot8Step8();
}

void robotsWait(){
  if(rbID == activeRb[0]){
    imReady = 0;
    halt(0.5);
  }else if(rbID == activeRb[1]){
    imReady = 0;
    halt(5);
  }else if(rbID == activeRb[2]){
    imReady = 0;
    halt(7);
  }else if(rbID == activeRb[3]){
    imReady = 0;
    halt(9);
  }else if(rbID == activeRb[4]){
    imReady = 0;
    halt(11);
  }else if(rbID == activeRb[5]){
    imReady = 0;
    halt(13);
  }else if(rbID == activeRb[6]){
    imReady = 0;
    halt(15);
  }else if(rbID == activeRb[7]){
    imReady = 0;
    halt(17);
  }
}

void shapeForm2(){
  if(rbID == activeRb[0]){
    robot1_2();
  }else if(rbID == activeRb[1]){
    robot2_2();
  }else if(rbID == activeRb[2]){
    robot3_2();
  }else if(rbID == activeRb[3]){
    robot4_2();
  }else if(rbID == activeRb[4]){
    robot5_2();
  }else if(rbID == activeRb[5]){
    robot6_2();
  }else if(rbID == activeRb[6]){
    robot7_2();
  }else if(rbID == activeRb[7]){
    robot8_2();
  }    
}

void shapeForm(){
  if(rbID == activeRb[0]){
    robot1();
  }else if(rbID == activeRb[1]){
    robot2();
  }else if(rbID == activeRb[2]){
    robot3();
  }else if(rbID == activeRb[3]){
    robot4();
  }else if(rbID == activeRb[4]){
    robot5();
  }else if(rbID == activeRb[5]){
    robot6();
  }else if(rbID == activeRb[6]){
    robot7();
  }else if(rbID == activeRb[7]){
    robot8();
  }    
}

void standLine(){
  if(rbID == activeRb[0]){
    goFastStraight(posA1.x,posA1.y);
  }else if(rbID == activeRb[1]){
    goFastStraight(posA2.x,posA2.y);
  }else if(rbID == activeRb[2]){
    goFastStraight(posA3.x,posA3.y);
  }else if(rbID == activeRb[3]){
    goFastStraight(posA4.x,posA4.y);
  }else if(rbID == activeRb[4]){
    goFastStraight(posA5.x,posA5.y);
  }else if(rbID == activeRb[5]){
    goFastStraight(posA6.x,posA6.y);
  }else if(rbID == activeRb[6]){
    goFastStraight(posA7.x,posA7.y);
  }else if(rbID == activeRb[7]){
    goFastStraight(posA8.x,posA8.y);
  }
}

void getDataSample(){
  goFastStraight(posD4.x,posD4.y);
  goFastStraight(posD3.x,posD3.y);
  goFastStraight(posD2.x,posD2.y);
  goFastStraight(posD1.x,posD1.y); 
}

void formationControl(){  
  snakeForm(posS1.x, posS1.y);  
  robotsWait();  
  snakeStart(posS1.x,posS1.y);  
  standLine();
  for(;;){
    shapeForm();
    sychronize();
    vTaskDelay(500);
    shapeForm2();
    sychronize();
    vTaskDelay(500);
  }  
}

void vDemoOneTask( void *pvParameters ){ 
  formationControl();
}


