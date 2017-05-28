#ifndef __ROBOT_H
#define __ROBOT_H

#include "apps.h"
#include "halDriver.h"
#include "vDemoCtl.h"

#define DISTANSE_B1_2_B2         (0.54)
#define DISTANSE_B_2_GROUND      (0.45)
#define MAX_ROTATE_SPEED         (15)

#define ROBOT_STABLE_ACCEL_X_THR (0.08)
#define ROBOT_STABLE_ACCEL_Y_THR (0.01)
#define ROBOT_STABLE_RETRY_THR   (20)
#define MS_SECOND				 (1000)
#define OBS_TIMEOUT              (7)

#define CLOCKWISE                (1)
#define ANTI_CLOCKWISE           (-1)

#define MAX_MOTORDETA            (8)
#define MIN_MOTORDETA            (3)

#define LONG_LINE                (1)

#define MIN_RANGE                (0.02)
#define MAX_RANGE                (0.05)

#define LITTLE_RANGE             0
#define BIG_RANGE                1

#define ROTATE_ACCURATE          (0)
#define ROTATE_LARGER_15         (1)
#define ROTATE_LARGER_25         (2)

#define GOBACK_TIMES             (4)

#define RIGHT_HAND               (1)
#define LEFT_HAND               (-1)

#define PHASE_ONE                (1)
#define PHASE_TWO                (2)
#define PHASE_THREE              (3)

#define FRONT_RBID				 (rbID-1)


typedef struct typeCoordinate{
  float x;
  float y;
}typeCoordinate;

extern typeCoordinate coordinateK;
extern typeCoordinate GetCoordinate(void);
extern void ControlRobotRotate(float angle, float speed);
extern void RobotPrepare(void);
extern float GetRobotAngle(void);
extern void SetRobotAngle(float angle);
extern float GetLineDirection(float x0, float y0, float x1, float y1);
extern float GetLineDirectionX(float x0, float y0, float x1, float y1);
extern float ReadMagSensorAngle2North(void);
extern void northRotate(float x, float y, float speed);
extern float getDistance2(float Ax, float Ay, float Bx, float By);
extern int whichSideByPoint (float ax, float ay, float bx, float by, float cx, float cy);
extern int whichSide (float x, float y);
extern bool GoalInFront(float x, float y,int phase);
extern bool GoalInFront1(float x, float y,int phase);

extern void rotateToNorthAngle(float tar, float speed);
extern void ControlRobotgo2Position(float x, float y, float speed, int flag);
extern void ControlRobotRotate(float angle, float speed);
extern bool ObjectInFront(float dir,typeCoordinate coordinate, float* interAngle,float length,int *leftOrRight);
extern float r2rAngle(float lineDir, float myDir);
extern void rotateTo(float x, float y, float speed, int flag);
extern void rotateFastTo(float x, float y, float speed,int flag);
extern void rotateToNorthAngle(float tar, float speed);
extern u8 rotateToSafe(float x,float y);
extern u8 sleepSafe(float length);
extern int SleepUntilSafe(u8 frontR, float length);
extern void runStraight(float x, float y, float speed);
extern void goStraight(float x, float y, float speed,int fRbID, int phase);
extern int reachLine(float x, float y, float speed);
extern u8 IsinActive(u8 id);
extern int otherIsinActive(u8 id);
extern u8 TargetPosIsDead(float x, float y, u8 id);
extern typeCoordinate getNowPos(float x, float y);
extern float CalibrateNorth2X(void);
extern int SleepUntilFrontSafe(u8 index, float length);

extern void gotoLeftDelta();
extern void gotoRightDelta();
extern void goForward();
extern void gotoRightStep2();

extern void specialRun(float x, float y,float speedR);
#endif


