#ifndef __CONTROLMOTOR_H
#define __CONTROLMOTOR_H

#include "sysConfig.h"
#include "halDriver.h"
#include "readSpeed.h"
#include "stm32f10x.h"
#include "function.h"
#include "vInfoList.h"

typedef struct typeAlgSpeedControlResult{
  float left;
  float right;
}typeAlgSpeedControlResult;

extern typeWheelSpeed GetWheelGivenSpeed(void);
extern float GetLeftWheelGivenSpeed(void);
extern float GetRightWheelGivenSpeed(void);
extern void  SetLeftWheelGivenSpeed(float speed);
extern void  SetRightWheelGivenSpeed(float speed);
extern void  SetWheelGivenSpeed(typeWheelSpeed speed);
extern void  ControlWheelSpeedAlg(typeWheelSpeed givenWheelSpeed, typeWheelSpeed feedbackWheelSpeed);
extern void  MotorInit(void);
extern void  MotorRunEnable(bool value);
extern void ControlRobotRotate(float angle, float speed);
extern void ControlRobot2Position(float x, float y, float speed);


#endif


