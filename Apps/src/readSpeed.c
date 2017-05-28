#include "readSpeed.h"
 

typeWheelSpeed weelFeedbackSpeed;
static u32 lastReadSpeedTime=0;

void WheelSpeedSensorInit(void) {
  halABDecoderInit();
}

typeWheelSpeed GetWheelSpeed(void) {
  return weelFeedbackSpeed;
}

////////////////////////////////////////////////////////////////////////////////
//读取模型左右两轮的速度
////////////////////////////////////////////////////////////////////////////////
volatile int64_t pulseAccL=0;
volatile int64_t pulseAccR=0;

typeWheelSpeed ReadWheelSpeed(void){
  typeWheelSpeed wheelSpeedTemp;
  signed long readXValue, readYValue;
  u32 xTimeDiff, currentReadSpeedTime;
  u32 xNowTimeTick;
  xNowTimeTick = xTaskGetTickCount();
  //if ((xNowTimeTick - lastReadSpeedTime)<10){    //max read freq=100HZ
    //return weelFeedbackSpeed;
  //}
  readXValue = halABXDecoderReadData();
  halABClearABXInhibitLogic();
  readYValue = halABYDecoderReadData();
  halABClearABYInhibitLogic();
  //////////////////////////////////////////////////////////////////////////////
  //增加变量累积脉冲数，误差10ms脉冲数
  //////////////////////////////////////////////////////////////////////////////
  pulseAccL+=readYValue;
  pulseAccR+=readXValue;
  
  currentReadSpeedTime = xTaskGetTickCount();
  xTimeDiff = currentReadSpeedTime-lastReadSpeedTime;
  lastReadSpeedTime = currentReadSpeedTime;
  //因为左右两个轮子相反安装，并且确定前进方向
  if (xTimeDiff != 0){
    wheelSpeedTemp.right  = ((float)readXValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)xTimeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;
    wheelSpeedTemp.left   = ((float)readYValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)xTimeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;  
  }
  weelFeedbackSpeed.left  = wheelSpeedTemp.left;
  weelFeedbackSpeed.right = wheelSpeedTemp.right;
  return wheelSpeedTemp;
}

