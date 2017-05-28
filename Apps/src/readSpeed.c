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
//��ȡģ���������ֵ��ٶ�
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
  //���ӱ����ۻ������������10ms������
  //////////////////////////////////////////////////////////////////////////////
  pulseAccL+=readYValue;
  pulseAccR+=readXValue;
  
  currentReadSpeedTime = xTaskGetTickCount();
  xTimeDiff = currentReadSpeedTime-lastReadSpeedTime;
  lastReadSpeedTime = currentReadSpeedTime;
  //��Ϊ�������������෴��װ������ȷ��ǰ������
  if (xTimeDiff != 0){
    wheelSpeedTemp.right  = ((float)readXValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)xTimeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;
    wheelSpeedTemp.left   = ((float)readYValue/(float)MOTOR_FEEDBACK_TICKS)*(1000.0/(float)xTimeDiff)*WHEEL_DIAMETER*PI*MOTOR_GEAR_RATIO*WHEEL_GEAR_RATIO;  
  }
  weelFeedbackSpeed.left  = wheelSpeedTemp.left;
  weelFeedbackSpeed.right = wheelSpeedTemp.right;
  return wheelSpeedTemp;
}

