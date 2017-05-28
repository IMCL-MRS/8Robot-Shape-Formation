
#include "vLocalizationTask.h"

////////////////////////////////////////////////////////////////////////////////
//卡尔曼滤波
////////////////////////////////////////////////////////////////////////////////
////Q=0.001效果好，但时间延时很长
float X_k1_k1=0, P_k1_k1=1, Q = 0.001, R = 0.1, Kg=1;
float X_k_k=0, P_k_k1=0, P_k_k=0, X_k_k1=0;
float KalmanFilterCal(float xMeatrue, float xPridict){
  float Z_k;
  Q = 0.01;
  R = 0.1;
  
  Z_k = xMeatrue;         //观测量
  X_k_k1 = xPridict;      //预测量
  
  P_k_k1 = P_k1_k1 + Q;
  X_k_k = X_k_k1 + Kg*(Z_k-X_k_k1);
  Kg = P_k_k1 / (P_k_k1 + R);
  P_k_k = (1-Kg)*P_k_k1;
  P_k1_k1 = P_k_k;
  X_k1_k1 = X_k_k;
  return X_k_k;
}

typeCoordinate coordinateK;

typeCoordinate coordinateLast;

void vLocalizationTask( void *pvParameters ) {
  while(1) {
    typeCoordinate coordinate;
//    typeWheelSpeed wheelSpeed;
    //float angle;
    //float vm;
    vTaskDelay(LOCATION_PERIOD);
    coordinate = GetCoordinate();
//    wheelSpeed = GetWheelSpeed();
    //angle = ReadMagSensorAngle2North();
    //vm = (wheelSpeed.left + wheelSpeed.right)/2*0.001;
    
    coordinateK = coordinate;
    //coordinateK.x = KalmanFilterCal(coordinate.x, coordinateLast.x + 0.2*vm*cos((angle-90)*3.14/180));
    //coordinateK.y = KalmanFilterCal(coordinate.y, coordinateLast.y + 0.2*vm*sin((angle-90)*3.14/180));
    
    coordinateLast = coordinateK;
    
    asm("NOP");
  }
}

typeCoordinate GetCoordinateK(void) {
  return coordinateK;
}