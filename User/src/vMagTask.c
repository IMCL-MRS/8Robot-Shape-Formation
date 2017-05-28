#include "vDemoTask.h"
#include "robot.h"
#include "vInfoList.h"
#include <math.h>
#include "vBCastInfoTask.h"
#include "apps.h"

#define GETMAG
volatile float magX, magY;

#ifdef GETMAGP
float data1[1000] = {0};
float data2[1000] = {0};
void vMagTask( void *pvParameters ) {
  static typeMagSensor nowdata;
  int datan = 0;
  for (datan = 0; datan < 1000; ++datan) {
	nowdata = ReadMagSensor();
	data1[datan] = nowdata.magX;
	data2[datan] = nowdata.magY;
	SetLeftWheelGivenSpeed(10);
	SetRightWheelGivenSpeed(-10);
	vTaskDelay(50);
	//ControlRobotRotate(15, 5);
  } 
  vTaskDelay(100000);
  asm("NOP");
}

#endif 

#ifdef GETMAG
//get magX and magY
typeMagSensor data[100], nowdata;
float minX, maxX, minY, maxY;
int datan;
void vMagTask( void *pvParameters ) {
  magX = magY = 0;
  minX = minY = 100000;
  maxX = maxY = -100000;
  halt(0.5);
  ControlRobotRotate(15, 5);
  for (datan = 0; datan < 48; ++ datan) {
    nowdata = ReadMagSensor();
    if (minX > nowdata.magX) minX = nowdata.magX;
    if (minY > nowdata.magY) minY = nowdata.magY;
    if (maxX < nowdata.magX) maxX = nowdata.magX;
    if (maxY < nowdata.magY) maxY = nowdata.magY;
    data[datan] = nowdata;
    ControlRobotRotate(15, 5);
    halt(0.5);
  }
  magX = (minX + maxX) / 2;
  magY = (minY + maxY) / 2;
  vTaskDelay(100000);
  asm("NOP");
}
#endif
#ifdef TESTMAG
// test mag
float robotDir;
void vMagTask( void *pvParameters ) {
  while (1) {
    robotDir = ReadMagSensorAngle2North();
    vTaskDelay(1000);
  }
}
#endif

extern rbNode bcastInfo;
extern volatile u8 isActive;
extern volatile u8 rbType;
extern volatile u8 imReady;
extern volatile u8 isLeaderOK;
extern volatile u8 imFoundPath;
extern volatile u8 isStop;
extern rbNode bCastInfo[ROBOTS];
extern volatile u8 nbrList[ROBOTS];
extern u8 infoLeader;
extern u8 stage;

void vBCastMagTask( void *pvParameters ){
  //  static UBaseType_t uxHighWaterMark;
  //  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  while(1) {    
    vTaskDelay(LOCATION_PERIOD);
    
    coordinateK = GetCoordinate();
    bcastInfo.nodeID = rbID; 
    bcastInfo.rpos.locationX = magX;
    bcastInfo.rpos.locationY = magY;
    bcastInfo.angle2n     = ReadMagSensorAngle2North();
    bcastInfo.isActive    = isActive;
    bcastInfo.isLeaderOK  = isLeaderOK;
    bcastInfo.isReady     = imReady;
    bcastInfo.isFoundPath = imFoundPath;
    //bcastInfo.type        = rbType;
    bcastInfo.type        = stage;
    bcastInfo.isStop      = isStop;
    memcpy((u8*)&(bcastInfo.nbrList),(u8 *)(&nbrList),sizeof(nbrList));
    bCastInfo[rbID - 1] = bcastInfo;	/*set own information*/
    
    RFTxPacket(RF_BROADCAST_INFO, (u8*)(&bcastInfo), sizeof(bcastInfo));
    SetLedStatus(LED_GREEN, LED_TOGGLE);
    //	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    asm("NOP");
  }
}
