#include "vBCastInfoTask.h"
#include "apps.h"
#include "vInfoList.h"

rbNode bcastInfo;
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

//static int count = 0;
//	count++;
//	if(count == 8){
//	  count = 0;
//	  clearNbrMsg();
//	}

portTickType xBcastInfoTime;
void vBCastInfoTask( void *pvParameters ){
  //  static UBaseType_t uxHighWaterMark;
  //  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  while(1) {
    
    xBcastInfoTime = xTaskGetTickCount();
    typeCoordinate coordinateK;
    vTaskDelay(LOCATION_PERIOD);
    
    coordinateK = GetCoordinate();
    //	u8 x = isnan(coordinateK.x);
    //	u8 y = isnan(coordinateK.y);
    //	while(x || y){
    //	    halt(0.2);
    //		coordinateK = GetCoordinate();
    //		x = isnan(coordinateK.x);
    //	    y = isnan(coordinateK.y);
    //	}
    
    bcastInfo.nodeID = rbID;  
    bcastInfo.rpos.locationX = coordinateK.x;
    bcastInfo.rpos.locationY = coordinateK.y;
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
