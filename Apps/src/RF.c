#include <math.h>
#include "vInfoList.h"
#include "RF.h"
////////////////////////////////////////////////////////////////////////////////
//RF init
////////////////////////////////////////////////////////////////////////////////
void RFInit(void) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_SetBits(GPIOD, GPIO_Pin_2);
}

void RFReset(void) {
  GPIO_SetBits(GPIOD, GPIO_Pin_2);
  GPIO_ResetBits(GPIOD, GPIO_Pin_2);
  halMCUWaitUs(50);
  GPIO_SetBits(GPIOD, GPIO_Pin_2);
}

////////////////////////////////////////////////////////////////////////////////
//PB5 PD2 connect to CC2530
////////////////////////////////////////////////////////////////////////////////
bool ReadRFIndicationPin1(void) {
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)==Bit_RESET){
    return false;
  }
  return true;
}
/*
bool ReadRFIndicationPin2(void) {
  if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)==Bit_RESET){
    return false;
  } 
  return true;
}
*/
////////////////////////////////////////////////////////////////////////////////
//RF send packet
////////////////////////////////////////////////////////////////////////////////
u8 RFTxPacket(u8 command, u8* packet, u8 packetLen) {
  u8 sum=0, i=0;
  UartTxByte(0x7E);
  UartTxByte(0x45);
  UartTxByte(command);
  UartTxByte(packetLen);
  UartTxPacket(packet, packetLen);
  sum+=(0x7E+0x45+command+packetLen);
  for (i=0;i<packetLen;i++) {
    sum+=*(packet+i);
  }
  UartTxByte(sum);
  return 1;
}
////////////////////////////////////////////////////////////////////////////////
//RF receive packet
////////////////////////////////////////////////////////////////////////////////
//bool rfPacketRecDone=false;
static u8 rfRxBuf[RF_REC_BUF_SIZE];
static u8 rfRxBufWP=0;

rbNode bCastInfo[ROBOTS];
portTickType xUsartRxTime;

void Usart0RxIsr(u8 data){
  xUsartRxTime = xTaskGetTickCountFromISR();

  if ((rfRxBufWP==0) && (data != 0x7E)){
    rfRxBufWP=0;
  }
  if ((rfRxBufWP==1) && (data != 0x45)){
    rfRxBufWP=0;
  }
  
  if(rfRxBufWP==0) {       //header1
    rfRxBuf[0] = data;
    rfRxBufWP++;
  }
  else if(rfRxBufWP==1) {  //header2
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
  }
  else if(rfRxBufWP==2) {  //command
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
  }
  else if(rfRxBufWP==3) {  //payload Len
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
    if (data>RF_REC_BUF_SIZE-4) {
      rfRxBufWP = 0;
    }
  }
  else if((rfRxBufWP>=4)&&(rfRxBufWP<(rfRxBuf[3]+4))) {  //payload 
    rfRxBuf[rfRxBufWP] = data;
    rfRxBufWP++;
  }
  else if (rfRxBufWP==(rfRxBuf[3]+4)) {  //sum
    rfRxBuf[rfRxBufWP] = data;
    u8 i,sum=0;
    float dis=0;
    for (i=0;i<rfRxBufWP;i++) {
      sum+=rfRxBuf[i];
    } 	
    if (sum==rfRxBuf[rfRxBufWP]) {                   //sum is right!	     
      if (rfRxBuf[2]==RF_DIST_2_BEACON1) {           //dis to B1
        memcpy(&dis, &rfRxBuf[4],4);
        SetDistanse2B1(dis);
      }
      else if (rfRxBuf[2]==RF_DIST_2_BEACON2) {      //dis to B2
        memcpy(&dis, &rfRxBuf[4],4);
        SetDistanse2B2(dis);
		halSetLedStatus(LED_RED, LED_TOGGLE);        //D1 and D2
      }
     else if (rfRxBuf[2]==RF_BROADCAST_INFO) {       //BCAST INFO
        //user add code to decode bcast info
        static rbNode* bInfo;
        bInfo = (rbNode*)(rfRxBuf+4);
        memcpy((u8*)(&bCastInfo[(bInfo->nodeID)-1]), rfRxBuf+4, sizeof(rbNode));
        halSetLedStatus(LED_YELLOW, LED_TOGGLE);  //D3 and D4
        asm ("NOP");
        //user end
      }
    }
    rfRxBufWP =  0;
  }
}

static float distanse2B1=0, distanse2B2=0;

float GetDistanse2B1(void) {
  return distanse2B1;
}

float GetDistanse2B2(void) {
  return distanse2B2;
}

void SetDistanse2B1(float dis) {
  distanse2B1=dis;
}
void SetDistanse2B2(float dis) {
  distanse2B2=dis;
}

//void setPos(float distB1, float distB2){
//  float distBeacon = DISTANSE_B1_2_B2;
//  float height = DISTANSE_B_2_GROUND;
////  float cosa = (distB1*distB1 + distBeacon * distBeacon - distB2*distB2)/(2*distBeacon*distB1);
//  newPos.Y = (distB1*distB1 - distB2*distB2 + distBeacon*distBeacon)/(2*distBeacon);
//  float xPos = distB1*distB1 - height*height - newPos.Y*newPos.Y;
//  if(xPos >=0){
//	   newPos.X = sqrt(xPos);
//   }
//  
//  SetDistanse2B1(distB1);
//  SetDistanse2B2(distB2);  
//  asm("NOP");
//}

u8 isReady0(u8 id) {
  if(bCastInfo[id-1].isReady == 0){
	  return 1;
  }
  return 0;
}

u8 isReady(u8 id) {
  if(bCastInfo[id-1].isReady == 1){
	  return 1;
  }
  return 0;
}

u8 stReady(u8 id, u8 status) {
  if(bCastInfo[id-1].isReady == status){
	  return 1;
  }
  return 0;
}


u8 isReady2(u8 id) {
  if(bCastInfo[id-1].isReady == 2){
	  return 1;
  }
  return 0;
}

u8 isReady3(u8 id) {
  if(bCastInfo[id-1].isReady == 3){
	  return 1;
  }
  return 0;
}

u8 isReady4(u8 id) {
  if(bCastInfo[id-1].isReady == 4){
	  return 1;
  }
  return 0;
}

u8 initReady(u8 id) {
  return bCastInfo[id-1].isLeaderOK;
}

u8 isFoundPath(u8 id){
  return bCastInfo[id - 1].isFoundPath;
}

u8 behIsActive(u8 id) {
	return bCastInfo[id-1].isActive;
}

//add by sundy
rbNode recBoardCastInfo(u8 id){
  return bCastInfo[id - 1];
}
