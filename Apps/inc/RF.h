#ifndef __RF_H
#define __RF_H

#include "uart.h"
#include "stm32f10x.h"
#include "stdbool.h"
#include "string.h"
#include "led.h"
#include "vInfoList.h"

#define RF_BROADCAST_ADDR        (0xFF)
#define RF_BROADCAST_INFO        (0xA1)
#define RF_DIST_2_BEACON1        (0xA2)
#define RF_DIST_2_BEACON2        (0xA3)
#define RF_BROADCAST_USONIC      (0xA4)

#define RF_REC_BUF_SIZE          (200)


typedef struct typeRFPacket{
  u8  header1;
  u8  header2;
  u8  command;
  u8  payloadLen;
  u8* payload;
  u8  sum;
}typeRFPacket;

extern void RFInit(void);
extern bool ReadRFIndicationPin1(void);
extern bool ReadRFIndicationPin2(void);
extern u8 RFTxPacket(u8 command, u8* packet, u8 packetLen);

extern void SetDistanse2B1(float dis);
extern void SetDistanse2B2(float dis);
extern float GetDistanse2B1(void);
extern float GetDistanse2B2(void);
extern float GetPosX(void);
extern float GetPosY(void);
extern u8 isReady0(u8 id);
extern u8 isReady(u8 id);
extern u8 stReady(u8 id, u8 status);
extern u8 isReady2(u8 id);
extern u8 isReady3(u8 id);
extern u8 isReady4(u8 id);
extern u8 statusReady(u8 status);
extern u8 initReady(u8 id);
extern u8 isFoundPath(u8 id);
extern u8 behIsActive(u8 id);

extern void RFReset(void);

#endif


