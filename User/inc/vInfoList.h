
#ifndef VINFOLIST_H
#define VINFOLIST_H

#include "vDemoTask.h"
#include "robot.h"
#include "behaviors.h"
#include "vDemoCtl.h"

#define ROBOTS  8
#define GROBOTS 8
#define CONFIG_ROBOT8

#define FASTSPEED  45
#define LINESPEED  35
#define ANGLESPEED 5    
#define SENDTIMES  5
#define DELAY     (0.5)
#define EYEANGLE  (30)

#define FRONT_SIGHT_LENGTH   (0.13) 
#define SAFE_LENGTH          (0.10)
#define FOV		     (50)	
#define CHECK                (2)

//The angle between X axis and robot front direction
#define FRONT2X              (63) 
//#define _Y2NORTH           (145) //HKSP
#define ACCEL_Z              (-0.5)
#define ACCEL_TIMES          (10)

//nearby robot distance control
#define INIT_TIME  (8)
#define LONG_GAP   (0.35)
#define SHORT_GAP  (0.20)

#define leftHand  -1;
#define rightHand  1;

#ifdef CONFIG_ROBOT1 
//mag sensor para
#define rbID            (1)
#define MAG_SENSOR_X    (-39)
#define MAG_SENSOR_Y    (602)
#elif defined(CONFIG_ROBOT2)
#define rbID            (2)
#define MAG_SENSOR_X    (200)
#define MAG_SENSOR_Y    (793)
//#define MAG_SENSOR_X    (165)
//#define MAG_SENSOR_Y    (-14)
//#define MAG_SENSOR_X    (143)
//#define MAG_SENSOR_Y    (15)
#elif defined(CONFIG_ROBOT3)
#define rbID            (3)
#define MAG_SENSOR_X    (53.5)
#define MAG_SENSOR_Y    (823)
//#define MAG_SENSOR_X    (72)
//#define MAG_SENSOR_Y    (816.5)
#elif defined(CONFIG_ROBOT4)
#define rbID            (4)
#define MAG_SENSOR_X    (91)
#define MAG_SENSOR_Y    (1048)
//#define MAG_SENSOR_X    (38)
//#define MAG_SENSOR_Y    (1125)
#elif defined(CONFIG_ROBOT5)
#define rbID            (5)
#define MAG_SENSOR_X    (144.5)
#define MAG_SENSOR_Y    (867.5)	
//#define MAG_SENSOR_X    (115)
//#define MAG_SENSOR_Y    (854)	
//#define MAG_SENSOR_X    (144)
//#define MAG_SENSOR_Y    (863)
#elif defined(CONFIG_ROBOT6)
#define rbID            (6)
#define MAG_SENSOR_X    (13)
#define MAG_SENSOR_Y    (457)
//#define MAG_SENSOR_X    (-18.5)
//#define MAG_SENSOR_Y    (449.5)
//#define MAG_SENSOR_X    (-42)
//#define MAG_SENSOR_Y    (580)
#elif defined(CONFIG_ROBOT7)
#define rbID            (7)
#define MAG_SENSOR_X    (194)
#define MAG_SENSOR_Y    (80)
#elif defined(CONFIG_ROBOT8)
#define rbID            (8)
#define MAG_SENSOR_X    (2)
#define MAG_SENSOR_Y    (741)
//#define MAG_SENSOR_X    (-8)
//#define MAG_SENSOR_Y    (743)
#elif defined(CONFIG_ROBOTNONE)
#define rbID            (0)
#define MAG_SENSOR_X    (31)
#define MAG_SENSOR_Y    (687)
#endif

#define FRONTRB  (rbID - 1)

//motor para
#define PID_KP     (0.01)
#define PID_KD     (0)
#define PID_KI_1   (0.0001)
#define PID_KI_2   (0.0015)
#define PID_KI_3   (0.002)

typedef struct robotPos{
  float locationX;        //4bytes
  float locationY;        //4bytes
}robotPos;                //8bytes

typedef struct wayPoint{
  uint32 i;
  uint32 j;
}wayPoint;

typedef struct rbDist{
  u8 id;
  float dist;
}rbDist,*prbDist;

//package information 28 Bytes
typedef struct rbNode
{
  u8 nodeID;           //1 byte	
  robotPos rpos;       //8 bytes
  float angle2n;      //4 bytes
  u8 isActive;         //1 byte
  u8 isLeaderOK;       //1 byte
  u8 isReady;          //1 bytes
  u8 isFoundPath;      //1 byte
  u8 type;		     //1 byte
  u8 isStop;           //1 byte
  u8 nbrList[ROBOTS];  //6 byte
}rbNode;

typedef struct InfoNode
{
  struct rbNode rbInfo;
  struct InfoNode *next;
}InfoNode,*pInfoNode;    //24Bytes

//not used
typedef struct InfoDg{
  u8 nodeID;
  float relDis;
}InfoDg;

pInfoNode robotListInit();
void robotListCreatT(InfoNode **L);
pInfoNode robotListInsert(pInfoNode L,int i,rbNode rbInfo);
pInfoNode robotListDelete(pInfoNode L,rbNode rbInfo);
void robotListDestroy(pInfoNode L);
InfoNode getDgNodeFromList(InfoNode *p,robotPos rb,int *flag);
float getRelAngelFromList();
robotPos getRobotPos(InfoNode *rb);

extern float getDistance(robotPos posB,robotPos posC);
extern rbNode recBoardCastInfo(u8 id);

typedef unsigned portBASE_TYPE UBaseType_t;

#endif /*end of VINFOLIST_Hs*/