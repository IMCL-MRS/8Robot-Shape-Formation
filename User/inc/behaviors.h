#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#define MODE_IDLE		 0
#define LEADER   		 1
#define FOLLOWER		 2
#define OBJECT			 3
#define OBJECT_FOUND	 4
#define GRIPPED			 5

#define FOLLOWERS        6

extern uint8 clearNbrMsg();
extern bool groupReady();
extern void followLeader();
extern void runSelf(int i, int j);
extern bool reachWayPoint(float x, float y, int rgFlag);
extern void gotoWayPoint(float x, float y, float speed,int phase);
extern void runDrawCircle(float x, float y, float speed,int phase,int ptNum);
extern void gotoWayPoint1(float x, float y, float speed, int fRbID,int phase);
extern bool isNearWall(float x, float y);
extern u8 electInitiator();
extern u8 getCurrentIndex();
extern void nbrNearRangeBalance();

#endif /*BEHAVIORS_H */