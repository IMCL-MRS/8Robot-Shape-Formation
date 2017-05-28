#ifndef NEIGHBOURS_H
#define NEIGHBOURS_H

#define ROBOT_ID_NULL  0

/******** Defines ********/
#define NEIGHBOR_PERIOD_DEFAULT 		300
#define NEIGHBOR_TIMEOUT_ROUNDS			4
#define OBSTACLE_TIMEOUT_ROUNDS			2
#define NEIGHBOR_MAX					12

#define NEIGHBORS_TASK_PRIORITY         (tskIDLE_PRIORITY)
#define KEEP_DISTANCE_TASK_PRIORITY     (tskIDLE_PRIORITY)

extern uint8 electLeader(u8 initor,float x, float y);
extern void nbrDataSet(u8 ID, u8 type);
extern void neighborsGetMutex(void);
extern void neighborsPutMutex(void);
extern void neighborsInit(uint32 neighbor_period_arg);
extern int getActiveRbNum();
extern void neighborsDist( void* parameters);
extern u8 neighborsInit1();
extern u8 removeNode(u8 ID);
extern void clearNodeMsg();
extern bool followerInit(u8 initor);
extern bool robotsReady();

#endif /*NEIGHBOURS_H */