#ifndef ASTAR_H
#define ASTAR_H
#include "vInfoList.h"
#include "behaviors.h"
#include "neighbours.h"

#define mapHeight 9
#define mapWidth  12

#define xStart   200
#define yStart   800
#define oneStep  100

#define POINT_START    0
#define POINT_WAY      1
#define POINT_OBSTACLE 2
#define POINT_END      3
#define OBS_ENTRY      4
#define OBS_EXIT       5

#define STAR  40


typedef struct PathNode{
  uint32 i;
  uint32 j;
  uint32 g;
  uint32 h;
  u8     style;   //-1: start, 1:waypoint, 2: obstacle, 3:endpoint
  struct PathNode *parent;
  int isInCloseTable;		// 是否在close表中
  int isInOpenTable;		// 是否在open表中
}PathNode, *pPathNode;


extern boolean pathFinding();
extern void leaderRun(int i, int j);
extern void gotoTarget(int i, int j);
//extern void leaderFormLine(int i, int j, int flag);
//extern void followerFormLine(int i, int j);
extern void demoPathPlan();

#endif  /*end ASTAR_H*/