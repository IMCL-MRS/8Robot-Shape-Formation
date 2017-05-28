#include "AStar.h"

extern rbNode bCastInfo[GROBOTS];
extern u8 activeRb[]; 

const typeCoordinate posEnd  = {0.3,0.5};
wayPoint endWayPoint = {8,2};
wayPoint obsStart;
wayPoint obsEnd;
wayPoint wayArr[STAR] = {0};
int turnPoint = 0;

//static long world[][mapWidth] = {
//(200,800), (300,800), (400,800), (500,800), (600,800), (700,800), (800,800), (900,800), (1000,800), (1100,800), (1200,800), (1300,800), 
//(200,700), (300,700), (400,700), (500,700), (600,700), (700,700), (800,700), (900,700), (1000,700), (1100,700), (1200,700), (1300,700), 
//(200,600), (300,600), (400,600), (500,600), (600,600), (700,600), (800,600), (900,600), (1000,600), (1100,600), (1200,600), (1300,600), 
//(200,500), (300,500), (400,500), (500,500), (600,500), (700,500), (800,500), (900,500), (1000,500), (1100,500), (1200,500), (1300,500), 
//(200,400), (300,400), (400,400), (500,400), (600,400), (700,400), (800,400), (900,400), (1000,400), (1100,400), (1200,400), (1300,400), 
//(200,300), (300,300), (400,300), (500,300), (600,300), (700,300), (800,300), (900,300), (1000,300), (1100,300), (1200,300), (1300,300), 
//(200,200), (300,200), (400,200), (500,200), (600,200), (700,200), (800,200), (900,200), (1000,200), (1100,200), (1200,200), (1300,200), 
//(200,100), (300,100), (400,100), (500,100), (600,100), (700,100), (800,100), (900,100), (1000,100), (1100,100), (1200,100), (1300,100), 
//(200,0), (300,0), (400,0), (500,0), (600,0), (700,0), (800,0), (900,0), (1000,0), (1100,0), (1200,0), (1300,0), 
//};

//0: start, 1:waypoint, 2: obstacle, 3:endpoint
//static long mapPoint[mapHeight][mapWidth] = { 
//  1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1,
//  1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1,
//  1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1,
//  1, 3, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1,
//  1, 1, 1, 2, 2, 2, 1, 2, 2, 2, 1, 1,
//  1, 1, 1, 2, 2, 2, 1, 2, 2, 2, 1, 1,
//  1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1,
//  1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1,
//  1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1
//};

#if (defined(VISIT_DEMO) || defined(MAIN_DEMO1))

static long mapPoint[mapHeight][mapWidth] = { 
  1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1,
  1, 5, 1, 1, 1, 1, 1, 1, 4, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1,
  1, 1, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1
};
#elif defined(MAIN_DEMO2)
static long mapPoint[mapHeight][mapWidth] = { 
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1,
  1, 5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1,
  1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1
};
#endif

PathNode  mapNode[mapHeight][mapWidth];   // 结点数组
pPathNode openTable[100] = {0};		     // open表
pPathNode closeTable[100] = {0};		 // close表

int numOpenTable = 0;
int numCloseTable = 0;

pPathNode pathStack[20] = {0};		// 保存路径的栈
int        top = -1;			// 栈顶

PathNode *startNode = NULL;		 // 起始点
PathNode *endNode   = NULL;		 // 结束点
PathNode *currNode  = NULL;		 // 当前点
int       isFound = 0;			     // 是否找到路径

//static uint32 getG(PathNode *pNode){
//  return pNode->g;
//}
//
//static void setG(PathNode *pNode,uint32 g){
//  pNode->g = g;
//}
//
//static uint32 getH(PathNode *pNode){
//  return pNode->h;
//}
//
//static void setH(PathNode *pNode,uint32 h){
//  pNode->h = h;
//}
//
//static uint32 getF(PathNode *pNode){
//  return pNode->g + pNode->h;
//}
//
//
//static robotPos mapAt(int x, int y){
//  robotPos posTemp = {0,0};
//  if(x >= 0 && x < mapWidth && y >= 0 && y< mapHeight ){
//	posTemp.locationX = (float)(380+x*100);
//	posTemp.locationY = (y*100);
//	return posTemp;
//  }
//  return posTemp;
//}

//static boolean reachPoint(int x, int y){
//  typeCoordinate nowPos = GetCoordinate();
//  int32 currX = (int32)(nowPos.x*1000);
//  int32 currY = (int32)(nowPos.y*1000);
//  
//  int32 xMax = 380 + x*100 + 30;
//  int32 xMin = 380 + x*100 - 30;
//  int32 yMax = y*100 + 30;
//  int32 yMin = y*100 - 30;
//
//  boolean xRange = ((currX >= xMin) && (currX <= xMax))?1:0;
//  boolean yRange = ((currY >= yMin) && (currY <= yMax))?1:0;
//  if(xRange && yRange){
//	return TRUE;
//  }else{
//	return FALSE;
//  }
//}

uint32 sqrtInt(uint32 val) {
  uint32 g;
  uint32 bit = 0x8000;
  uint32 result = 0;
  uint32 test;
  
  
  // can't take the root of numbers higher than MAX_INT32
  if (val >= 0x80000000) {
	result = 46341;
  }
  else {
	while (bit != 0) {
	  g = result | bit;
	  test = g * g;
	  if (test == val) {
		// we're done!
		result = g;
		break;
	  }else if (test < val) {
		// our guess is too small, keep the temp bit
		result |= bit;
	  }
	  // shift the bit down one
	  bit >>= 1;
	}
  }
  return result;
}

//static uint32 getDistanceNbr(uint32 x1,uint32 y1, uint32 x2, uint32 y2){
//  uint32 xDeta = (x1*x1 - x2*x2);
//  uint32 yDeta = (y1*y1 - y2*y2);
//  return sqrtInt(xDeta + yDeta);
//}

boolean checkNAN(typeCoordinate pos){
  u8 x = isnan(pos.x);
  u8 y = isnan(pos.y);
  if(x || y)
	return 1;
  else
	return 0;
}


static boolean getStartNode(){
  u8 i = 0,j = 0;
  int32 xMin, xMax, yMin, yMax;
  typeCoordinate nowPos = getNowPos(0,0); //modified on 26/5/2016
  int32 x = (int32)(nowPos.x*1000);
  int32 y = (int32)(nowPos.y*1000);
  for(i = 0; i < mapHeight; i++){
	for(j = 0;j < mapWidth; j++){
	  xMin = xStart + j*oneStep - 50;
	  xMax = xStart + j*oneStep + 50;
	  yMin = yStart - i*oneStep - 50;
	  yMax = yStart - i*oneStep + 50;
	  
	  if((x >= xMin) && (x <= xMax) && (y >= yMin) && (y <= yMax)){
		startNode->i = i;  //map to mapPoint
		startNode->j = j;
		return TRUE;
	  }
	}
	asm("NOP");
  }
  asm("NOP");
  return FALSE;
}

static boolean reachEndNode(int x, int y){
  int32 xMax = xStart + endNode->j*oneStep + 30;
  int32 xMin = xStart + endNode->j*oneStep - 30; 
  int32 yMax = yStart - endNode->i*oneStep + 30;
  int32 yMin = yStart - endNode->i*oneStep - 30;
  
  int32 currX = (xStart + y*oneStep);
  int32 currY = (yStart - x*oneStep);
  boolean xRange = ((currX >= xMin) && (currX <= xMax))?1:0;
  boolean yRange = ((currY >= yMin) && (currY <= yMax))?1:0;
  if(xRange && yRange){
	return TRUE;
  }else{
	return FALSE;
  }
}

// 交换两个元素
static void swap( int idx1, int idx2 ){  
  pPathNode tmp = openTable[idx1];
  openTable[idx1] = openTable[idx2];
  openTable[idx2] = tmp;
}  

// 堆调整
static void adjustHeap( int nIndex ){ 	
  int curr = nIndex;
  int child = curr * 2 + 1;	        // 得到左孩子idx( 下标从0开始，所有左孩子是curr*2+1 )
  int parent = ( curr - 1 ) / 2;	// 得到双亲idx
  
  if (nIndex < 0 || nIndex >= numOpenTable){
	return;
  }
  
  // 往下调整( 要比较左右孩子和cuur parent )
  while ( child < numOpenTable ){
	// 小根堆是双亲值小于孩子值	
	if ( ((child + 1) < numOpenTable) && (openTable[child]->g + openTable[child]->h)  > (openTable[child+1]->g + openTable[child+1]->h) ){
	  ++child;  // 判断左右孩子大小
	}
	
	if ((openTable[curr]->g + openTable[curr]->h) <= (openTable[child]->g + openTable[child]->h)){
	  break;
	}
	else{
	  swap( child, curr );			// 交换节点
	  curr = child;				    // 再判断当前孩子节点
	  child = curr * 2 + 1;			// 再判断左孩子
	}
  }
  
  if (curr != nIndex){
	return;
  }
  
  // 往上调整( 只需要比较cuur child和parent )
  while (curr != 0){
	if ((openTable[curr]->g + openTable[curr]->h) >= (openTable[parent]->g + openTable[parent]->h)){
	  break;
	}
	else{
	  swap( curr, parent );
	  curr = parent;
	  parent = (curr-1)/2;
	}
  }
}

static void insertIntoOpenTable(int x, int y, pPathNode currNode, pPathNode endNode, uint32 g){
  int i;
  if( mapNode[x][y].style != POINT_OBSTACLE){     //not obstacle
	if(!mapNode[x][y].isInCloseTable){            //not in the close table
	  if(mapNode[x][y].isInOpenTable){            //in the open table
		if(mapNode[x][y].g > (currNode->g + g)){  //get the best node
		  mapNode[x][y].g = currNode->g + g;
		  mapNode[x][y].parent = currNode;
		  for ( i = 0; i < numOpenTable; ++i ){
			if ( openTable[i]->i == mapNode[x][y].i && openTable[i]->j == mapNode[x][y].j ){
			  break;
			}
		  }
		  //here you can sort the openTable
		  adjustHeap( i );				// heap sorting
		}
	  }else{
		mapNode[x][y].g = currNode->g + g;
		mapNode[x][y].h = 10*(abs(endNode->i - x ) + abs(endNode->j - y)); //getDistanceNbr(x,y,endNode->x,endNode->y);
		mapNode[x][y].parent = currNode;
		mapNode[x][y].isInOpenTable = 1;
		openTable[numOpenTable++] = &(mapNode[x][y]);
	  }
	}
  }
}

static void getNeighbors( pPathNode currNode, pPathNode endNode ){
  int x = currNode->i;
  int y = currNode->j;
  
  // 下面对于8个邻居进行处理！
  if ( x >= 0 && x < mapHeight && ( y + 1 ) >= 0 && ( y + 1 ) < mapWidth ){ //right
	insertIntoOpenTable( x, y+1, currNode, endNode, 10 );
  }  
  
  if ( x >= 0 && x < mapHeight && ( y - 1 ) >= 0 && ( y - 1 ) < mapWidth ){ //left
	insertIntoOpenTable( x, y-1, currNode, endNode, 10 );
  }  
  
  if ( ( x - 1 ) >= 0 && ( x - 1 ) < mapHeight && y >= 0 && y < mapWidth ){  //up
	insertIntoOpenTable( x-1, y, currNode, endNode, 10 );
  }  
  
  if ( ( x + 1 ) >= 0 && ( x + 1 ) < mapHeight && y >= 0 && y < mapWidth ){  //down
	insertIntoOpenTable( x+1, y, currNode, endNode, 10 );
  }  
  
  if ( ( x - 1 ) >= 0 && ( x - 1 ) < mapHeight && ( y + 1 ) >= 0 && ( y + 1 ) < mapWidth ){ //upper right
	insertIntoOpenTable( x-1, y+1, currNode, endNode, 14 );
  }   
  
  if ( ( x - 1 ) >= 0 && ( x - 1 ) < mapHeight && ( y - 1 ) >= 0 && ( y - 1 ) < mapWidth ){ //upper left
	insertIntoOpenTable( x-1, y-1, currNode, endNode, 14 );
  }
  
  if ( ( x + 1 ) >= 0 && ( x + 1 ) < mapHeight && ( y + 1 ) >= 0 && ( y + 1 ) < mapWidth ){ //lower right
	insertIntoOpenTable( x+1, y+1, currNode, endNode, 14 );
  }
  
  if ( ( x + 1 ) >= 0 && ( x + 1 ) < mapHeight && ( y - 1 ) >= 0 && ( y - 1 ) < mapWidth ){ //lower right
	insertIntoOpenTable( x+1, y-1, currNode, endNode, 14 );
  }
  
  
}

static void initMapNode(){
  u8 i, j;
  for(i = 0; i < mapHeight; i++){
	for(j = 0; j < mapWidth; j++){
	  mapNode[i][j].i = i;
	  mapNode[i][j].j = j;
	  mapNode[i][j].g = 0;
	  mapNode[i][j].h = 0;
	  mapNode[i][j].style = mapPoint[i][j];
	  mapNode[i][j].parent = NULL;
	  mapNode[i][j].isInCloseTable = 0;
	  mapNode[i][j].isInOpenTable = 0;
	  
	  if( mapNode[i][j].style == POINT_START ){  // startPoint
		mapNode[i][j].style = -1;
	  }
	  else if ( mapNode[i][j].style == OBS_EXIT/*POINT_END*/ )	//end point
	  {
		endNode = &(mapNode[i][j]);
		endNode->i = i;
		endNode->j = j;
		endNode->h = 0;
		endNode->parent = NULL;
		endWayPoint.i = i;
		endWayPoint.j = j;
		obsEnd.i = i;
		obsEnd.j = j;
	  }else if( mapNode[i][j].style == OBS_ENTRY){
		obsStart.i = i;
		obsStart.j = j;
	  }else if( mapNode[i][j].style == OBS_EXIT){
	    obsEnd.i = i;
		obsEnd.j = j;
	  }
	}
  }
}


static bool isInline(int x1, int y1, int x2, int y2, int x3, int y3){  
  if((y2 - y1)*((-x3) - (-x2)) == (y3 - y2)*((-x2) - (-x1)))
	return TRUE;
  else
  	return FALSE;
} 

bool getTurnPoint(){
  if(top <= 2){
	while(top >= 0){
	  wayArr[turnPoint].i = pathStack[top]->i;
	  wayArr[turnPoint].i = pathStack[top]->j;
	  top -= 1;
	  if(top < 0)
		return TRUE;
	}
  }else{
	wayArr[turnPoint].i = pathStack[top]->i;
	wayArr[turnPoint].j = pathStack[top]->j;
	turnPoint += 1;
	while(top >=0){
	  int x1 = pathStack[top]->i;
	  int y1 = pathStack[top]->j;  //5
	 
	  top -= 1;
	  int x2 = pathStack[top]->i;
	  int y2 = pathStack[top]->j;  //4
	  
	  top -= 1;
	  int currX = pathStack[top]->i;
	  int currY = pathStack[top]->j; //3
	  top += 1;
	  if(!isInline(x1, y1, x2, y2,currX,currY)){
		wayArr[turnPoint].i = x2;
	    wayArr[turnPoint].j = y2;
		turnPoint += 1;
//		wayArr[k].i = currX;
//	    wayArr[k].j = currY;
//	    k += 1;
	  }
	}
  }
  return TRUE;
}

boolean pathFinding(){
  startNode = (struct PathNode *)malloc(sizeof(struct PathNode));
  endNode = (struct PathNode *)malloc(sizeof(struct PathNode));
  initMapNode();
  if(getStartNode()){
	openTable[numOpenTable++] = startNode;	 // add start point to the openTable
  }else{
	return FALSE;  //the start node is not found.
  }
  
  startNode->isInOpenTable = 1;			    // add into openTable
  startNode->g = 0;
  startNode->h = 10*(abs(endNode->i - startNode->i) + abs(endNode->j - startNode->j));// getDistanceNbr(startNode->x,startNode->y,endNode->x,endNode->y); ;//
  startNode->parent = NULL;
  startNode->style = 0;
  
  if ( reachEndNode(startNode->i, startNode->j)){
	return TRUE;
  }
  isFound = 0;
  
  while(1){
	currNode = openTable[0];		// openTable[0] has the lowest f through heap sorting.
	openTable[0] = openTable[--numOpenTable];	// put the last node into openTable[0]
	adjustHeap(0);				   // adjust heap
	
	closeTable[numCloseTable++] = currNode;	// add into closeTable
	currNode->isInCloseTable = 1;		   // in the closeTable
	
	if ( currNode->i == endNode->i && currNode->j == endNode->j ){  //end node is in the closeTable, over!!
	  isFound = 1;
	  break;
	}
	
	getNeighbors( currNode, endNode );			// get neighbors
	if ( numOpenTable == 0 ){				   // the road is not found
	  isFound = 0;
	  break;
	}
  }
  
  if ( isFound ){
	currNode = endNode;
	
	while( currNode ){  //store the tarcking point
	  pathStack[++top] = currNode;
	  currNode = currNode->parent;
	}
	
	getTurnPoint();
//	asm("NOP");
	
	//	if(bCastInfo[rbID - 1].type == LEADER){
	//	  while( top >= 0 )		// 按照前面寻找的路径开始前进
	//	  {
	//		if ( top > 0 )
	//		{
	//		  goOneStep(pathStack[top--]);
	//		}
	//		else
	//		{
	//		  halt(10000);
	//		}
	//	  }
	//	}else if(bCastInfo[rbID - 1].type == FOLLOWER){
	//	  
	//	}
  } /*end if*/
  //  free(startNode);   //not test
  //  free(endNode);
  return isFound;
}

void demoPathPlan(){
  obsStart.i = 5;
  obsStart.j = 8;
  
  obsEnd.i = 5;
  obsEnd.j = 1;
}

