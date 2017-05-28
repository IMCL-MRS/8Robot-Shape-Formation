//#include "vInfoList.h"
//#include "neighbours.h"
//#include "behaviors.h"
//#include "AStar.h"
//
//#define NEIGHBOR_ROUND_PERIOD		400
//#define BEHAVIOR_TASK_PERIOD		50
//
//extern rbNode bCastInfo[ROBOTS];	
//extern u8 activeRb[ROBOTS];
//extern wayPoint wayArr[STAR];
//extern volatile u8 rbType;
//extern wayPoint endWayPoint;
//extern wayPoint obsStart;
//extern wayPoint obsEnd;
//extern int turnPoint;
//extern uint32 rbCount1;
//extern const typeCoordinate posEnd;
//extern wayPoint endWayPoint;
//
//extern volatile u8 isActive;
//volatile u8 imReady = 0;
//volatile u8 isLeaderOK = 0;
//volatile u8 imFoundPath = 0;
//volatile u8 isStop = 0;
//
//void mainDemo(){
//  u8 i = 0;
//  u8 leader = 0;
//  u8 initiator = 0;
//  isActive = 1;
//  halt(0.5);  
//  while(!allRobotAreReady()){
//	halt(0.5);
//  }
//  initiator = 1;
//  if(rbID == initiator){
//	halt(1);
//	leader = electLeader(initiator);
//	while(!leader){
//	  leader =  electLeader(initiator);
//	  halt(0.5);
//	}
//	isLeaderOK = 1;
//	halt(0.5);
//	if(rbCount1 >= 2){
//	  while(1){
//		u8 k = 0;
//		for(i = 0; i < rbCount1; i++){
//		  if(bCastInfo[activeRb[i] - 1].nodeID == rbID)
//			continue;
//		  if(bCastInfo[activeRb[i] - 1].isReady == 1){
//			k++;
//		  }
//		}
//		if(k != rbCount1 - 1){
//		  halt(0.5);
//		}else{
//		  imReady = 1;
//		  halt(0.5);
//		  break;
//		}
//	  }
//	}else{
//	  imReady = 1;
//	  halt(0.5);
//	}
//  }else{  //not initiator
//	halt(1);
//	while(bCastInfo[initiator - 1].isLeaderOK != 1){
//	  halt(0.5);
//	}
//	while(!followerInit(initiator)){
//	  halt(0.5);
//	}
//	imReady = 1;
//	halt(0.5);
//	
//	while(1){
//	  if(isReady(initiator)){
//		imReady = 1;
//		break;
//	  }
//	  halt(0.5);
//	}
//  }
//  //all robots will go here.
//  if(imReady){
//	for(i = 1; i < rbCount1; i++){
//	  if(activeRb[i] == rbID){
//		halt(i*4 + 3);
//	  }
//	}
//	if(rbID == activeRb[2]){
//	  halt(2);
//	}	
//	if(rbID == activeRb[3]){
//	  halt(3);
//	}
//	demoPathPlan();
//	runSelf(obsStart.i,obsStart.j);
//	runSelf(obsEnd.i,obsEnd.j);
//	if((activeRb[0]) == rbID){
//	  runSelf(obsEnd.i - 4,obsEnd.j);
//	}else if((activeRb[1]) == rbID){
//	  runSelf(obsEnd.i - 2,obsEnd.j);
//	  isStop = 1;
//	}else if((activeRb[2]) == rbID){
//	  runSelf(obsEnd.i + 2,obsEnd.j);
//	  isStop = 1;
//	}else if((activeRb[3]) == rbID){
//	  runSelf(obsEnd.i,obsEnd.j);
//	}
//	halt(10000);
//  }
//}
//
//void mainTask1(){
//  u8 i = 0;
//  u8 leader = 0;
//  u8 initiator = 0;
//  
//  initiator = electInitiator();
//  while(!initiator){
//	initiator = electInitiator();
//	halt(0.5);
//  }
//  
//  if(rbID == initiator){
//	halt(1);
//	leader = electLeader(initiator);
//	while(!leader){
//	  leader =  electLeader(initiator);
//	  halt(0.5);
//	}
//	isLeaderOK = 1;
//	halt(0.5);
//	if(rbCount1 >= 2){
//	  while(1){
//		u8 k = 0;
//		for(i = 0; i < rbCount1; i++){
//		  if(bCastInfo[activeRb[i] - 1].nodeID == rbID)
//			continue;
//		  if(bCastInfo[activeRb[i] - 1].isReady == 1){
//			k++;
//		  }
//		}
//		if(k != rbCount1 - 1){
//		  halt(0.5);
//		}else{
//		  imReady = 1;
//		  halt(0.5);
//		  break;
//		}
//	  }
//	}else{
//	  imReady = 1;
//	  halt(0.5);
//	}
//  }else{  //not initiator
//	halt(1);
//	while(bCastInfo[initiator - 1].isLeaderOK != 1){
//	  halt(0.5);
//	}
//	while(!followerInit(initiator)){
//	  halt(0.5);
//	}
//	imReady = 1;
//	halt(0.5);
//	
//	while(1){
//	  if(isReady(initiator)){
//		imReady = 1;
//		break;
//	  }
//	  halt(0.5);
//	}
//  }
//  //all robots will go here.
//  if(imReady){	
//	for(i = 1; i < rbCount1; i++){
//	  if(activeRb[i] == rbID){
//		halt(i*4 + 3);
//	  }
//	}
//	if(rbID == activeRb[2]){
//	  halt(2);
//	}	
//	if(rbID == activeRb[3]){
//	  halt(3);
//	}
//	demoPathPlan();
//	runSelf(obsStart.i,obsStart.j);
//	runSelf(obsEnd.i,obsEnd.j);
//	if((activeRb[0]) == rbID){
//	  runSelf(obsEnd.i - 4,obsEnd.j);
//	}else if((activeRb[1]) == rbID){
//	  runSelf(obsEnd.i - 2,obsEnd.j);
//	  isStop = 1;
//	}else if((activeRb[2]) == rbID){
//	  runSelf(obsEnd.i + 2,obsEnd.j);
//	  isStop = 1;
//	}else if((activeRb[3]) == rbID){
//	  runSelf(obsEnd.i,obsEnd.j);
//	}
//	halt(10000);
//  }
//}
//
//void mainTask2(){
//  u8 i = 0;
//  u8 leader = 0;
//  u8 initiator = 0;
//  
//  initiator = electInitiator();
//  while(!initiator){
//	initiator = electInitiator();
//	halt(0.5);
//  }
//  
//  if(rbID == initiator){
//	halt(1);
//	leader = electLeader(initiator);
//	while(!leader){
//	  leader =  electLeader(initiator);
//	  halt(0.5);
//	}
//	isLeaderOK = 1;
//	halt(0.5);
//	if(rbCount1 >= 2){
//	  while(1){
//		u8 k = 0;
//		for(i = 0; i < rbCount1; i++){
//		  if(bCastInfo[activeRb[i] - 1].nodeID == rbID)
//			continue;
//		  if(bCastInfo[activeRb[i] - 1].isReady == 1){
//			k++;
//		  }
//		}
//		if(k != rbCount1 - 1){
//		  halt(0.5);
//		}else{
//		  imReady = 1;
//		  halt(0.5);
//		  break;
//		}
//	  }
//	}else{
//	  imReady = 1;
//	  halt(0.5);
//	}
//  }else{  //not initiator
//	halt(1);
//	while(bCastInfo[initiator - 1].isLeaderOK != 1){
//	  halt(0.5);
//	}
//	while(!followerInit(initiator)){
//	  halt(0.5);
//	}
//	imReady = 1;
//	halt(0.5);
//	
//	while(1){
//	  if(isReady(initiator)){
//		imReady = 1;
//		break;
//	  }
//	  halt(0.5);
//	}
//  }
//  //all robots will go here.
//  if(imReady){
//	/**************************************/
//  RETRY:	if(pathFinding()){
//	imFoundPath = 1;
//	halt(0.5);
//	while(1){
//	  u8 k = 0;
//	  for(i = 0; i < rbCount1; i++){
//		if(bCastInfo[activeRb[i] - 1].isFoundPath){
//		  k++;
//		}
//	  }
//	  if(rbCount1 == k){
//		break;
//	  }else{
//		halt(0.5);
//	  }
//	}
//	/**************************************/	
//	for(i = 1; i < rbCount1; i++){
//	  if(activeRb[i] == rbID){
//		halt(i*4 + 3);
//	  }
//	}
//	if(rbID == activeRb[2]){
//	  halt(2);
//	}	
//	if(rbID == activeRb[3]){
//	  halt(3);
//	}
//	for(i = 1 ;i < turnPoint; i++){
//	  runSelf(wayArr[i].i, wayArr[i].j);
//	}
//	if((activeRb[0]) == rbID){
//	  runSelf(obsEnd.i - 4,obsEnd.j);
//	}else if((activeRb[1]) == rbID){
//	  runSelf(obsEnd.i - 2,obsEnd.j);
//	  isStop = 1;
//	}else if((activeRb[2]) == rbID){
//	  runSelf(obsEnd.i + 2,obsEnd.j);
//	  isStop = 1;
//	}else if((activeRb[3]) == rbID){
//	  runSelf(obsEnd.i,obsEnd.j);
//	}
//	halt(10000);
//  }else{
//	halt(1); //path not found
//	SetLeftWheelGivenSpeed(30);
//	SetRightWheelGivenSpeed(30); 
//	vTaskDelay(300);
//	goto RETRY;
//  }
//  }
//}
//
//void vDistributedTask( void *pvParameters ){
//  while(1){
//#ifdef VISIT_DEMO	
//	mainDemo();
//#endif
//
//#ifdef MAIN_DEMO1
//	mainTask1();
//#endif
//	
//#ifdef MAIN_DEMO2
//	mainTask2();
//#endif
//	halt(5000);
//  }
//}
