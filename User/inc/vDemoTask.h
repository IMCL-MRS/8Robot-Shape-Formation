#ifndef __VDEMOTASK_H
#define __VDEMOTASK_H

#include "FreeRTOS.h"
#include "task.h"

#include "led.h"
#include "vTasks.h"
#include "apps.h"
#include "stdbool.h"

#include "vInfoList.h"

#define M_PI  3.14159265358979323846
#define SLOW  100
#define STOP   40

extern void vDemoOneTask( void *pvParameters );
extern void vDistributedTask( void *pvParameters );
extern void neighborsTask(void* parameters);
extern void vSnakeTask(void* parameters);
extern void vStackCheck(void* parameters);
extern void vExampleTask1(void *pvParameters);
extern void vBCastMagTask(void *pvParameters);

void halt(float time);


#endif
