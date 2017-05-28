#ifndef __VLOCATIONTASK_H
#define __VLOCATIONTASK_H

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

extern void vLocationTask( void *pvParameters );


#endif

