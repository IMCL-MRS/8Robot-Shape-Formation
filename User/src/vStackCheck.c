#include "vDemoTask.h"
#include "task.h"
#include "vInfoList.h"

void vStackCheck( void * pvParameters )
{
 static UBaseType_t uxHighWaterMark;
  
  /* Inspect our own high water mark on entering the task. */
  uxHighWaterMark = uxTaskGetStackHighWaterMark( (void *)vSnakeTask );
//  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
//  uxHighWaterMark = uxTaskGetStackHighWaterMark( (void *)vBCastInfoTask );
  for( ;; )
  {
	/* Call any function. */
	vTaskDelay( 1000 );
	
	/* Calling the function will have used some stack space, we would 
	therefore now expect uxTaskGetStackHighWaterMark() to return a 
	value lower than when it was called on entering the task. */
	uxHighWaterMark = uxTaskGetStackHighWaterMark( (void *)vSnakeTask );
//	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
//	uxHighWaterMark = uxTaskGetStackHighWaterMark( (void *)vBCastInfoTask );
	vTaskDelay(uxHighWaterMark);
  }
}