#include "apps.h"
#include "vExample.h"

#define TIME_SECONDS  1000
#define TIME_MINUTES (6*TIME_SECOND)

/*1. Motor test for only one task running, calcuate the difference 
 *   between different robots.
 */
void vExampleTask( void *pvParameters){
  halt(3);
  while(1){
    SetLeftWheelGivenSpeed(30);
    SetRightWheelGivenSpeed(30);
    vTaskDelay(TIME_SECONDS*20);
    halt(2000);
  } 
}
