#include "beep.h"
#include "vDemoTask.h"

void BeepInit(void){
  halBeepInit();
}
void SetBeepStatus(u8 beepStatus){
   halSetBeepStatus(beepStatus);
}

void beepSing(){
  	SetBeepStatus(BEEP_ON);
	vTaskDelay(300);
	SetBeepStatus(BEEP_OFF);
}

