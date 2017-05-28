#ifndef __HALMPUINIT_H
#define __HALMPUINIT_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"

#include "stm32f10x.h"
#include "sysConfig.h"
typedef struct typeMPUSensor{
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float pitch;  //翻转
  float roll;   //滚动
  float yaw;    //俯仰
  u32   timeStamp;
}typeMPUSensor;

typedef struct typeMagSensor{
  float magX;
  float magY;
  float magZ;
  u32   timeStamp;
}typeMagSensor;


#define q30  (1073741824.0f)

extern u8 halMPUInit(void);
extern typeMPUSensor halReadMPUSensor(void);
extern typeMagSensor halReadMagSensor(void);

#endif

