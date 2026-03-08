// 陀螺仪碰撞检测算法
#ifndef GYRO_COLLISION_H
#define GYRO_COLLISION_H

#include "stdint.h"
#include "math.h"   // 只用 ABS 宏，可自己写
#include "gpio.h"

void CDK_Update(int16_t *accRaw, int16_t *gyroRaw);
void CDK_Callback(void);
	
#endif
