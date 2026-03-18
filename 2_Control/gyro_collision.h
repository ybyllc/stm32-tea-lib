// 陀螺仪碰撞检测算法
#ifndef GYRO_COLLISION_H
#define GYRO_COLLISION_H

#include "stdint.h"
#include "math.h"   // 只用 ABS 宏，可自己写
#include "gpio.h"
#include "common.h"

void CDK_Update(int16_t *accRaw, int16_t *gyroRaw);
void CDK_Callback(void);

// 姿态滤波（控制层算法）
void Gyro_Filter_Reset(void);
void Gyro_Filter_Update(float *acc_g, float *gyro_dps, float *angle_deg, float *yaw_deg);
uint8_t Gyro_Filter_IsReady(void);
	
#endif
