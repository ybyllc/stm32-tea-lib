/**
 * @file car_task.h
 * @brief 小车控制任务头文件
 * @note 声明小车控制任务的相关函数和变量
 */

#ifndef __CAR_TASK_H
#define __CAR_TASK_H

#include "common.h"

/**
 * @brief 小车任务模式
 */
typedef enum {
    CAR_TASK_MODE_STRAIGHT = 0,   // 原直行任务
    CAR_TASK_MODE_FIELD_SCAN,     // 水田弓字遍历任务
    CAR_TASK_MODE_REMOTE          // PS2 远程控制验证任务
} CarTaskModeTypeDef;

/**
 * @brief 小车任务初始化
 * @retval 无
 */
void Car_Task_Init(void);

/**
 * @brief 小车任务开始
 * @retval 无
 */
void Car_Task_Start(void);

/**
 * @brief 小车任务停止
 * @retval 无
 */
void Car_Task_Stop(void);

/**
 * @brief 小车任务主函数
 * @retval 无
 */
void Car_Task_Main(void);

/**
 * @brief 获取小车任务运行状态
 * @retval 运行状态：1-运行中，0-停止
 */
uint8_t Car_Task_IsRunning(void);

/**
 * @brief 设置小车目标角度
 * @param angle 目标角度（度）
 * @retval 无
 */
void Car_Task_SetTargetAngle(float angle);

/**
 * @brief 设置小车最大速度
 * @param speed 最大速度（-300到300）
 * @retval 无
 */
void Car_Task_SetMaxSpeed(int16_t speed);

/**
 * @brief 设置当前准备启动的小车任务模式
 * @param mode 任务模式
 * @retval 无
 */
void Car_Task_SetMode(CarTaskModeTypeDef mode);

/**
 * @brief 获取当前选中的小车任务模式
 * @retval 任务模式
 */
CarTaskModeTypeDef Car_Task_GetMode(void);

#endif /* __CAR_TASK_H */
