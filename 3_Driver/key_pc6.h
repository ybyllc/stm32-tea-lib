/**
 * @file key_pc6.h
 * @brief PC6按键驱动接口定义
 * @note 用于检测PC6引脚的按键状态
 */

#ifndef KEY_PC6_H
#define KEY_PC6_H

#include <stdint.h>
#include "common.h"

/**
 * @brief PC6按键初始化
 * @retval 0-失败, 1-成功
 */
uint8_t KeyPC6_Init(void);

/**
 * @brief 获取PC6按键状态
 * @retval 0-按键按下, 1-按键释放
 */
uint8_t KeyPC6_GetState(void);

/**
 * @brief 检测PC6按键事件
 * @retval 0-无事件, 1-短按, 2-长按
 */
uint8_t KeyPC6_CheckEvent(void);

#endif /* KEY_PC6_H */
