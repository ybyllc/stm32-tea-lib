/**
 * @file menu_input.h
 * @brief 菜单输入处理接口
 * @note 负责菜单系统的输入逻辑，包括编码器、按键和遥控器输入
 */

#ifndef MENU_INPUT_H
#define MENU_INPUT_H

#include "menu.h"

/**
 * @brief 输入处理初始化
 */
void MenuInput_Init(void);

/**
 * @brief 处理编码器输入
 * @param count 编码器计数值
 */
void MenuInput_HandleEncoder(int32_t count);

/**
 * @brief 处理按键事件
 * @param event 按键事件：1-短按，2-长按
 */
void MenuInput_HandleKeyEvent(uint8_t event);

/**
 * @brief 扫描遥控器按键状态
 */
void MenuInput_ScanRemoteKey(void);

/**
 * @brief 获取编码器计数值
 * @return 编码器计数值
 */
int32_t MenuInput_GetEncoderCount(void);

/**
 * @brief 清除编码器计数
 */
void MenuInput_ClearEncoderCount(void);

/**
 * @brief 获取编码器按键状态
 * @return 按键状态：1-释放，0-按下
 */
uint8_t MenuInput_GetEncoderKeyState(void);

/**
 * @brief 获取编码器按键事件
 * @return 按键事件：0-无事件，1-短按，2-长按
 */
uint8_t MenuInput_GetEncoderKeyEvent(void);

#endif /* MENU_INPUT_H */
