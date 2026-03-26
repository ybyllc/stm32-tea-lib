#ifndef __PPM_OUTPUT_H
#define __PPM_OUTPUT_H

/**
 * @file ppm_output.h
 * @brief PPM信号输出驱动（默认PB9）
 *
 * ==================== 协议说明 ====================
 * PPM（Pulse Position Modulation）输出格式：
 * - 每帧包含多个通道，通道间用固定低脉冲间隙分隔
 * - 帧同步间隙 >3.5ms
 * - 通道脉宽范围：1000us ~ 2000us（中位1500us）
 *
 * ==================== 设计说明 ====================
 * 1. 本驱动使用"定时节拍状态机"生成PPM波形，不额外占用硬件定时器通道
 * 2. 需要在 TIM1_100us_Callback 中周期调用 PPM_Out_TIM_Callback()
 * 3. 默认输出引脚 PB9（GPIO推挽输出）
 *
 * ==================== 硬件接线 ====================
 * PB9 -> 对端设备 PPM 输入
 * GND -> 对端设备 GND
 *
 * ==================== 重要注意 ====================
 * - PPM输入和PPM输出共用PB9，不能同时使用
 * - 进入PPM输出测试页面时自动初始化输出模式
 * - 退出页面时需要停止输出
 *
 * ==================== 集成步骤 ====================
 * 1. 在初始化阶段调用 PPM_Out_Init() 和 PPM_Out_Start()
 * 2. 在 TIM1_100us_Callback() 中增加：
 *      PPM_Out_TIM_Callback();
 * 3. 使用 PPM_Out_SetChannels() 或 PPM_Out_SetChannel() 更新通道值
 */

#include "common.h"

/************************* 用户配置区 *************************/
// 引脚配置：默认PB9
#define PPM_OUT_GPIO_PORT          GPIOB
#define PPM_OUT_GPIO_PIN           GPIO_PIN_9
#define PPM_OUT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
/**************************************************************/

// 驱动参数（可按接收端要求调整）
#define PPM_OUT_MAX_CHANNELS         8U      // 最大通道数
#define PPM_OUT_DEFAULT_CHANNELS     8U      // 默认通道数
#define PPM_OUT_CHANNEL_MIN_US       1000U   // 通道脉宽下限（us）
#define PPM_OUT_CHANNEL_MAX_US       2000U   // 通道脉宽上限（us）
#define PPM_OUT_CENTER_US            1500U   // 中位脉宽（us）
#define PPM_OUT_PULSE_LOW_US         300U    // 低脉冲间隙（us）
#define PPM_OUT_FRAME_TARGET_US      22500U  // 目标帧周期（us）
#define PPM_OUT_SYNC_MIN_US          3500U   // 同步间隙最小值（us）
#define PPM_OUT_TICK_US              100U    // 由 TIM1_100us_Callback 决定

/**
 * @brief 初始化PPM输出驱动
 */
void PPM_Out_Init(void);

/**
 * @brief 启动PPM输出
 */
void PPM_Out_Start(void);

/**
 * @brief 停止PPM输出
 */
void PPM_Out_Stop(void);

/**
 * @brief 定时器回调函数（需在TIM1_100us_Callback中调用）
 */
void PPM_Out_TIM_Callback(void);

/**
 * @brief 设置所有通道值
 */
u8 PPM_Out_SetChannels(const u16 *channels, u8 channel_count);

/**
 * @brief 设置单个通道值
 */
u8 PPM_Out_SetChannel(u8 channel_index, u16 pulse_us);

/**
 * @brief 获取所有通道值
 */
u8 PPM_Out_GetChannels(u16 *channels, u8 *channel_count);

/**
 * @brief 判断PPM输出是否正在运行
 */
u8 PPM_Out_IsRunning(void);

#endif /* __PPM_OUTPUT_H */
