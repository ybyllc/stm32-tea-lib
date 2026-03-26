#ifndef __PPM_INPUT_H
#define __PPM_INPUT_H

/**
 * @file ppm_input.h
 * @brief PPM信号输入解码驱动（默认PB9）
 *
 * ==================== 协议说明 ====================
 * PPM（Pulse Position Modulation）是遥控器常用信号格式
 * - 每帧包含多个通道，通道间用固定间隙分隔
 * - 帧同步间隙 >2.7ms
 * - 通道脉宽范围：750us ~ 2250us（中位1500us）
 *
 * ==================== 硬件接线 ====================
 * PPM信号线 -> PB9（默认）
 * 接收机GND -> MCU GND
 *
 * ==================== CubeMX配置步骤 ====================
 * 1. PB9 配置为 GPIO_EXTI9
 *    - Mode: External Interrupt Mode with Falling edge trigger detection
 *    - Pull: Pull-up（适配常见低脉冲PPM）
 *
 * 2. NVIC中断：
 *    - EXTI line[9:5] interrupt
 *
 * 3. 在 HAL_GPIO_EXTI_Callback 中转发：
 *    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 *    {
 *        PPM_In_EXTI_Callback(GPIO_Pin);
 *    }
 *
 * ==================== 使用方式 ====================
 * 1. 上电初始化后调用 PPM_In_Init()
 * 2. 周期读取：
 *    uint16_t ch[PPM_IN_MAX_CHANNELS];
 *    uint8_t cnt;
 *    if (PPM_In_GetFrame(ch, &cnt))
 *    {
 *        // ch[0]~ch[cnt-1] 单位: us
 *    }
 *
 * ==================== 注意事项 ====================
 * - PPM输入和PPM输出共用PB9， * - 进入PPM输入测试页面时自动初始化输入模式
 * - 退出页面时需要反初始化GPIO
 */

#include "common.h"

/************************* 用户配置区 *************************/
// 引脚配置：默认PB9
#define PPM_IN_GPIO_PORT          GPIOB
#define PPM_IN_GPIO_PIN           GPIO_PIN_9
#define PPM_IN_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
/**************************************************************/

// 中断号（PB9属于EXTI9_5）
#define PPM_IN_EXTI_IRQn           EXTI9_5_IRQn

// 驱动参数
#define PPM_IN_MAX_CHANNELS        8U      // 支持的最大通道数
#define PPM_IN_MIN_CHANNELS        4U      // 判定为有效帧的最小通道数
#define PPM_IN_SYNC_GAP_US         2700U   // 大于该间隔判定为帧同步间隙
#define PPM_IN_PULSE_MIN_US        750U    // 通道脉宽下限（us）
#define PPM_IN_PULSE_MAX_US        2250U   // 通道脉宽上限（us）
#define PPM_IN_SIGNAL_TIMEOUT_MS   100U    // 超过该时间无新帧判定离线

/**
 * @brief 初始化PPM输入驱动（GPIO中断 + DWT计时）
 */
void PPM_In_Init(void);

/**
 * @brief 反初始化PPM输入（关闭中断，释放GPIO）
 */
void PPM_In_Deinit(void);

/**
 * @brief 在HAL_GPIO_EXTI_Callback中转发调用
 * @param GPIO_Pin 触发中断的GPIO引脚
 */
void PPM_In_EXTI_Callback(uint16_t GPIO_Pin);

/**
 * @brief 读取一帧PPM通道值
 * @param channels 输出缓冲，单位us
 * @param channel_count 返回本帧通道数量
 * @retval 1=读取成功，0=无有效帧或信号离线
 */
u8 PPM_In_GetFrame(u16 *channels, u8 *channel_count);

/**
 * @brief 读取指定通道的脉宽值
 * @param channel_index 通道序号（0起）
 * @param pulse_us 输出脉宽（us）
 * @retval 1=成功，0=失败
 */
u8 PPM_In_GetChannelUs(u8 channel_index, u16 *pulse_us);

/**
 * @brief 判断PPM信号是否在线
 * @retval 1=在线，0=离线
 */
u8 PPM_In_IsOnline(void);

#endif
