#ifndef __SBUS_OUTPUT_H
#define __SBUS_OUTPUT_H

/**
 * @file sbus_output.h
 * @brief SBUS信号输出编码驱动（使用USART1， PA9 TX）
 *
 * ==================== 协议说明 ====================
 * SBUS（Serial Bus）协议：
 * - 波特率: 100000
 * - 格式  : 8E2（8数据位， 偶校验，2停止位）
 * - 帧长  : 25字节
 * - 帧头  : 0x0F
 * - 帧尾  : 0x00
 *
 * ==================== 硬件接线 ====================
 * USART1_TX (PA9) -> SBUS信号输出
 * GND            -> 对端设备GND
 *
 * ==================== 重要注意 ====================
 * - SBUS常见为"反相串口"电平
 * - 若对端要求反相，需要：
 *   a) 使用UART硬件反相功能（STM32F4支持）
 *   b) 或外部反相电路
 * - SBUS输入和输出共用USART1（PA9/PA10），不能同时使用
 *
 * ==================== CubeMX配置步骤 ====================
 * 1. 配置USART1：
 *    - Mode: Asynchronous
 *    - Baud Rate: 100000
 *    - Word Length: 8 Bits
 *    - Parity: Even
 *    - Stop Bits: 2
 *    - Data Direction: Transmit Only（仅输出模式）
 *
 * 2. 在 HAL_UART_TxCpltCallback 中转发：
 *    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 *    {
 *        SBUS_Out_UART_TxCpltCallback(huart);
 *    }
 *
 * ==================== 使用方式 ====================
 * 1. 上电初始化后调用 SBUS_Out_Init()
 * 2. 使用 SBUS_Out_SetChannels() 或 SBUS_Out_SetChannel() 设置通道值
 * 3. 周期调用 SBUS_Out_SendFrame() 发送数据
 * 4. 帧周期建议：7~14ms
 *
 * ==================== 通道值说明 ====================
 * - 通道值范围: 172~1811（中位992）
 * - PWM脉宽范围: 1000~2000us（中位1500us）
 * - 使用 SBUS_Out_PwmToChannel() 进行转换
 */

#include "common.h"

/************************* 用户配置区 *************************/
// 使用USART1（PA9 TX, PA10 RX）
// 注意：需要在CubeMX中配置USART1为 100000,8E2
extern UART_HandleTypeDef huart1;
#define SBUS_OUT_UART_HANDLE         huart1
#define SBUS_OUT_TX_TIMEOUT_MS       5U
/**************************************************************/

// 帧长度
#define SBUS_OUT_FRAME_LEN           25U
// 通道数量
#define SBUS_OUT_CHANNEL_COUNT       16U

// 通道值范围
#define SBUS_OUT_VALUE_MIN           172U    // 最小值
#define SBUS_OUT_VALUE_MAX           1811U   // 最大值
#define SBUS_OUT_VALUE_CENTER        992U    // 中值

// 标志位定义
#define SBUS_OUT_FLAG_CH17           0x01U
#define SBUS_OUT_FLAG_CH18           0x02U
#define SBUS_OUT_FLAG_FRAME_LOST     0x04U
#define SBUS_OUT_FLAG_FAILSAFE       0x08U

/**
 * @brief SBUS输出帧结构体
 */
typedef struct
{
    u16 channels[SBUS_OUT_CHANNEL_COUNT];  // 16个通道值
    u8 ch17;                         // CH17数字通道
    u8 ch18;                         // CH18数字通道
    u8 frame_lost;                   // 信号丢失标志
    u8 failsafe;                     // 故障保护标志
} SBUS_OutFrameTypeDef;

/**
 * @brief 初始化SBUS输出驱动
 */
void SBUS_Out_Init(void);

/**
 * @brief 设置所有通道值
 * @param channels 通道值数组（172~1811）
 * @param channel_count 通道数量
 * @retval 1=成功, 0=失败
 */
u8 SBUS_Out_SetChannels(const u16 *channels, u8 channel_count);

/**
 * @brief 设置指定通道值
 * @param channel_index 通道索引（0~15）
 * @param value 通道值（172~1811）
 * @retval 1=成功, 0=失败
 */
u8 SBUS_Out_SetChannel(u8 channel_index, u16 value);

/**
 * @brief 设置标志位
 * @param ch17 CH17数字通道状态
 * @param ch18 CH18数字通道状态
 * @param frame_lost 信号丢失标志
 * @param failsafe 故障保护标志
 */
void SBUS_Out_SetFlags(u8 ch17, u8 ch18, u8 frame_lost, u8 failsafe);

/**
 * @brief 发送一帧SBUS数据
 * @retval HAL状态
 */
HAL_StatusTypeDef SBUS_Out_SendFrame(void);

/**
 * @brief 获取当前通道值
 * @param channels 输出缓冲区
 * @param channel_count 返回通道数量
 * @retval 1=成功, 0=失败
 */
u8 SBUS_Out_GetChannels(u16 *channels, u8 *channel_count);

/**
 * @brief 将脉宽（1000~2000us）转换为通道值（172~1811）
 * @param pulse_us 脉宽值（us）
 * @retval 通道值
 */
u16 SBUS_Out_PwmToChannel(u16 pulse_us);

/**
 * @brief 将通道值（172~1811）转换为脉宽（1000~2000us）
 * @param channel_value 通道值
 * @retval 脉宽值（us）
 */
u16 SBUS_Out_ChannelToPwm(u16 channel_value);

/**
 * @brief UART发送完成回调（在HAL_UART_TxCpltCallback中调用）
 * @param huart UART句柄
 */
void SBUS_Out_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* __SBUS_OUTPUT_H */
