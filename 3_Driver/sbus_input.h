#ifndef __SBUS_INPUT_H
#define __SBUS_INPUT_H

#include "common.h"

/**
 * @file sbus_input.h
 * @brief SBUS信号输入解码驱动（使用USART1， PA10 RX）
 *
 * ==================== 协议说明 ====================
 * SBUS（Serial Bus）协议：
 * - 波特率: 100000
 * - 格式  : 8E2（8数据位，偶校验，2停止位）
 * - 帧长  : 25字节
 * - 帧头  : 0x0F
 * - 帧尾  : 0x00
 * - 通道数: 16个通道 + 2个数字通道
 *
 * ==================== 硬件接线 ====================
 * USART1_RX (PA10) -> SBUS信号输入
 * GND            -> 对端设备GND
 *
 * ==================== 重要注意 ====================
 * - SBUS常见为"反相串口"电平
 * - 若接收机输出为反相，MCU侧需：
 *   使用硬件反相功能（STM32F4支持UART_TXPIN/RXPIN反相）
 *   或外部反相电路
 * - SBUS输入和SBUS输出共用USART1（PA9/PA10），不能同时使用
 * - 进入SBUS输入测试页面时自动初始化输入模式
 * - 退出页面时需要反初始化
 *
 * ==================== CubeMX配置步骤 ====================
 * 1. USART1配置:
 *    - Mode: Asynchronous
 *    - Baud Rate: 100000
 *    - Word Length: 8 Bits
 *    - Parity: Even
 *    - Stop Bits: 2
 *    - Data Direction: Receive Only（输入模式）
 *    - 勾选"Overrun"（用于DMA或中断接收）
 *
 * 2. NVIC中断：
 *    - USART1 global interrupt
 *
 * 3. 在 HAL_UART_RxCpltCallback 中转发：
 *    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *    {
 *        SBUS_In_UART_RxCpltCallback(huart);
 *    }
 *
 * 4. 在 HAL_UART_ErrorCallback 中转发：
 *    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 *    {
 *        SBUS_In_UART_ErrorCallback(huart);
 *    }
 *
 * ==================== 使用方式 ====================
 * 1. 上电初始化后调用 SBUS_In_Init()
 * 2. 周期读取：
 *    SBUS_InFrameTypeDef frame;
 *    if (SBUS_In_GetFrame(&frame))
 *    {
 *        // frame.channels[0~15] 单位: SBUS原始值(172~1811)
 *        // 使用 SBUS_In_ChannelToPwm() 转换为 1000~2000us
 *    }
 */

#include "common.h"


/************************* 用户配置区 *************************/
// 使用USART1（PA9 TX, PA10 RX）
// 注意：需要在CubeMX中配置USART1为 100000,8E2
extern UART_HandleTypeDef huart1;
#define SBUS_IN_UART_HANDLE          huart1
/**************************************************************/

#define SBUS_FRAME_LEN               25U     // SBUS帧长度
#define SBUS_CHANNEL_COUNT           16U    // 通道数量
#define SBUS_TIMEOUT_MS              50U     // 信号超时时间

#define SBUS_HEADER_BYTE             0x0FU   // 帧头
#define SBUS_END_BYTE                0x00U   // 帧尾

// 标志位定义
#define SBUS_FLAG_CH17               0x01U   // CH17数字通道
#define SBUS_FLAG_CH18               0x02U   // CH18数字通道
#define SBUS_FLAG_FRAME_LOST         0x04U   // 信号丢失标志
#define SBUS_FLAG_FAILSAFE           0x08U   // 故障保护标志
// 通道值范围
#define SBUS_VALUE_MIN               172U    // 通道最小值
#define SBUS_VALUE_MAX               1811U   // 通道最大值
#define SBUS_VALUE_CENTER            992U    // 通道中值

/**
 * @brief SBUS帧数据结构
 */
typedef struct
{
    u16 channels[SBUS_CHANNEL_COUNT];  // 16个通道值（172~1811）
    u8 ch17;                        // CH17数字通道状态
    u8 ch18;                        // CH18数字通道状态
    u8 frame_lost;                  // 信号丢失标志
    u8 failsafe;                    // 故障保护标志
} SBUS_InFrameTypeDef;
/**
 * @brief 初始化SBUS输入驱动
 */
void SBUS_In_Init(void);
/**
 * @brief 在HAL_UART_RxCpltCallback中转发调用
 */
void SBUS_In_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/**
 * @brief 在HAL_UART_ErrorCallback中转发调用
 */
void SBUS_In_UART_ErrorCallback(UART_HandleTypeDef *huart);
/**
 * @brief 读取一帧SBUS数据
 */
u8 SBUS_In_GetFrame(SBUS_InFrameTypeDef *frame);
/**
 * @brief 读取指定通道值
 */
u8 SBUS_In_GetChannel(u8 channel_index, u16 *value);
/**
 * @brief 判断SBUS信号是否在线
 */
u8 SBUS_In_IsOnline(void);
/**
 * @brief 将通道值（172~1811）转换为脉宽（1000~2000us）
 */
u16 SBUS_In_ChannelToPwm(u16 channel_value);
/**
 * @brief 将脉宽（1000~2000us）转换为通道值（172~1811）
 */
u16 SBUS_In_PwmToChannel(u16 pulse_us);
#endif /* __SBUS_INPUT_H */
