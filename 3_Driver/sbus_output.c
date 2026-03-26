/**
 * @file sbus_output.c
 * @brief SBUS信号输出编码驱动实现
 */

#include "sbus_output.h"
#include "string.h"

// 帧头和帧尾字节
#define SBUS_HEADER_BYTE             0x0FU
#define SBUS_END_BYTE                0x00U

// 发送缓冲区
static volatile SBUS_OutFrameTypeDef s_tx_frame;
static u8 s_raw_frame[SBUS_OUT_FRAME_LEN];
static volatile u8 s_initialized = 0U;

/**
 * @brief 限制通道值范围
 */
static u16 SBUS_Out_ClampValue(u16 value)
{
    if (value < SBUS_OUT_VALUE_MIN)
    {
        return SBUS_OUT_VALUE_MIN;
    }
    
    if (value > SBUS_OUT_VALUE_MAX)
    {
        return SBUS_OUT_VALUE_MAX;
    }
    
    return value;
}

/**
 * @brief 构建原始帧数据
 */
static void SBUS_Out_BuildRawFrame(void)
{
    u8 ch;
    u8 bit;
    u16 bit_index = 0U;
    
    // 清空帧缓冲区
    memset(s_raw_frame, 0, SBUS_OUT_FRAME_LEN);
    
    // 设置帧头
    s_raw_frame[0] = SBUS_HEADER_BYTE;
    
    // 编码16个通道（每个通道11位）
    for (ch = 0U; ch < SBUS_OUT_CHANNEL_COUNT; ch++)
    {
        u16 value = s_tx_frame.channels[ch] & 0x07FFU;
        
        for (bit = 0U; bit < 11U; bit++)
        {
            if (value & (u16)(1U << bit))
            {
                u16 byte_index = (u16)(1U + (bit_index >> 3));
                u8 bit_offset = (u8)(bit_index & 0x07U);
                s_raw_frame[byte_index] |= (u8)(1U << bit_offset);
            }
            
            bit_index++;
        }
    }
    
    // 设置标志字节（第24字节）
    s_raw_frame[23] = 0U;
    if (s_tx_frame.ch17)
    {
        s_raw_frame[23] |= SBUS_OUT_FLAG_CH17;
    }
    if (s_tx_frame.ch18)
    {
        s_raw_frame[23] |= SBUS_OUT_FLAG_CH18;
    }
    if (s_tx_frame.frame_lost)
    {
        s_raw_frame[23] |= SBUS_OUT_FLAG_FRAME_LOST;
    }
    if (s_tx_frame.failsafe)
    {
        s_raw_frame[23] |= SBUS_OUT_FLAG_FAILSAFE;
    }
    
    // 设置帧尾
    s_raw_frame[24] = SBUS_END_BYTE;
}

/**
 * @brief 初始化SBUS输出驱动
 */
void SBUS_Out_Init(void)
{
    u8 ch;
    
    // 初始化所有通道为中位值
    for (ch = 0U; ch < SBUS_OUT_CHANNEL_COUNT; ch++)
    {
        s_tx_frame.channels[ch] = SBUS_OUT_VALUE_CENTER;
    }
    
    s_tx_frame.ch17 = 0U;
    s_tx_frame.ch18 = 0U;
    s_tx_frame.frame_lost = 0U;
    s_tx_frame.failsafe = 0U;
    
    // 构建初始帧
    SBUS_Out_BuildRawFrame();
    
    s_initialized = 1U;
}

/**
 * @brief 设置所有通道值
 */
u8 SBUS_Out_SetChannels(const u16 *channels, u8 channel_count)
{
    u8 ch;
    
    if ((channels == 0) || (channel_count == 0U) || (channel_count > SBUS_OUT_CHANNEL_COUNT))
    {
        return 0U;
    }
    
    for (ch = 0U; ch < channel_count; ch++)
    {
        s_tx_frame.channels[ch] = SBUS_Out_ClampValue(channels[ch]);
    }
    
    // 重新构建帧
    SBUS_Out_BuildRawFrame();
    
    return 1U;
}

/**
 * @brief 设置指定通道值
 */
u8 SBUS_Out_SetChannel(u8 channel_index, u16 value)
{
    if (channel_index >= SBUS_OUT_CHANNEL_COUNT)
    {
        return 0U;
    }
    
    s_tx_frame.channels[channel_index] = SBUS_Out_ClampValue(value);
    
    // 重新构建帧
    SBUS_Out_BuildRawFrame();
    
    return 1U;
}

/**
 * @brief 设置标志位
 */
void SBUS_Out_SetFlags(u8 ch17, u8 ch18, u8 frame_lost, u8 failsafe)
{
    s_tx_frame.ch17 = ch17 ? 1U : 0U;
    s_tx_frame.ch18 = ch18 ? 1U : 0U;
    s_tx_frame.frame_lost = frame_lost ? 1U : 0U;
    s_tx_frame.failsafe = failsafe ? 1U : 0U;
    
    // 重新构建帧
    SBUS_Out_BuildRawFrame();
}

/**
 * @brief 发送一帧SBUS数据
 */
HAL_StatusTypeDef SBUS_Out_SendFrame(void)
{
    if (s_initialized == 0U)
    {
        return HAL_ERROR;
    }
    
    return HAL_UART_Transmit(&SBUS_OUT_UART_HANDLE, s_raw_frame, SBUS_OUT_FRAME_LEN, SBUS_OUT_TX_TIMEOUT_MS);
}

/**
 * @brief 获取当前通道值
 */
u8 SBUS_Out_GetChannels(u16 *channels, u8 *channel_count)
{
    u8 ch;
    
    if ((channels == 0) || (channel_count == 0))
    {
        return 0U;
    }
    
    *channel_count = SBUS_OUT_CHANNEL_COUNT;
    
    for (ch = 0U; ch < SBUS_OUT_CHANNEL_COUNT; ch++)
    {
        channels[ch] = s_tx_frame.channels[ch];
    }
    
    return 1U;
}

/**
 * @brief 将脉宽（1000~2000us）转换为通道值（172~1811）
 */
u16 SBUS_Out_PwmToChannel(u16 pulse_us)
{
    // 反向转换: Channel = 172 + (Pulse - 1000) * 1639 / 1000
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    return (u16)(172 + (u32)(pulse_us - 1000) * 1639 / 1000);
}

/**
 * @brief 将通道值（172~1811）转换为脉宽（1000~2000us）
 */
u16 SBUS_Out_ChannelToPwm(u16 channel_value)
{
    // SBUS值范围: 172~1811 (中位992)
    // PWM值范围: 1000~2000us (中位1500us)
    // 公式: PWM = 1000 + (SBUS - 172) * 1000 / (1811 - 172)
    // 简化: PWM = 1000 + (SBUS - 172) * 1000 / 1639
    return (u16)(1000 + (int32_t)(channel_value - 172) * 1000 / 1639);
}

/**
 * @brief UART发送完成回调
 */
void SBUS_Out_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 可以在这里添加发送完成后的处理逻辑
    // 例如：启动下一次发送、或者更新统计信息等
}
