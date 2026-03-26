#include "sbus_input.h"

// 帧头字节
#define SBUS_HEADER_BYTE             0x0FU

// 接收缓冲区
static volatile u8 s_rx_byte = 0U;
static volatile u8 s_rx_buf[SBUS_FRAME_LEN];
static volatile u8 s_rx_index = 0U;
static volatile u8 s_rx_state = 0U; // 0:空闲, 1:等待帧头

// 帧数据
static volatile SBUS_InFrameTypeDef s_last_frame;
static volatile u8 s_has_frame = 0U;
static volatile u32 s_last_frame_tick = 0U;
static volatile u8 s_initialized = 0U;

// 函数声明
static void SBUS_In_StartReceive(void);
static void SBUS_In_DecodeFrame(void);
void SBUS_In_UART_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief 启动单字节接收
 */
static void SBUS_In_StartReceive(void)
{
    HAL_UART_Receive_IT(&SBUS_IN_UART_HANDLE, (u8 *)&s_rx_byte, 1);
}
/**
 * @brief 初始化SBUS输入驱动
 */
void SBUS_In_Init(void)
{
    u8 ch;
    
    s_rx_index = 0U;
    s_rx_state = 0U;
    s_has_frame = 0U;
    s_initialized = 1U;
    
    // 初始化通道值为中值
    for (ch = 0U; ch < SBUS_CHANNEL_COUNT; ch++)
    {
        s_last_frame.channels[ch] = SBUS_VALUE_CENTER; // 992
    }
    
    s_last_frame.ch17 = 0U;
    s_last_frame.ch18 = 0U;
    s_last_frame.frame_lost = 0U;
    s_last_frame.failsafe = 0U;
    
    // 启动接收
    SBUS_In_StartReceive();
}
/**
 * @brief UART接收完成回调
 */
void SBUS_In_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &SBUS_IN_UART_HANDLE)
    {
        return;
    }
    
    if (s_initialized == 0U)
    {
        return;
    }
    
    if (s_rx_state == 0U)
    {
        // 等待帧头
        if (s_rx_byte == SBUS_HEADER_BYTE)
        {
            s_rx_state = 1U;
            s_rx_buf[0] = SBUS_HEADER_BYTE;
            s_rx_index = 1U;
        }
    }
    else
    {
        // 接收数据
        s_rx_buf[s_rx_index] = s_rx_byte;
        s_rx_index++;
        
        if (s_rx_index >= SBUS_FRAME_LEN)
        {
            // 帧接收完成，解析帧数据
            SBUS_In_DecodeFrame();
            s_rx_state = 0U;
            s_rx_index = 0U;
        }
    }
    
    // 继续接收下一个字节
    SBUS_In_StartReceive();
}
/**
 * @brief UART错误回调
 */
void SBUS_In_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &SBUS_IN_UART_HANDLE)
    {
        return;
    }
    
    // 重新启动接收
    s_rx_state = 0U;
    s_rx_index = 0U;
    SBUS_In_StartReceive();
}
/**
 * @brief 解析SBUS帧数据
 */
static void SBUS_In_DecodeFrame(void)
{
    u8 ch;
    u8 bit;
    u16 bit_index = 0U;
    u16 value;
    
    // 解析16个通道（每个通道11位）
    for (ch = 0U; ch < SBUS_CHANNEL_COUNT; ch++)
    {
        value = 0U;
        
        for (bit = 0U; bit < 11U; bit++)
        {
            u16 byte_index = (u16)(1U + (bit_index >> 3));
            u8 bit_offset = (u8)(bit_index & 0x07U);
            
            if (s_rx_buf[byte_index] & (u8)(1U << bit_offset))
            {
                value |= (u16)(1U << bit);
            }
            
            bit_index++;
        }
        
        s_last_frame.channels[ch] = value;
    }
    
    // 解析标志字节（第24字节）
    s_last_frame.ch17 = (s_rx_buf[23] & SBUS_FLAG_CH17) ? 1U : 0U;
    s_last_frame.ch18 = (s_rx_buf[23] & SBUS_FLAG_CH18) ? 1U : 0U;
    s_last_frame.frame_lost = (s_rx_buf[23] & SBUS_FLAG_FRAME_LOST) ? 1U : 0U;
    s_last_frame.failsafe = (s_rx_buf[23] & SBUS_FLAG_FAILSAFE) ? 1U : 0U;
    
    s_has_frame = 1U;
    s_last_frame_tick = HAL_GetTick();
}
/**
 * @brief 读取一帧SBUS数据
 */
u8 SBUS_In_GetFrame(SBUS_InFrameTypeDef *frame)
{
    u8 ch;
    u32 primask;
    
    if (frame == 0)
    {
        return 0U;
    }
    
    if ((s_has_frame == 0U) || (SBUS_In_IsOnline() == 0U))
    {
        return 0U;
    }
    
    primask = __get_PRIMASK();
    __disable_irq();
    
    for (ch = 0U; ch < SBUS_CHANNEL_COUNT; ch++)
    {
        frame->channels[ch] = s_last_frame.channels[ch];
    }
    
    frame->ch17 = s_last_frame.ch17;
    frame->ch18 = s_last_frame.ch18;
    frame->frame_lost = s_last_frame.frame_lost;
    frame->failsafe = s_last_frame.failsafe;
    
    if (primask == 0U)
    {
        __enable_irq();
    }
    
    return 1U;
}
/**
 * @brief 读取指定通道值
 */
u8 SBUS_In_GetChannel(u8 channel_index, u16 *value)
{
    if (channel_index >= SBUS_CHANNEL_COUNT)
    {
        *value = SBUS_VALUE_CENTER;
        return 0U;
    }
    
    if (SBUS_In_IsOnline() == 0U)
    {
        *value = SBUS_VALUE_CENTER;
        return 0U;
    }
    
    *value = s_last_frame.channels[channel_index];
    return 1U;
}
/**
 * @brief 判断SBUS信号是否在线
 */
u8 SBUS_In_IsOnline(void)
{
    if (s_initialized == 0U)
    {
        return 0U;
    }
    
    if (s_has_frame == 0U)
    {
        return 0U;
    }
    
    if ((HAL_GetTick() - s_last_frame_tick) > SBUS_TIMEOUT_MS)
    {
        return 0U;
    }
    
    return 1U;
}
/**
 * @brief 将通道值（172~1811）转换为脉宽（1000~2000us）
 */
u16 SBUS_In_ChannelToPwm(u16 channel_value)
{
    // SBUS值范围: 172~1811 (中位992)
    // PWM值范围: 1000~2000us (中位1500us)
    // 公式: PWM = 1000 + (SBUS - 172) * 1000 / (1811 - 172)
    // 简化: PWM = 1000 + (SBUS - 172) * 1000 / 1639
    return (u16)(1000 + (int32_t)(channel_value - 172) * 1000 / 1639);
}
/**
 * @brief 将脉宽（1000~2000us）转换为通道值（172~1811)
 */
u16 SBUS_In_PwmToChannel(u16 pulse_us)
{
    // 反向转换: SBUS = 172 + (PWM - 1000) * 1639 / 1000
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    return (u16)(172 + (int32_t)(pulse_us - 1000) * 1639 / 1000);
}
