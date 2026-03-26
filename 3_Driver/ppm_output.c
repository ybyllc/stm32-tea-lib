#include "ppm_output.h"

#include "string.h"

// 驱动内部状态
static volatile u16 s_channels[PPM_OUT_MAX_CHANNELS];
static volatile u8 s_channel_count = PPM_OUT_DEFAULT_CHANNELS;
static volatile u8 s_running = 0U;
static volatile u8 s_slot_index = 0U;
static volatile u8 s_phase = 0U;  // 0=低脉冲阶段, 1=高电平阶段
static volatile u32 s_phase_remain_us = 0U;
static volatile u32 s_sum_channel_us = 0U;

// 通道脉宽限幅
static u16 PPM_Out_ClampChannelUs(u16 pulse_us)
{
    if (pulse_us < PPM_OUT_CHANNEL_MIN_US)
    {
        return PPM_OUT_CHANNEL_MIN_US;
    }

    if (pulse_us > PPM_OUT_CHANNEL_MAX_US)
    {
        return PPM_OUT_CHANNEL_MAX_US;
    }

    return pulse_us;
}

// 计算当前槽位的高电平保持时间
static u32 PPM_Out_GetHighUsBySlot(u8 slot_index)
{
    u32 slot_total_us;

    // 最后一个通道之后需要补充同步间隙
    if (slot_index < s_channel_count)
    {
        slot_total_us = s_channels[slot_index];
    }
    else if (slot_index == s_channel_count)
    {
        // 同步间隙 = 目标帧周期 - 所有通道脉宽之和 - 所有低脉冲之和
        slot_total_us = PPM_OUT_FRAME_TARGET_US - s_sum_channel_us 
                        - (u32)(s_channel_count + 1U) * PPM_OUT_PULSE_LOW_US;
        
        // 确保同步间隙不小于最小值
        if (slot_total_us < PPM_OUT_SYNC_MIN_US)
        {
            slot_total_us = PPM_OUT_SYNC_MIN_US;
        }
    }
    else
    {
        // 不应该到达这里
        return PPM_OUT_TICK_US;
    }

    // 减去已经消耗的低脉冲时间
    if (slot_total_us <= PPM_OUT_PULSE_LOW_US)
    {
        return PPM_OUT_TICK_US;
    }

    return slot_total_us - PPM_OUT_PULSE_LOW_US;
}

static void PPM_Out_SetPinLow(void)
{
    HAL_GPIO_WritePin(PPM_OUT_GPIO_PORT, PPM_OUT_GPIO_PIN, GPIO_PIN_RESET);
}
static void PPM_Out_SetPinHigh(void)
{
    HAL_GPIO_WritePin(PPM_OUT_GPIO_PORT, PPM_OUT_GPIO_PIN, GPIO_PIN_SET);
}
void PPM_Out_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    u8 i;

    PPM_OUT_GPIO_CLK_ENABLE();
    gpio_init.Pin = PPM_OUT_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PPM_OUT_GPIO_PORT, &gpio_init);
    // 初始化通道值为中位
    for (i = 0U; i < PPM_OUT_MAX_CHANNELS; i++)
    {
        s_channels[i] = PPM_OUT_CENTER_US;
    }
    s_channel_count = PPM_OUT_DEFAULT_CHANNELS;
    s_sum_channel_us = (u32)PPM_OUT_DEFAULT_CHANNELS * PPM_OUT_CENTER_US;
    s_running = 0U;
    s_slot_index = 0U;
    s_phase = 0U;
    s_phase_remain_us = PPM_OUT_PULSE_LOW_US;
    // 空闲态维持高电平
    PPM_Out_SetPinHigh();
}
void PPM_Out_Start(void)
{
    u32 primask = __get_PRIMASK();
    __disable_irq();
    s_running = 1U;
    s_slot_index = 0U;
    s_phase = 0U;
    s_phase_remain_us = PPM_OUT_PULSE_LOW_US;
    // 起始先输出一个低脉冲起点（第0通道起始边沿）
    PPM_Out_SetPinLow();
    if (primask == 0U)
    {
        __enable_irq();
    }
}
void PPM_Out_Stop(void)
{
    u32 primask = __get_PRIMASK();
    __disable_irq();
    s_running = 0U;
    s_slot_index = 0U;
    s_phase = 0U;
    s_phase_remain_us = PPM_OUT_PULSE_LOW_US;
    // 停止后回到空闲高电平
    PPM_Out_SetPinHigh();
    if (primask == 0U)
    {
        __enable_irq();
    }
}
void PPM_Out_TIM_Callback(void)
{
    if (s_running == 0U)
    {
        return;
    }
    if (s_phase_remain_us > PPM_OUT_TICK_US)
    {
        s_phase_remain_us -= PPM_OUT_TICK_US;
        return;
    }
    if (s_phase == 0U)  // 低脉冲阶段
    {
        // 低脉冲结束，进入高电平保持段
        PPM_Out_SetPinHigh();
        s_phase = 1U;
        s_phase_remain_us = PPM_Out_GetHighUsBySlot(s_slot_index);
    }
    else  // 高电平阶段
    {
        // 高电平段结束，进入下一个槽位的低脉冲
        s_slot_index++;
        if (s_slot_index > s_channel_count)
        {
            s_slot_index = 0U;
        }
        PPM_Out_SetPinLow();
        s_phase = 0U;
        s_phase_remain_us = PPM_OUT_PULSE_LOW_US;
    }
}
u8 PPM_Out_SetChannels(const u16 *channels, u8 channel_count)
{
    u8 i;
    u32 sum_us = 0U;
    u32 primask;

    if ((channels == 0) || (channel_count == 0U) || (channel_count > PPM_OUT_MAX_CHANNELS))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < channel_count; i++)
    {
        s_channels[i] = PPM_Out_ClampChannelUs(channels[i]);
        sum_us += s_channels[i];
    }
    s_channel_count = channel_count;
    s_sum_channel_us = sum_us;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}
u8 PPM_Out_SetChannel(u8 channel_index, u16 pulse_us)
{
    u32 primask;
    u16 old_value;
    u16 new_value;

    if (channel_index >= s_channel_count)
    {
        return 0U;
    }

    new_value = PPM_Out_ClampChannelUs(pulse_us);

    primask = __get_PRIMASK();
    __disable_irq();
    old_value = s_channels[channel_index];
    s_channels[channel_index] = new_value;
    s_sum_channel_us = s_sum_channel_us - old_value + new_value;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}
u8 PPM_Out_GetChannels(u16 *channels, u8 *channel_count)
{
    u8 i;
    u32 primask;

    if ((channels == 0) || (channel_count == 0))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *channel_count = s_channel_count;
    for (i = 0U; i < s_channel_count; i++)
    {
        channels[i] = s_channels[i];
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}
u8 PPM_Out_IsRunning(void)
{
    return s_running;
}
