/**
 * @file ppm_input.c
 * @brief PPM信号输入解码驱动实现
 */

#include "ppm_input.h"

// 已发布到上层的稳定帧数据
static volatile u16 s_channels[PPM_IN_MAX_CHANNELS];
// 当前正在拼帧的数据
static volatile u16 s_work_channels[PPM_IN_MAX_CHANNELS];
static volatile u8 s_channel_count = 0U;
static volatile u8 s_work_index = 0U;
static volatile u8 s_frame_valid = 0U;
static volatile u32 s_last_frame_tick = 0U;
static volatile u32 s_last_edge_us = 0U;
static u32 s_cycles_per_us = 1U;
static volatile u8 s_initialized = 0U;

// 开启DWT周期计数器，用于微秒级时间戳
static void PPM_In_DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;

    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_cycles_per_us == 0U)
    {
        s_cycles_per_us = 1U;
    }
}

// 获取当前时间（us）
static u32 PPM_In_GetTimeUs(void)
{
    return (u32)(DWT->CYCCNT / s_cycles_per_us);
}

// 将当前拼帧结果发布为"最新有效帧"
static void PPM_In_PublishFrame(void)
{
    u8 i;

    for (i = 0U; (i < s_work_index) && (i < PPM_IN_MAX_CHANNELS); i++)
    {
        s_channels[i] = s_work_channels[i];
    }

    s_channel_count = s_work_index;
    s_last_frame_tick = HAL_GetTick();
    s_frame_valid = 1U;
}

/**
 * @brief 初始化PPM输入驱动
 */
void PPM_In_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    u8 i;

    // 1) 配置PPM输入引脚为外部中断
    PPM_IN_GPIO_CLK_ENABLE();

    gpio_init.Pin = PPM_IN_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_IT_FALLING;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PPM_IN_GPIO_PORT, &gpio_init);

    // 2) 使能对应EXTI中断线
    HAL_NVIC_SetPriority(PPM_IN_EXTI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(PPM_IN_EXTI_IRQn);

    // 3) 初始化微秒计时基准
    PPM_In_DwtInit();

    s_work_index = 0U;
    s_channel_count = 0U;
    s_frame_valid = 0U;
    s_last_frame_tick = 0U;
    s_last_edge_us = PPM_In_GetTimeUs();

    // 4) 初始化默认通道为中值，避免上层读取到随机值
    for (i = 0U; i < PPM_IN_MAX_CHANNELS; i++)
    {
        s_channels[i] = 1500U;
        s_work_channels[i] = 1500U;
    }

    s_initialized = 1U;
}

/**
 * @brief 反初始化PPM输入（关闭中断，释放GPIO）
 */
void PPM_In_Deinit(void)
{
    // 禁用中断
    HAL_NVIC_DisableIRQ(PPM_IN_EXTI_IRQn);

    // 关闭GPIO中断模式
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = PPM_IN_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PPM_IN_GPIO_PORT, &gpio_init);

    s_initialized = 0U;
    s_frame_valid = 0U;
}

/**
 * @brief GPIO外部中断回调
 */
void PPM_In_EXTI_Callback(uint16_t GPIO_Pin)
{
    u32 now_us;
    u32 pulse_us;

    if (GPIO_Pin != PPM_IN_GPIO_PIN)
    {
        return;
    }

    now_us = PPM_In_GetTimeUs();
    pulse_us = now_us - s_last_edge_us;
    s_last_edge_us = now_us;

    // 大间隙表示一帧结束，发布上一帧并开始下一帧
    if (pulse_us >= PPM_IN_SYNC_GAP_US)
    {
        if (s_work_index >= PPM_IN_MIN_CHANNELS)
        {
            PPM_In_PublishFrame();
        }

        s_work_index = 0U;
        return;
    }

    // 正常通道脉宽，按顺序写入缓存
    if ((pulse_us >= PPM_IN_PULSE_MIN_US) && (pulse_us <= PPM_IN_PULSE_MAX_US))
    {
        if (s_work_index < PPM_IN_MAX_CHANNELS)
        {
            s_work_channels[s_work_index] = (u16)pulse_us;
            s_work_index++;
        }
        else
        {
            // 通道数溢出，丢弃当前拼帧，等待下一次同步间隙
            s_work_index = 0U;
        }
    }
    else
    {
        // 异常脉宽，丢弃当前拼帧
        s_work_index = 0U;
    }
}

/**
 * @brief 读取一帧PPM通道值
 */
u8 PPM_In_GetFrame(u16 *channels, u8 *channel_count)
{
    u8 i;
    u8 count_local;
    u32 primask;

    if ((channels == 0) || (channel_count == 0))
    {
        return 0U;
    }

    if (PPM_In_IsOnline() == 0U)
    {
        return 0U;
    }

    // 进入临界区，避免中断更新过程中读到半帧数据
    primask = __get_PRIMASK();
    __disable_irq();

    if (s_frame_valid == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    count_local = s_channel_count;
    for (i = 0U; (i < count_local) && (i < PPM_IN_MAX_CHANNELS); i++)
    {
        channels[i] = s_channels[i];
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    *channel_count = count_local;
    return 1U;
}

/**
 * @brief 读取指定通道的脉宽值
 */
u8 PPM_In_GetChannelUs(u8 channel_index, u16 *pulse_us)
{
    u8 count_local;
    u32 primask;

    if (pulse_us == 0)
    {
        return 0U;
    }

    if (PPM_In_IsOnline() == 0U)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    count_local = s_channel_count;
    if (channel_index < count_local)
    {
        *pulse_us = s_channels[channel_index];
    }
    else
    {
        *pulse_us = 1500U; // 默认中值
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

/**
 * @brief 判断PPM信号是否在线
 */
u8 PPM_In_IsOnline(void)
{
    if (s_initialized == 0U)
    {
        return 0U;
    }

    if (s_frame_valid == 0U)
    {
        return 0U;
    }

    if ((HAL_GetTick() - s_last_frame_tick) > PPM_IN_SIGNAL_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}
