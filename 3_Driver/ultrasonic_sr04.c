#include "ultrasonic_sr04.h"

static u32 s_cycles_per_us = 1U;
static u8 s_sr04_inited = 0U;
static u32 s_last_measure_tick_ms = 0U;
static u8 s_last_status = SR04_STATUS_NOT_INIT;
static u16 s_last_distance_mm = 0U;
static u8 s_has_last_result = 0U;

// 滤波状态
static u16 s_filter_window[SR04_FILTER_WINDOW_SIZE];
static u8 s_filter_count = 0U;
static u8 s_filter_index = 0U;
static u8 s_filter_jump_reject_count = 0U;
static u16 s_filter_last_stage2_output = 0U;
static u8 s_filter_has_stage2_output = 0U;
static u16 s_filter_last_output = 0U;
static u8 s_filter_has_output = 0U;

typedef struct
{
    float x;   // 估计值
    float p;   // 估计误差协方差
    float q;   // 过程噪声
    float r;   // 测量噪声
    float k;   // 卡尔曼增益
    u8 inited;
} SR04_KalmanFilter_t;

static SR04_KalmanFilter_t s_filter_kalman;

/**
 * @brief 配置 TRIG 引脚为输出
 */
static void SR04_ConfigTrigOutput(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = SR04_TRIG_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SR04_TRIG_GPIO_PORT, &gpio_init);
}

/**
 * @brief 配置 ECHO 引脚为输入
 */
static void SR04_ConfigEchoInput(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = SR04_ECHO_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SR04_ECHO_GPIO_PORT, &gpio_init);
}

/**
 * @brief 启用 DWT 周期计数器，用于微秒级计时
 */
static void SR04_DwtInit(void)
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

/**
 * @brief 获取当前时间（us）
 */
static u32 SR04_GetTimeUs(void)
{
    return (u32)(DWT->CYCCNT / s_cycles_per_us);
}

/**
 * @brief 微秒延时（忙等待）
 */
static void SR04_DelayUs(u32 delay_us)
{
    u32 start_us = SR04_GetTimeUs();
    while ((SR04_GetTimeUs() - start_us) < delay_us)
    {
        // busy wait
    }
}

/**
 * @brief 等待 ECHO 变为目标电平
 */
static u8 SR04_WaitEchoLevel(GPIO_PinState target_level, u32 timeout_us)
{
    u32 start_us = SR04_GetTimeUs();
    while (HAL_GPIO_ReadPin(SR04_ECHO_GPIO_PORT, SR04_ECHO_GPIO_PIN) != target_level)
    {
        if ((SR04_GetTimeUs() - start_us) > timeout_us)
        {
            return 0U;
        }
    }
    return 1U;
}

void SR04_Init(void)
{
    SR04_TRIG_GPIO_CLK_ENABLE();
    SR04_ECHO_GPIO_CLK_ENABLE();

    // 默认空闲态：TRIG 低，ECHO 输入
    SR04_ConfigTrigOutput();
    HAL_GPIO_WritePin(SR04_TRIG_GPIO_PORT, SR04_TRIG_GPIO_PIN, GPIO_PIN_RESET);
    SR04_ConfigEchoInput();

    SR04_DwtInit();
    s_sr04_inited = 1U;
    s_last_measure_tick_ms = 0U;
    s_last_status = SR04_STATUS_NOT_INIT;
    s_last_distance_mm = 0U;
    s_has_last_result = 0U;

    s_filter_count = 0U;
    s_filter_index = 0U;
    s_filter_jump_reject_count = 0U;
    s_filter_last_stage2_output = 0U;
    s_filter_has_stage2_output = 0U;
    s_filter_last_output = 0U;
    s_filter_has_output = 0U;

    s_filter_kalman.x = 0.0f;
    s_filter_kalman.p = SR04_KALMAN_P_INIT;
    s_filter_kalman.q = SR04_KALMAN_Q;
    s_filter_kalman.r = SR04_KALMAN_R;
    s_filter_kalman.k = 0.0f;
    s_filter_kalman.inited = 0U;
}

u8 SR04_ReadEchoUs(u32 *echo_us)
{
    u32 rise_us;
    u32 fall_us;

    if (echo_us == 0)
    {
        return SR04_STATUS_PARAM_ERR;
    }

    if (s_sr04_inited == 0U)
    {
        return SR04_STATUS_NOT_INIT;
    }

#if SR04_SINGLE_WIRE_ENABLE
    // 单线模式：触发前先切输入，避免与模块驱动冲突
    SR04_ConfigEchoInput();
#endif

    // 确保触发前 ECHO 已回到低电平
    if (SR04_WaitEchoLevel(GPIO_PIN_RESET, SR04_IDLE_WAIT_TIMEOUT_US) == 0U)
    {
        return SR04_STATUS_WAIT_IDLE_TO;
    }

    // 发送触发脉冲
#if SR04_SINGLE_WIRE_ENABLE
    SR04_ConfigTrigOutput();
#endif
    HAL_GPIO_WritePin(SR04_TRIG_GPIO_PORT, SR04_TRIG_GPIO_PIN, GPIO_PIN_RESET);
    SR04_DelayUs(2U);
    HAL_GPIO_WritePin(SR04_TRIG_GPIO_PORT, SR04_TRIG_GPIO_PIN, GPIO_PIN_SET);
    SR04_DelayUs(SR04_TRIGGER_PULSE_US);
    HAL_GPIO_WritePin(SR04_TRIG_GPIO_PORT, SR04_TRIG_GPIO_PIN, GPIO_PIN_RESET);
#if SR04_SINGLE_WIRE_ENABLE
    SR04_ConfigEchoInput();
#endif

    // 等待上升沿（ECHO 变高）
    if (SR04_WaitEchoLevel(GPIO_PIN_SET, SR04_WAIT_RISE_TIMEOUT_US) == 0U)
    {
        return SR04_STATUS_WAIT_RISE_TO;
    }
    rise_us = SR04_GetTimeUs();

    // 等待下降沿（ECHO 变低）
    if (SR04_WaitEchoLevel(GPIO_PIN_RESET, SR04_WAIT_FALL_TIMEOUT_US) == 0U)
    {
        return SR04_STATUS_WAIT_FALL_TO;
    }
    fall_us = SR04_GetTimeUs();

    *echo_us = (fall_us - rise_us);
    return SR04_STATUS_OK;
}

u8 SR04_ReadDistanceMm(u16 *distance_mm)
{
    u8 status;
    u32 echo_us;
    u32 distance_calc_mm;
    u32 now_tick_ms;

    if (distance_mm == 0)
    {
        return SR04_STATUS_PARAM_ERR;
    }

    if (s_sr04_inited == 0U)
    {
        return SR04_STATUS_NOT_INIT;
    }

    now_tick_ms = HAL_GetTick();
    if (s_has_last_result && ((now_tick_ms - s_last_measure_tick_ms) < SR04_MIN_MEASURE_PERIOD_MS))
    {
        *distance_mm = s_last_distance_mm;
        return s_last_status;
    }

    status = SR04_ReadEchoUs(&echo_us);
    if (status != SR04_STATUS_OK)
    {
        s_last_status = status;
        s_last_measure_tick_ms = now_tick_ms;
        s_has_last_result = 1U;
        return status;
    }

    // 距离(mm) = 时间(us) * 343 / 2000，+1000用于四舍五入
    distance_calc_mm = (echo_us * 343U + 1000U) / 2000U;
    if (distance_calc_mm > 65535U)
    {
        distance_calc_mm = 65535U;
    }

    *distance_mm = (u16)distance_calc_mm;
    s_last_distance_mm = *distance_mm;
    s_last_status = SR04_STATUS_OK;
    s_last_measure_tick_ms = now_tick_ms;
    s_has_last_result = 1U;
    return SR04_STATUS_OK;
}

/**
 * @brief 对输入数据进行中值计算（拷贝后排序）
 */
static u16 SR04_FilterGetMedian(const u16 *values, u8 count)
{
    u16 sorted[SR04_FILTER_WINDOW_SIZE];
    u8 i;
    u8 j;

    if ((values == 0) || (count == 0U))
    {
        return 0U;
    }

    if (count > SR04_FILTER_WINDOW_SIZE)
    {
        count = SR04_FILTER_WINDOW_SIZE;
    }

    for (i = 0U; i < count; i++)
    {
        sorted[i] = values[i];
    }

    // 简单插入排序
    for (i = 1U; i < count; i++)
    {
        u16 key = sorted[i];
        j = i;
        while ((j > 0U) && (sorted[j - 1U] > key))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = key;
    }

    return sorted[count / 2U];
}

/**
 * @brief 整数低通滤波：y = a*x + (1-a)*y_prev
 */
static u16 SR04_FilterLowPass(u16 prev, u16 current, u16 alpha_num, u16 alpha_den)
{
    u32 output;

    if ((alpha_den == 0U) || (alpha_num >= alpha_den))
    {
        return current;
    }

    output = (u32)alpha_num * (u32)current + (u32)(alpha_den - alpha_num) * (u32)prev;
    output = (output + (alpha_den / 2U)) / alpha_den; // 四舍五入
    if (output > 65535U)
    {
        output = 65535U;
    }

    return (u16)output;
}

/**
 * @brief 卡尔曼更新（单变量）
 */
static float SR04_KalmanUpdate(SR04_KalmanFilter_t *kf, float measurement)
{
    if (kf == 0)
    {
        return measurement;
    }

    if (kf->inited == 0U)
    {
        kf->x = measurement;
        kf->p = SR04_KALMAN_P_INIT;
        kf->k = 0.0f;
        kf->inited = 1U;
        return kf->x;
    }

    // 预测
    kf->p = kf->p + kf->q;

    // 更新
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->x;
}

/**
 * @brief 卡尔曼滤波输出（mm）
 */
static u16 SR04_FilterKalman(u16 input_mm)
{
    float filtered;

    filtered = SR04_KalmanUpdate(&s_filter_kalman, (float)input_mm);

    if (filtered < 0.0f)
    {
        filtered = 0.0f;
    }
    if (filtered > 65535.0f)
    {
        filtered = 65535.0f;
    }

    return (u16)(filtered + 0.5f);
}

u8 SR04_ReadDistanceMmFiltered(u16 *distance_mm_raw, u16 *distance_mm)
{
    u8 status;
    u16 raw_mm = 0U;
    u16 median_mm;
    u16 candidate_mm;
    u16 lowpass_mm;
    u16 output_mm;
    u8 i;

    if ((distance_mm_raw == 0) || (distance_mm == 0))
    {
        return SR04_STATUS_PARAM_ERR;
    }

    status = SR04_ReadDistanceMm(&raw_mm);
    *distance_mm_raw = raw_mm;

    if (status != SR04_STATUS_OK)
    {
        // 读值异常时优先保持上一帧，避免显示抖动
        if (s_filter_has_output)
        {
            *distance_mm = s_filter_last_output;
            return SR04_STATUS_OK;
        }
        return status;
    }

    // 1) 窗口更新：用于中值滤波
    s_filter_window[s_filter_index] = raw_mm;
    s_filter_index++;
    if (s_filter_index >= SR04_FILTER_WINDOW_SIZE)
    {
        s_filter_index = 0U;
    }

    if (s_filter_count < SR04_FILTER_WINDOW_SIZE)
    {
        s_filter_count++;
    }

    // 取有效窗口，构建连续数组求中值
    {
        u16 buffer[SR04_FILTER_WINDOW_SIZE];
        for (i = 0U; i < s_filter_count; i++)
        {
            buffer[i] = s_filter_window[i];
        }
        median_mm = SR04_FilterGetMedian(buffer, s_filter_count);
    }

    candidate_mm = median_mm;

    // 2) 近距抗跳：近距离时，异常突增先拒绝若干帧
    if (s_filter_has_stage2_output)
    {
        if ((s_filter_last_stage2_output < SR04_FILTER_NEAR_MM) &&
            (candidate_mm > (u16)(s_filter_last_stage2_output + SR04_FILTER_JUMP_GATE_MM)))
        {
            if (s_filter_jump_reject_count < SR04_FILTER_JUMP_REJECT_MAX)
            {
                s_filter_jump_reject_count++;
                candidate_mm = s_filter_last_stage2_output;
            }
            else
            {
                s_filter_jump_reject_count = 0U;
            }
        }
        else
        {
            s_filter_jump_reject_count = 0U;
        }
    }

    // 3) 分区低通：近距更强、远距更快
    if (!s_filter_has_stage2_output)
    {
        lowpass_mm = candidate_mm;
        s_filter_has_stage2_output = 1U;
    }
    else if ((candidate_mm < SR04_FILTER_NEAR_MM) || (s_filter_last_stage2_output < SR04_FILTER_NEAR_MM))
    {
        lowpass_mm = SR04_FilterLowPass(s_filter_last_stage2_output,
                                        candidate_mm,
                                        SR04_FILTER_ALPHA_NEAR_NUM,
                                        SR04_FILTER_ALPHA_NEAR_DEN);
    }
    else
    {
        lowpass_mm = SR04_FilterLowPass(s_filter_last_stage2_output,
                                        candidate_mm,
                                        SR04_FILTER_ALPHA_FAR_NUM,
                                        SR04_FILTER_ALPHA_FAR_DEN);
    }

    s_filter_last_stage2_output = lowpass_mm;

//    // 4) 卡尔曼滤波：用于动态场景下平衡抑噪与响应
//    output_mm = SR04_FilterKalman(lowpass_mm);

//    s_filter_last_output = output_mm;
//    s_filter_has_output = 1U;
	
    *distance_mm = lowpass_mm;//output_mm;
    return SR04_STATUS_OK;
}
