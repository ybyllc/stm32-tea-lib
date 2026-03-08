#include "servo_pwm.h"
#include "tim.h"

/**
 * @brief  舵机/航模PWM初始化
 * @note   重新配置TIM3为50Hz，启动4路PWM输出
 */
void Servo_Pwm_Init(void)
{
	// 配置TIM3为50Hz (48MHz / 48 / 20000 = 50Hz)
    __HAL_TIM_SET_PRESCALER(&htim3, 48 - 1);
#ifndef SERVO_HIGHFREQ_MODE // 高频率模式，反应更快 
    __HAL_TIM_SET_AUTORELOAD(&htim3, 20000 - 1);
#else
    __HAL_TIM_SET_AUTORELOAD(&htim3, 15000 - 1);//信号频率提升到66hz
#endif
    
    // 生成更新事件，使配置立即生效
    HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
	
    // 启动TIM3的4个PWM通道
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PA6
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // PA7
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // PB0
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // PB1
    
    // 初始化为中位
    Servo_Pwm_StopAll();
}

/**
 * @brief  设置舵机PWM脉宽
 * @param  ch: 通道 (SERVO_PWM_CH_A6, SERVO_PWM_CH_A7, SERVO_PWM_CH_B0, SERVO_PWM_CH_B1)
 * @param  pulse_us: 脉宽值 (1000-2000us)，1500us为中位
 * @retval None
 */
void Servo_Pwm_SetPulse(uint8_t ch, uint16_t pulse_us)
{
    // 限制脉宽范围
    if (pulse_us < SERVO_PWM_MIN) pulse_us = SERVO_PWM_MIN;
    if (pulse_us > SERVO_PWM_MAX) pulse_us = SERVO_PWM_MAX;
    
    // 根据通道设置对应的TIM3比较值
    switch (ch) {
        case SERVO_PWM_CH_A6:  // TIM3_CH1 - PA6
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
            break;
        case SERVO_PWM_CH_A7:  // TIM3_CH2 - PA7
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);
            break;
        case SERVO_PWM_CH_B0:  // TIM3_CH3 - PB0
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_us);
            break;
        case SERVO_PWM_CH_B1:  // TIM3_CH4 - PB1
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_us);
            break;
        default:
            break;
    }
}

/**
 * @brief  停止所有舵机输出（设置为中位）
 * @retval None
 */
void Servo_Pwm_StopAll(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_PWM_MID);  // PA6
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_PWM_MID);  // PA7
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO_PWM_MID);  // PB0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, SERVO_PWM_MID);  // PB1
}
