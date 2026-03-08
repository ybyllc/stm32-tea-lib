#ifndef _SERVO_PWM_H
#define _SERVO_PWM_H

#include "stdint.h"
#include "main.h"

// 舵机/PWM通道定义 - 对应TIM3的4个通道
#define SERVO_PWM_CH_A6     0   // TIM3_CH1 - PA6 - 右摇杆X
#define SERVO_PWM_CH_A7     1   // TIM3_CH2 - PA7 - 右摇杆Y
#define SERVO_PWM_CH_B0     2   // TIM3_CH3 - PB0 - 左摇杆X
#define SERVO_PWM_CH_B1     3   // TIM3_CH4 - PB1 - 左摇杆Y

// 航模PWM参数定义 (50Hz)
#define SERVO_PWM_FREQ      50      // 50Hz频率
#define SERVO_PWM_PERIOD    20000   // 20ms周期对应的计数值 (1MHz计数频率)
#define SERVO_PWM_MIN       500    // 1ms最小脉宽
#define SERVO_PWM_MAX       2500    // 2ms最大脉宽
#define SERVO_PWM_MID       1500    // 1.5ms中位脉宽

// 高频率模式，反应更快
#define SERVO_HIGHFREQ_MODE 1

/**
 * @brief  舵机/航模PWM初始化
 * @note   重新配置TIM3为50Hz，启动4路PWM输出
 */
void Servo_Pwm_Init(void);

/**
 * @brief  设置舵机PWM脉宽
 * @param  ch: 通道 (SERVO_PWM_CH_A6, SERVO_PWM_CH_A7, SERVO_PWM_CH_B0, SERVO_PWM_CH_B1)
 * @param  pulse_us: 脉宽值 (1000-2000us)，1500us为中位
 * @retval None
 */
void Servo_Pwm_SetPulse(uint8_t ch, uint16_t pulse_us);

/**
 * @brief  停止所有舵机输出（设置为中位）
 * @retval None
 */
void Servo_Pwm_StopAll(void);

#endif
