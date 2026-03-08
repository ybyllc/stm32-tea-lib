#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"
#include "usart.h"
#include "gpio.h"

void Delay_us(u32 nus);
#define delay_ms(n) HAL_Delay(n)
#define delay_us(n) Delay_us(n)

/* 舵机控制相关全局变量 ------------------------------------*/
extern float servo_target_angle; // 目标角度
extern float servo_current_angle; // 当前角度

void App_Init(void);       // 上电一次性初始化
void App_Loop(void);       // 超级循环
void App_1ms_IRQHandler(void); // 1ms中断回调
void App_TIM1_Callback(void); // TIM1中断回调
void TIM1_100us_Callback(void); // TIM1 100us中断回调

#define delay_ms(n) HAL_Delay(n)


#ifdef __cplusplus
}
#endif
#endif /* APP_MAIN_H */