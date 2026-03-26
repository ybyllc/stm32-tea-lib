#include "encoder.h"

// 电机编码器初始化
void Encoder_Init(void)
{
    // 引脚在.h文件里定义
    // 启动TIM2和TIM5的编码器模式
    // 注意：TIM2和TIM5的初始化已经在CubeMX生成的MX_TIM2_Init()和MX_TIM5_Init()中完成
    // 这里只需要启动计数器
    HAL_TIM_Encoder_Start(ENCODER1_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(ENCODER2_TIM, TIM_CHANNEL_ALL);
    
    // 初始化时清零计数器
    Encoder_GetCount(ENCODER_1);
    Encoder_GetCount(ENCODER_2);
}

// 获取编码器计数
int32_t Encoder_GetCount(uint8_t encoder_num)
{
    int32_t count = 0;
    
    switch (encoder_num)
    {
        case ENCODER_1:
            count = (int32_t)__HAL_TIM_GET_COUNTER(ENCODER1_TIM);
            __HAL_TIM_SET_COUNTER(ENCODER1_TIM, 0);// 清零计数器
            break;
        case ENCODER_2:
            count = -(int32_t)__HAL_TIM_GET_COUNTER(ENCODER2_TIM);
            __HAL_TIM_SET_COUNTER(ENCODER2_TIM, 0);// 清零计数器
            break;
        default:
            count = 0;
            break;
    }
    
    return count;
}