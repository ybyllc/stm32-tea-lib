/**
 * @file key_pc6.c
 * @brief PC6按键驱动实现
 * @note 用于检测PC6引脚的按键状态
 */

#include "key_pc6.h"

/**
 * @brief PC6按键初始化
 * @retval 0-失败, 1-成功
 */
uint8_t KeyPC6_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 启用GPIOC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // 配置PC6引脚为输入模式
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // 上拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    return 1;
}

/**
 * @brief 获取PC6按键状态
 * @retval 0-按键按下, 1-按键释放
 */
uint8_t KeyPC6_GetState(void) {
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
}

/**
 * @brief 检测PC6按键事件
 * @retval 0-无事件, 1-短按, 2-长按
 */
uint8_t KeyPC6_CheckEvent(void) {
    static uint8_t key_state = 1;  // 按键状态，初始为释放
    static uint32_t key_time = 0;   // 按键按下的时间
    static uint8_t event_triggered = 0;  // 事件是否已触发
    
    uint8_t current_state = KeyPC6_GetState();
    
    if (key_state != current_state) {
        key_state = current_state;
        event_triggered = 0;
        
        if (current_state == 0) {  // 按键按下
            key_time = HAL_GetTick();
        } else {  // 按键释放
            if (HAL_GetTick() - key_time < 50) {
                // 按键按下时间小于50ms，可能是抖动
                return 0;
            } else if (HAL_GetTick() - key_time < 1000) {
                // 按键按下时间小于1秒，短按
                return 1;
            } else {
                // 按键按下时间大于等于1秒，长按
                return 2;
            }
        }
    } else if (current_state == 0 && !event_triggered) {
        // 按键持续按下，检查是否达到长按时间
        if (HAL_GetTick() - key_time >= 1000) {
            event_triggered = 1;
            return 2;
        }
    }
    
    return 0;
}
