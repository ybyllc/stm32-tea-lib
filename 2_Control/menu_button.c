/**
 * @file menu_input.c
 * @brief 菜单输入处理实现
 * @note 负责菜单系统的输入逻辑，包括编码器、按键和遥控器输入
 */

#include "menu_input.h"
#include "menu.h"
#include "encoder_knob.h"
#include "remote_key.h"
#include "motor_tb6612.h"
#include "tim.h"
#include <stdio.h>



/**
 * @brief 输入处理初始化
 */
void MenuInput_Init(void) {
    // 初始化编码器
    Ec11_knob_Init();
    
    // 初始化遥控器
    RemoteKey_Init();
}

/**
 * @brief 处理编码器输入
 * @param count 编码器计数值
 */
void MenuInput_HandleEncoder(int32_t count) {
    if (menuState.currentPage == MENU_PAGE_MAIN) {
        // 在主菜单中，根据编码器计数切换菜单项
        static int32_t last_count = 0;
        static uint32_t last_rotation_time = 0;
        static int8_t scroll_counter = 0;
        
        // 旋转检测时间阈值（ms）- 增大此值可降低灵敏度
        #define ROTATION_DELAY_MS 100
        // 边界滚动阈值 - 需要滚动的次数才循环回到另一边
        #define SCROLL_THRESHOLD 2
        
        int32_t delta = count - last_count;
        uint32_t current_time = HAL_GetTick();
        
        if (delta != 0 && (current_time - last_rotation_time) >= ROTATION_DELAY_MS) {
            if (delta > 0) {
                // 顺时针旋转
                if (menuState.currentItem < MAIN_MENU_ITEM_COUNT - 1) {
                    // 不是最后一项，正常滚动
                    menuState.currentItem++;
                    scroll_counter = 0;
                } else {
                    // 是最后一项，累积滚动次数
                    scroll_counter++;
                    if (scroll_counter >= SCROLL_THRESHOLD) {
                        // 达到阈值，回到第一项
                        menuState.currentItem = 0;
                        scroll_counter = 0;
                    }
                }
            } else if (delta < 0) {
                // 逆时针旋转
                if (menuState.currentItem > 0) {
                    // 不是第一项，正常滚动
                    menuState.currentItem--;
                    scroll_counter = 0;
                } else {
                    // 是第一项，累积滚动次数
                    scroll_counter--;
                    if (scroll_counter <= -SCROLL_THRESHOLD) {
                        // 达到阈值，回到最后一项
                        menuState.currentItem = MAIN_MENU_ITEM_COUNT - 1;
                        scroll_counter = 0;
                    }
                }
            }
            last_rotation_time = current_time;
        }
        
        last_count = count;
    } else if (menuState.currentPage == MENU_PAGE_MOTOR_TEST) {
        static int32_t last_count = 0;
        static uint32_t last_rotation_time = 0;
        
        // 旋转检测时间阈值（ms）
        #define MOTOR_ROTATION_DELAY_MS 100
        
        int32_t delta = count - last_count;
        uint32_t current_time = HAL_GetTick();
        
        if (delta != 0 && (current_time - last_rotation_time) >= MOTOR_ROTATION_DELAY_MS) {
            if (motor_speed_adjust_mode) {
                // 调整速度模式：调整当前选中电机的速度
                if (delta > 0) {
                    // 顺时针旋转，增加速度
                    if (motor_speeds[current_motor_index] < 1000) {
                        motor_speeds[current_motor_index] += 50;
                    }
                } else if (delta < 0) {
                    // 逆时针旋转，减少速度
                    if (motor_speeds[current_motor_index] > -1000) {
                        motor_speeds[current_motor_index] -= 50;
                    }
                }
                
                // 更新电机速度
                switch (current_motor_index) {
                    case 0:
                        Motor_SetSpeed(MOTOR_FRONT_LEFT, motor_speeds[0]);
                        break;
                    case 1:
                        Motor_SetSpeed(MOTOR_FRONT_RIGHT, motor_speeds[1]);
                        break;
                    case 2:
                        Motor_SetSpeed(MOTOR_BACK_LEFT, motor_speeds[2]);
                        break;
                    case 3:
                        Motor_SetSpeed(MOTOR_BACK_RIGHT, motor_speeds[3]);
                        break;
                }
            } else {
                // 选择电机模式：选择不同的电机
                if (delta > 0) {
                    // 顺时针旋转，选择下一个电机
                    current_motor_index = (current_motor_index + 1) % 4;
                } else if (delta < 0) {
                    // 逆时针旋转，选择上一个电机
                    current_motor_index = (current_motor_index + 3) % 4;
                }
            }
            
            last_rotation_time = current_time;
        }
        
        last_count = count;
    }
}

/**
 * @brief 处理按键事件
 * @param event 按键事件：1-短按，2-长按
 */
void MenuInput_HandleKeyEvent(uint8_t event) {
    if (event == 1) {  // 短按
        if (menuState.currentPage == MENU_PAGE_MAIN) {
            // 在主菜单中，短按进入选中的页面
            menuState.currentPage = mainMenuItems[menuState.currentItem].page;
        } else if (menuState.currentPage == MENU_PAGE_EC11_TEST) {
            // 在EC11测试页面，短按清零计数
            Ec11_knob_Clear_Count();
        } else if (menuState.currentPage == MENU_PAGE_MOTOR_TEST) {
            // 在电机测试页面，短按切换速度调整模式
            motor_speed_adjust_mode = !motor_speed_adjust_mode;
        } else if (menuState.currentPage == MENU_PAGE_PS2_TEST) {
            // 在PS2测试页面，短按触发详细调试
            extern void AX_PS2_DebugScan(void);
            AX_PS2_DebugScan();
        }
    } else if (event == 2) {  // 长按
        // 长按返回主菜单，保留一级菜单光标记忆
        menuState.currentPage = MENU_PAGE_MAIN;
    }
}

/**
 * @brief 扫描遥控器按键状态
 */
void MenuInput_ScanRemoteKey(void) {
    RemoteKey_Scan();
}

/**
 * @brief 获取编码器计数值
 * @return 编码器计数值
 */
int32_t MenuInput_GetEncoderCount(void) {
    return Ec11_knob_Get_Count();
}

/**
 * @brief 清除编码器计数
 */
void MenuInput_ClearEncoderCount(void) {
    Ec11_knob_Clear_Count();
}

/**
 * @brief 获取编码器按键状态
 * @return 按键状态：1-释放，0-按下
 */
uint8_t MenuInput_GetEncoderKeyState(void) {
    return Ec11_knob_Key_GetState();
}

/**
 * @brief 获取编码器按键事件
 * @return 按键事件：0-无事件，1-短按，2-长按
 */
uint8_t MenuInput_GetEncoderKeyEvent(void) {
    return Ec11_knob_Key_GetEvent();
}
