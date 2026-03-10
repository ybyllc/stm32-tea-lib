/**
 * @file menu.h
 * @brief 菜单系统接口定义
 * @note 支持主菜单导航和信息页面显示
 */

#ifndef MENU_H
#define MENU_H

#include <stdint.h>

// 菜单页面类型枚举
typedef enum {
    MENU_PAGE_MAIN,      // 主菜单
    MENU_PAGE_GYRO,      // 陀螺仪信息页面
    MENU_PAGE_EC11_TEST, // EC11测试页面
    MENU_PAGE_TOF_TEST,  // TOF测距测试页面
    MENU_PAGE_FIELD_TASK, // 水田遍历任务说明页面
    MENU_PAGE_REMOTE_KEY_TEST, // 遥控器测试页面
    MENU_PAGE_PPM_INPUT_TEST,  // PPM输入测试页面
    MENU_PAGE_SBUS_INPUT_TEST, // SBUS输入测试页面
    MENU_PAGE_MOTOR_TEST, // 电机测试页面
    MENU_PAGE_ENCODER_TEST, // 编码器测试页面
    MENU_PAGE_ADC_TEST, // ADC测试页面
    MENU_PAGE_PS2_TEST,  // PS2手柄测试页面
    MENU_PAGE_PS2_CONTROL, // PS2控制模式页面
    MENU_PAGE_SERVO_PWM,  // 舵机调试页面
    MENU_PAGE_GPIO_TEST  // GPIO测试页面（PS2引脚测试）
} MenuPageType;

// 菜单项结构
typedef struct {
    const char* name;       // 菜单项名称
    MenuPageType page;      // 菜单项对应的页面
} MenuItem;

// 菜单状态结构
typedef struct {
    MenuPageType currentPage;  // 当前页面
    uint8_t currentItem;       // 当前选中的菜单项
    uint8_t lastEncoderCount;  // 上次编码器计数
    uint8_t isInMenu;          // 是否在菜单模式
} MenuState;

// 外部变量声明
extern MenuState menuState;
extern const MenuItem mainMenuItems[];
extern uint8_t MAIN_MENU_ITEM_COUNT;
extern uint8_t gyro_initialized;
extern uint8_t tof_initialized;
extern uint8_t remote_key_initialized;
extern uint8_t ppm_initialized;
extern uint8_t sbus_initialized;
extern uint8_t motor_initialized;
extern uint8_t adc_initialized;
extern uint8_t ps2_initialized;
extern uint8_t gpio_initialized;
extern int16_t motor_speeds[4];
extern uint8_t current_motor_index;
extern uint8_t motor_speed_adjust_mode;
extern uint8_t system_mode;
extern uint32_t task_start_time;
extern int32_t encoder1_total_count;
extern int32_t encoder2_total_count;

/**
 * @brief 菜单系统初始化
 */
void Menu_Init(void);

/**
 * @brief 菜单系统运行
 * @note 在主循环中调用
 */
void Menu_Run(void);

/**
 * @brief 获取当前菜单状态
 * @return 菜单状态指针
 */
MenuState* Menu_GetState(void);

/**
 * @brief 初始化陀螺仪（仅在第一次进入Gyro页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitGyro(void);

/**
 * @brief 初始化TOF（仅在第一次进入TOF页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitTof(void);

/**
 * @brief 初始化遥控器（仅在第一次进入Remote Key页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitRemoteKey(void);

/**
 * @brief 初始化PPM输入（仅在第一次进入PPM页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitPPM(void);

/**
 * @brief 初始化SBUS输入（仅在第一次进入SBUS页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitSBUS(void);

/**
 * @brief 初始化电机（仅在第一次进入Motor Test页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitMotor(void);

/**
 * @brief 初始化ADC（仅在第一次进入ADC页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitAdc(void);

/**
 * @brief 初始化PS2手柄（仅在第一次进入PS2页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitPS2(void);

/**
 * @brief  通用陀螺仪更新函数，根据当前使用的陀螺仪类型调用对应的更新函数
 * @param  无
 * @retval 无
 */
void Menu_Gryo_Update(void);

/**
 * @brief 添加新的子菜单
 * @param name 菜单项名称
 * @param page 菜单项对应的页面
 * @return 0-失败, 1-成功
 * @note 此函数需要在 Menu_Init() 之前调用
 */
uint8_t Menu_AddItem(const char* name, MenuPageType page);

/**
 * @brief 获取主菜单项数量
 * @return 主菜单项数量
 */
uint8_t Menu_GetMainItemCount(void);

/**
 * @brief 获取主菜单项
 * @param index 菜单项索引
 * @return 菜单项指针
 */
const MenuItem* Menu_GetMainItem(uint8_t index);

#endif /* MENU_H */
