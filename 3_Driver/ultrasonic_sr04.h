#ifndef __ULTRASONIC_SR04_H
#define __ULTRASONIC_SR04_H

/**
 * @file ultrasonic_sr04.h
 * @brief SR04 脉宽模式超声波驱动（支持双线/单线分时复用）
 *
 * ==================== 协议说明 ====================
 * 1) TRIG 拉高 >= 10us 触发一次测距
 * 2) ECHO 高电平脉宽 = 声波往返时间
 * 3) 距离换算（mm）：distance_mm = echo_us * 343 / 2000
 *
 * ==================== 单线模式接线（默认启用） ====================
 * 1) 将模块 TRIG 与 ECHO 短接（或按硬件方案并联）
 * 2) MCU 仅使用一个 IO：默认 PC12
 * 3) 软件流程：先输出 >=10us 触发脉冲，再切输入等待回波
 *
 * ==================== 双线模式接线（可切换） ====================
 * 模块 TRIG -> PC12（TRIG）
 * 模块 ECHO -> PD2（ECHO）
 * 模块 VCC/GND -> 对应电源和地
 *
 * ==================== CubeMX 配置建议 ====================
 * 单线：PC12 需支持运行时在输出/输入之间切换
 * 双线：PC12 输出，PD2 输入
 *
 * ==================== 注意事项 ====================
 * 单线短接时建议串联限流电阻，避免与模块 ECHO 驱动硬冲突。
 * PC12 常与 USART3_TX 复用，若已占用请更换 IO。
 */

#include "common.h"

/************************* 用户配置区 *************************/
// 1=单线分时复用（一个引脚），0=双线模式（TRIG/ECHO独立）
#define SR04_SINGLE_WIRE_ENABLE      1U

#if SR04_SINGLE_WIRE_ENABLE
    // 单线模式 IO（TRIG/ECHO 共用）
    #define SR04_IO_GPIO_PORT            GPIOC
    #define SR04_IO_GPIO_PIN             GPIO_PIN_12
    #define SR04_IO_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()

    // 为兼容驱动实现，单线下 TRIG/ECHO 映射到同一个 IO
    #define SR04_TRIG_GPIO_PORT          SR04_IO_GPIO_PORT
    #define SR04_TRIG_GPIO_PIN           SR04_IO_GPIO_PIN
    #define SR04_TRIG_GPIO_CLK_ENABLE()  SR04_IO_GPIO_CLK_ENABLE()

    #define SR04_ECHO_GPIO_PORT          SR04_IO_GPIO_PORT
    #define SR04_ECHO_GPIO_PIN           SR04_IO_GPIO_PIN
    #define SR04_ECHO_GPIO_CLK_ENABLE()  SR04_IO_GPIO_CLK_ENABLE()
#else
    // 双线模式 IO
    #define SR04_TRIG_GPIO_PORT          GPIOC
    #define SR04_TRIG_GPIO_PIN           GPIO_PIN_12
    #define SR04_TRIG_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()

    #define SR04_ECHO_GPIO_PORT          GPIOD
    #define SR04_ECHO_GPIO_PIN           GPIO_PIN_2
    #define SR04_ECHO_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#endif
/**************************************************************/

// 触发与超时参数（可按场景调整）
#define SR04_TRIGGER_PULSE_US        12U
#define SR04_WAIT_RISE_TIMEOUT_US    30000U
#define SR04_WAIT_FALL_TIMEOUT_US    30000U
#define SR04_IDLE_WAIT_TIMEOUT_US    2000U
#define SR04_MIN_MEASURE_PERIOD_MS   60U

// 滤波参数（用于 SR04_ReadDistanceMmFiltered）
#define SR04_FILTER_WINDOW_SIZE      5U      // 滑窗长度（建议奇数）
#define SR04_FILTER_NEAR_MM          300U    // 近距阈值（mm）
#define SR04_FILTER_JUMP_GATE_MM     180U    // 近距突增门限（mm）
#define SR04_FILTER_JUMP_REJECT_MAX  2U      // 连续拒绝突增次数上限
#define SR04_FILTER_ALPHA_NEAR_NUM   2U      // 近距低通系数 2/10
#define SR04_FILTER_ALPHA_NEAR_DEN   10U
#define SR04_FILTER_ALPHA_FAR_NUM    5U      // 远距低通系数 5/10
#define SR04_FILTER_ALPHA_FAR_DEN    10U

// 卡尔曼参数（第三层高级滤波）
#define SR04_KALMAN_Q                2.0f
#define SR04_KALMAN_R                16.0f
#define SR04_KALMAN_P_INIT           200.0f

// 状态码
#define SR04_STATUS_OK               0U
#define SR04_STATUS_PARAM_ERR        1U
#define SR04_STATUS_NOT_INIT         2U
#define SR04_STATUS_WAIT_IDLE_TO     3U
#define SR04_STATUS_WAIT_RISE_TO     4U
#define SR04_STATUS_WAIT_FALL_TO     5U

/**
 * @brief 初始化 SR04 驱动（GPIO + DWT 微秒计时）
 */
void SR04_Init(void);

/**
 * @brief 读取原始回波脉宽
 * @param echo_us 输出回波高电平脉宽，单位 us
 * @retval SR04_STATUS_*
 */
u8 SR04_ReadEchoUs(u32 *echo_us);

/**
 * @brief 读取距离（单次阻塞测量）
 * @param distance_mm 输出距离，单位 mm
 * @retval SR04_STATUS_*
 */
u8 SR04_ReadDistanceMm(u16 *distance_mm);

/**
 * @brief 读取距离（滤波版）
 * @param distance_mm_raw 输出原始距离，单位 mm
 * @param distance_mm 输出滤波后的距离，单位 mm
 * @retval SR04_STATUS_*
 * @note 内部包含：中值滤波 + 近距抗跳 + 分区低通 + 卡尔曼滤波
 */
u8 SR04_ReadDistanceMmFiltered(u16 *distance_mm_raw, u16 *distance_mm);

#endif /* __ULTRASONIC_SR04_H */
