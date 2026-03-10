/**
 * @file car_task.c
 * @brief 小车任务实现
 * @note 保留原直行任务，并新增水田弓字遍历任务
 */

#include "car_task.h"
#include "ICM42670.h"
#include "TOF_VL53L0X.h"
#include "motor_tb6612.h"
#include "oled.h"
#include "wit_gyro_sdk.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"

#include <math.h>
#include <stdio.h>

/**
 * @brief 陀螺仪来源
 */
typedef enum {
    GYRO_TYPE_NONE = 0,      // 无可用陀螺仪
    GYRO_TYPE_WIT,           // WIT 串口陀螺仪
    GYRO_TYPE_ICM42670       // ICM42670 陀螺仪
} GyroType;

/**
 * @brief 水田遍历内部状态
 */
typedef enum {
    FIELD_SCAN_STATE_IDLE = 0,       // 空闲
    FIELD_SCAN_STATE_FORWARD,        // 沿当前作业行前进
    FIELD_SCAN_STATE_OBSTACLE_CHECK, // 障碍确认
    FIELD_SCAN_STATE_TURN_TO_SHIFT,  // 第一次转 90 度，准备换行
    FIELD_SCAN_STATE_SHIFT_FORWARD,  // 横向前进一段距离
    FIELD_SCAN_STATE_TURN_TO_ROW,    // 第二次转 90 度，进入下一行
    FIELD_SCAN_STATE_FAULT           // 故障停机
} FieldScanState;

/**
 * @brief 水田遍历故障码
 */
typedef enum {
    FIELD_SCAN_FAULT_NONE = 0,       // 无故障
    FIELD_SCAN_FAULT_TOF_INIT,       // TOF 初始化失败
    FIELD_SCAN_FAULT_TOF_TIMEOUT     // TOF 长时间无有效数据
} FieldScanFault;

/* 电压补偿参数 */
#define REFERENCE_VOLTAGE                8.4f
#define ADC_DIVIDER_RATIO                4.0f
#define DEBUG_SIMULATE_VOLTAGE           8.4f

/* 原直行任务参数 */
#define STRAIGHT_TASK_BASE_SPEED         150

/* 水田遍历任务参数 */
#define FIELD_SCAN_FORWARD_PWM           180
#define FIELD_SCAN_SHIFT_PWM             170
#define FIELD_SCAN_TURN_PWM              160
#define FIELD_SCAN_MAX_PWM               300
#define FIELD_SCAN_OBSTACLE_MM           350U
#define FIELD_SCAN_CONFIRM_COUNT         3U
#define FIELD_SCAN_TOF_PERIOD_MS         50U
#define FIELD_SCAN_SIDESTEP_TIME_MS      600U
#define FIELD_SCAN_ANGLE_TOLERANCE_DEG   3.0f
#define FIELD_SCAN_ANGLE_STABLE_MS       120U
#define FIELD_SCAN_TOF_FAULT_TIMEOUT_MS  500U
#define FIELD_SCAN_PID_I_LIMIT           80.0f

/* 通用运行状态 */
static uint8_t car_task_running = 0U;               // 任务运行标志
static CarTaskModeTypeDef car_task_selected_mode = CAR_TASK_MODE_STRAIGHT; // 菜单中当前选中的任务
static CarTaskModeTypeDef car_task_active_mode = CAR_TASK_MODE_STRAIGHT;   // 已启动的任务模式
static GyroType current_gyro_type = GYRO_TYPE_NONE; // 当前实际使用的陀螺仪

/* 通用配置参数 */
static float target_angle = 30.0f;                  // 原直行任务目标角度
static int16_t max_speed = FIELD_SCAN_MAX_PWM;      // 电机速度上限

/* 电压补偿相关变量 */
static ADC_HandleTypeDef hadc1;                     // ADC 句柄
static uint8_t adc_initialized = 0U;               // ADC 初始化标志
static float current_voltage = 0.0f;               // 当前电池电压
static float voltage_compensation = 1.0f;          // 电压补偿系数

/* 原直行任务运行变量 */
static float straight_error = 0.0f;                // 当前角度误差
static float straight_last_error = 0.0f;           // 上一次角度误差
static float straight_integral = 0.0f;             // 原直行任务积分项
static float straight_derivative = 0.0f;           // 原直行任务微分项
static float straight_pid_output = 0.0f;           // 原直行任务 PID 输出

/* 水田遍历 PID 变量 */
static float field_pid_kp = 4.5f;                  // 航向环比例系数
static float field_pid_ki = 0.02f;                 // 航向环积分系数
static float field_pid_kd = 0.18f;                 // 航向环微分系数
static float field_error = 0.0f;                   // 当前航向误差
static float field_last_error = 0.0f;              // 上一次航向误差
static float field_integral = 0.0f;                // 积分项
static float field_derivative = 0.0f;              // 微分项
static float field_pid_output = 0.0f;              // 航向环输出

/* 水田遍历运行状态 */
static uint8_t field_tof_ready = 0U;               // TOF 是否初始化成功
static uint8_t field_tof_valid = 0U;               // 最近一次 TOF 读数是否有效
static uint8_t field_tof_sample_updated = 0U;      // 本周期是否刷新了 TOF 数据
static uint8_t field_obstacle_confirm_count = 0U;  // 障碍确认计数
static uint8_t field_row_index = 0U;               // 当前作业行索引，0/1 交替
static uint16_t field_tof_distance_mm = 0xFFFFU;   // 最近一次 TOF 距离
static uint32_t field_last_control_tick = 0U;      // 上一次控制时间戳
static uint32_t field_last_tof_tick = 0U;          // 上一次 TOF 采样时间
static uint32_t field_last_tof_valid_tick = 0U;    // 上一次有效 TOF 时间
static uint32_t field_turn_stable_tick = 0U;       // 转向进入角度容差后的时间
static uint32_t field_shift_start_tick = 0U;       // 横移起始时间
static float field_heading_row_a = 0.0f;           // 第一行航向
static float field_heading_row_b = 180.0f;         // 第二行航向
static float field_heading_shift = 90.0f;          // 换行航向
static float field_current_row_heading = 0.0f;     // 当前作业行目标航向
static float field_current_turn_target = 0.0f;     // 当前转向目标航向
static int16_t field_last_left_speed = 0;          // 上一次后左轮 PWM
static int16_t field_last_right_speed = 0;         // 上一次后右轮 PWM
static FieldScanState field_scan_state = FIELD_SCAN_STATE_IDLE; // 水田遍历状态
static FieldScanFault field_scan_fault = FIELD_SCAN_FAULT_NONE; // 水田遍历故障码

/**
 * @brief 初始化 ADC 电压采样
 * @retval 1-成功，0-失败
 */
static uint8_t Car_Task_InitADC(void);

/**
 * @brief 读取当前电池电压
 * @retval 电压值，单位 V
 */
static float Car_Task_ReadVoltage(void);

/**
 * @brief 根据当前电压计算补偿系数
 * @param voltage 当前电压
 * @retval 补偿系数
 */
static float Car_Task_CalcCompensation(float voltage);

/**
 * @brief 更新当前陀螺仪数据
 * @retval 无
 */
static void Car_Task_UpdateGyro(void);

/**
 * @brief 复位原直行任务的 PID 变量
 * @retval 无
 */
static void Car_Task_ResetStraightPid(void);

/**
 * @brief 复位水田遍历任务的 PID 变量
 * @retval 无
 */
static void Car_Task_ResetFieldPid(void);

/**
 * @brief 复位水田遍历运行时状态
 * @retval 无
 */
static void Car_Task_ResetFieldRuntime(void);

/**
 * @brief 清空所有电机输出
 * @retval 无
 */
static void Car_Task_StopAllMotors(void);

/**
 * @brief 将角度归一化到 [-180, 180]
 * @param angle 输入角度
 * @retval 归一化后的角度
 */
static float Car_Task_NormalizeAngle(float angle);

/**
 * @brief 计算目标角与当前角之间的最短误差
 * @param target 目标角度
 * @param current 当前角度
 * @retval 最短角度误差
 */
static float Car_Task_CalcAngleError(float target, float current);

/**
 * @brief 约束速度到允许范围
 * @param speed 输入速度
 * @retval 约束后的速度
 */
static int16_t Car_Task_ClampSpeed(int32_t speed);

/**
 * @brief 对速度施加电压补偿
 * @param speed 输入速度
 * @retval 补偿后的速度
 */
static int16_t Car_Task_ApplyCompensation(int16_t speed);

/**
 * @brief 设置仅后轮的差速输出
 * @param left_speed 后左轮速度
 * @param right_speed 后右轮速度
 * @retval 无
 */
static void Car_Task_SetRearMotorSpeed(int16_t left_speed, int16_t right_speed);

/**
 * @brief 执行原有直行任务
 * @retval 无
 */
static void Car_Task_RunStraightTask(void);

/**
 * @brief 更新水田遍历任务的 TOF 缓存
 * @param now_tick 当前系统时刻
 * @retval 无
 */
static void Car_Task_UpdateFieldTof(uint32_t now_tick);

/**
 * @brief 以航向闭环方式驱动后轮前进
 * @param target_heading 目标航向
 * @param current_heading 当前航向
 * @param dt_s 控制周期，单位秒
 * @param base_speed 基础前进 PWM
 * @retval 无
 */
static void Car_Task_RunFieldHeadingHold(float target_heading, float current_heading, float dt_s, int16_t base_speed);

/**
 * @brief 驱动后轮原地转向
 * @param target_heading 目标航向
 * @param current_heading 当前航向
 * @retval 无
 */
static void Car_Task_RunFieldTurn(float target_heading, float current_heading);

/**
 * @brief 判断转向是否稳定到达目标角度
 * @param target_heading 目标航向
 * @param current_heading 当前航向
 * @param now_tick 当前系统时刻
 * @retval 1-稳定到位，0-未到位
 */
static uint8_t Car_Task_FieldTurnStable(float target_heading, float current_heading, uint32_t now_tick);

/**
 * @brief 进入水田遍历故障状态
 * @param fault 故障码
 * @retval 无
 */
static void Car_Task_FieldEnterFault(FieldScanFault fault);

/**
 * @brief 执行水田遍历任务
 * @retval 无
 */
static void Car_Task_RunFieldScanTask(void);

/**
 * @brief 显示水田遍历任务运行信息
 * @param current_heading 当前航向
 * @retval 无
 */
static void Car_Task_DisplayFieldInfo(float current_heading);

/**
 * @brief 获取水田遍历状态字符串
 * @param state 当前状态
 * @retval 状态字符串
 */
static const char *Car_Task_GetFieldStateText(FieldScanState state);

/**
 * @brief 获取水田遍历故障字符串
 * @param fault 当前故障
 * @retval 故障字符串
 */
static const char *Car_Task_GetFieldFaultText(FieldScanFault fault);

/**
 * @brief 初始化 ADC 电压采样
 * @retval 1-成功，0-失败
 */
static uint8_t Car_Task_InitADC(void)
{
    if (adc_initialized) {
        return 1U;
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {
        .Pin = GPIO_PIN_4,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL
    };
    HAL_GPIO_Init(GPIOC, &gpio_init);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return 0U;
    }

    ADC_ChannelConfTypeDef adc_config = {
        .Channel = ADC_CHANNEL_14,
        .Rank = 1,
        .SamplingTime = ADC_SAMPLETIME_480CYCLES
    };

    if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK) {
        return 0U;
    }

    adc_initialized = 1U;
    return 1U;
}

/**
 * @brief 读取当前电池电压
 * @retval 电压值，单位 V
 */
static float Car_Task_ReadVoltage(void)
{
    if (DEBUG_SIMULATE_VOLTAGE > 0.0f) {
        return DEBUG_SIMULATE_VOLTAGE;
    }

    if (!adc_initialized) {
        return 0.0f;
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);

    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return ((float)adc_value * 3.3f / 4095.0f) * ADC_DIVIDER_RATIO;
}

/**
 * @brief 根据当前电压计算补偿系数
 * @param voltage 当前电压
 * @retval 补偿系数
 */
static float Car_Task_CalcCompensation(float voltage)
{
    if (DEBUG_SIMULATE_VOLTAGE > 0.0f) {
        if ((voltage < 5.0f) || (voltage > 10.0f)) {
            return 1.0f;
        }
        return REFERENCE_VOLTAGE / voltage;
    }

    if ((voltage < 6.0f) || (voltage > 8.4f)) {
        return 1.0f;
    }

    return REFERENCE_VOLTAGE / voltage;
}

/**
 * @brief 根据当前陀螺仪来源刷新姿态数据
 * @retval 无
 */
static void Car_Task_UpdateGyro(void)
{
    switch (current_gyro_type) {
        case GYRO_TYPE_ICM42670:
            ICM42670_Gryo_Update();
            break;
        case GYRO_TYPE_WIT:
            Gryo_Update();
            break;
        case GYRO_TYPE_NONE:
        default:
            break;
    }
}

/**
 * @brief 复位原直行任务 PID 变量
 * @retval 无
 */
static void Car_Task_ResetStraightPid(void)
{
    straight_error = 0.0f;
    straight_last_error = 0.0f;
    straight_integral = 0.0f;
    straight_derivative = 0.0f;
    straight_pid_output = 0.0f;
}

/**
 * @brief 复位水田遍历 PID 变量
 * @retval 无
 */
static void Car_Task_ResetFieldPid(void)
{
    field_error = 0.0f;
    field_last_error = 0.0f;
    field_integral = 0.0f;
    field_derivative = 0.0f;
    field_pid_output = 0.0f;
}

/**
 * @brief 复位水田遍历运行状态
 * @retval 无
 */
static void Car_Task_ResetFieldRuntime(void)
{
    field_tof_valid = 0U;
    field_tof_sample_updated = 0U;
    field_obstacle_confirm_count = 0U;
    field_row_index = 0U;
    field_tof_distance_mm = 0xFFFFU;
    field_last_control_tick = 0U;
    field_last_tof_tick = 0U;
    field_last_tof_valid_tick = 0U;
    field_turn_stable_tick = 0U;
    field_shift_start_tick = 0U;
    field_heading_row_a = 0.0f;
    field_heading_row_b = 180.0f;
    field_heading_shift = 90.0f;
    field_current_row_heading = 0.0f;
    field_current_turn_target = 0.0f;
    field_last_left_speed = 0;
    field_last_right_speed = 0;
    field_scan_state = FIELD_SCAN_STATE_IDLE;
    field_scan_fault = FIELD_SCAN_FAULT_NONE;
    Car_Task_ResetFieldPid();
}

/**
 * @brief 清空所有电机输出
 * @retval 无
 */
static void Car_Task_StopAllMotors(void)
{
    Motor_StopAll();
    field_last_left_speed = 0;
    field_last_right_speed = 0;
}

/**
 * @brief 将角度归一化到 [-180, 180]
 * @param angle 输入角度
 * @retval 归一化后的角度
 */
static float Car_Task_NormalizeAngle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 计算目标角与当前角之间的最短误差
 * @param target 目标角度
 * @param current 当前角度
 * @retval 最短角度误差
 */
static float Car_Task_CalcAngleError(float target, float current)
{
    return Car_Task_NormalizeAngle(target - current);
}

/**
 * @brief 约束速度到允许范围
 * @param speed 输入速度
 * @retval 约束后的速度
 */
static int16_t Car_Task_ClampSpeed(int32_t speed)
{
    if (speed > max_speed) {
        speed = max_speed;
    } else if (speed < -max_speed) {
        speed = -max_speed;
    }
    return (int16_t)speed;
}

/**
 * @brief 对速度施加电压补偿
 * @param speed 输入速度
 * @retval 补偿后的速度
 */
static int16_t Car_Task_ApplyCompensation(int16_t speed)
{
    float compensated = (float)speed * voltage_compensation;
    return Car_Task_ClampSpeed((int32_t)compensated);
}

/**
 * @brief 设置仅后轮的差速输出，前轮保持停止
 * @param left_speed 后左轮速度
 * @param right_speed 后右轮速度
 * @retval 无
 */
static void Car_Task_SetRearMotorSpeed(int16_t left_speed, int16_t right_speed)
{
    left_speed = Car_Task_ClampSpeed(left_speed);
    right_speed = Car_Task_ClampSpeed(right_speed);

    Motor_SetSpeed(MOTOR_FRONT_LEFT, 0);
    Motor_SetSpeed(MOTOR_FRONT_RIGHT, 0);
    Motor_SetSpeed(MOTOR_BACK_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_BACK_RIGHT, right_speed);

    field_last_left_speed = left_speed;
    field_last_right_speed = right_speed;
}

/**
 * @brief 执行原有直行任务
 * @retval 无
 */
static void Car_Task_RunStraightTask(void)
{
    int16_t left_speed;
    int16_t right_speed;
    float current_angle;
    char str[24];

    Car_Task_UpdateGyro();

    current_voltage = Car_Task_ReadVoltage();
    voltage_compensation = Car_Task_CalcCompensation(current_voltage);
    current_angle = fAngle[2];

    straight_error = target_angle - current_angle;
    straight_integral += straight_error * 0.01f;
    straight_derivative = (straight_error - straight_last_error) / 0.01f;
    straight_pid_output = straight_error + straight_integral + straight_derivative;
    straight_last_error = straight_error;

    left_speed = Car_Task_ApplyCompensation(STRAIGHT_TASK_BASE_SPEED);
    right_speed = Car_Task_ApplyCompensation(STRAIGHT_TASK_BASE_SPEED);

    Motor_SetSpeed(MOTOR_FRONT_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_FRONT_RIGHT, right_speed);
    Motor_SetSpeed(MOTOR_BACK_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_BACK_RIGHT, right_speed);

    OLED_Clear(0);
    OLED_ShowString(0, 0, "Car Task", 16);

    sprintf(str, "Z:%.1f T:%.1f", current_angle, target_angle);
    OLED_ShowString(0, 2, str, 12);

    sprintf(str, "V:%.1f C:%.2f", current_voltage, voltage_compensation);
    OLED_ShowString(0, 3, str, 12);

    sprintf(str, "L:%d R:%d", left_speed, right_speed);
    OLED_ShowString(0, 4, str, 12);

    OLED_Refresh();
}

/**
 * @brief 更新水田遍历任务的 TOF 缓存
 * @param now_tick 当前系统时刻
 * @retval 无
 */
static void Car_Task_UpdateFieldTof(uint32_t now_tick)
{
    field_tof_sample_updated = 0U;

    if (!field_tof_ready) {
        return;
    }

    if ((field_last_tof_tick != 0U) &&
        ((now_tick - field_last_tof_tick) < FIELD_SCAN_TOF_PERIOD_MS)) {
        return;
    }

    field_last_tof_tick = now_tick;
    field_tof_sample_updated = 1U;

    field_tof_distance_mm = VL53L0X_ReadDistance();
    if (field_tof_distance_mm != 0xFFFFU) {
        field_tof_valid = 1U;
        field_last_tof_valid_tick = now_tick;
    } else {
        field_tof_valid = 0U;
    }
}

/**
 * @brief 以航向闭环方式驱动后轮前进
 * @param target_heading 目标航向
 * @param current_heading 当前航向
 * @param dt_s 控制周期，单位秒
 * @param base_speed 基础前进 PWM
 * @retval 无
 */
static void Car_Task_RunFieldHeadingHold(float target_heading, float current_heading, float dt_s, int16_t base_speed)
{
    int16_t left_speed;
    int16_t right_speed;

    field_error = Car_Task_CalcAngleError(target_heading, current_heading);
    field_integral += field_error * dt_s;
    if (field_integral > FIELD_SCAN_PID_I_LIMIT) {
        field_integral = FIELD_SCAN_PID_I_LIMIT;
    } else if (field_integral < -FIELD_SCAN_PID_I_LIMIT) {
        field_integral = -FIELD_SCAN_PID_I_LIMIT;
    }

    field_derivative = (field_error - field_last_error) / dt_s;
    field_pid_output = (field_pid_kp * field_error) +
                       (field_pid_ki * field_integral) +
                       (field_pid_kd * field_derivative);
    field_last_error = field_error;

    left_speed = Car_Task_ApplyCompensation((int16_t)(base_speed - field_pid_output));
    right_speed = Car_Task_ApplyCompensation((int16_t)(base_speed + field_pid_output));
    Car_Task_SetRearMotorSpeed(left_speed, right_speed);
}

/**
 * @brief 驱动后轮原地转向
 * @param target_heading 目标航向
 * @param current_heading 当前航向
 * @retval 无
 */
static void Car_Task_RunFieldTurn(float target_heading, float current_heading)
{
    float angle_error = Car_Task_CalcAngleError(target_heading, current_heading);
    int16_t turn_speed = FIELD_SCAN_TURN_PWM;

    if (fabsf(angle_error) < 15.0f) {
        turn_speed = 120;
    }

    if (angle_error >= 0.0f) {
        Car_Task_SetRearMotorSpeed(Car_Task_ApplyCompensation(-turn_speed),
                                   Car_Task_ApplyCompensation(turn_speed));
    } else {
        Car_Task_SetRearMotorSpeed(Car_Task_ApplyCompensation(turn_speed),
                                   Car_Task_ApplyCompensation(-turn_speed));
    }
}

/**
 * @brief 判断转向是否稳定到达目标角度
 * @param target_heading 目标航向
 * @param current_heading 当前航向
 * @param now_tick 当前系统时刻
 * @retval 1-稳定到位，0-未到位
 */
static uint8_t Car_Task_FieldTurnStable(float target_heading, float current_heading, uint32_t now_tick)
{
    float angle_error = Car_Task_CalcAngleError(target_heading, current_heading);

    if (fabsf(angle_error) <= FIELD_SCAN_ANGLE_TOLERANCE_DEG) {
        if (field_turn_stable_tick == 0U) {
            field_turn_stable_tick = now_tick;
        }
        if ((now_tick - field_turn_stable_tick) >= FIELD_SCAN_ANGLE_STABLE_MS) {
            return 1U;
        }
    } else {
        field_turn_stable_tick = 0U;
    }

    return 0U;
}

/**
 * @brief 进入水田遍历故障状态
 * @param fault 故障码
 * @retval 无
 */
static void Car_Task_FieldEnterFault(FieldScanFault fault)
{
    field_scan_state = FIELD_SCAN_STATE_FAULT;
    field_scan_fault = fault;
    Car_Task_SetRearMotorSpeed(0, 0);
    Car_Task_ResetFieldPid();
}

/**
 * @brief 执行水田遍历任务
 * @retval 无
 */
static void Car_Task_RunFieldScanTask(void)
{
    uint32_t now_tick;
    float current_heading;
    float dt_s;

    now_tick = HAL_GetTick();
    Car_Task_UpdateGyro();

    current_voltage = Car_Task_ReadVoltage();
    voltage_compensation = Car_Task_CalcCompensation(current_voltage);
    current_heading = Car_Task_NormalizeAngle(fAngle[2]);

    if (field_last_control_tick == 0U) {
        dt_s = 0.005f;
    } else {
        uint32_t dt_ms = now_tick - field_last_control_tick;
        if (dt_ms == 0U) {
            dt_ms = 1U;
        } else if (dt_ms > 100U) {
            dt_ms = 100U;
        }
        dt_s = (float)dt_ms / 1000.0f;
    }
    field_last_control_tick = now_tick;

    Car_Task_UpdateFieldTof(now_tick);

    if (field_tof_ready && !field_tof_valid &&
        ((now_tick - field_last_tof_valid_tick) >= FIELD_SCAN_TOF_FAULT_TIMEOUT_MS)) {
        Car_Task_FieldEnterFault(FIELD_SCAN_FAULT_TOF_TIMEOUT);
    }

    switch (field_scan_state) {
        case FIELD_SCAN_STATE_FORWARD:
            field_current_turn_target = field_current_row_heading;
            if (field_tof_sample_updated && field_tof_valid &&
                (field_tof_distance_mm <= FIELD_SCAN_OBSTACLE_MM)) {
                field_scan_state = FIELD_SCAN_STATE_OBSTACLE_CHECK;
                field_obstacle_confirm_count = 1U;
                Car_Task_SetRearMotorSpeed(0, 0);
                Car_Task_ResetFieldPid();
            } else {
                Car_Task_RunFieldHeadingHold(field_current_row_heading, current_heading, dt_s, FIELD_SCAN_FORWARD_PWM);
            }
            break;

        case FIELD_SCAN_STATE_OBSTACLE_CHECK:
            field_current_turn_target = field_current_row_heading;
            Car_Task_SetRearMotorSpeed(0, 0);
            if (field_tof_sample_updated && field_tof_valid) {
                if (field_tof_distance_mm <= FIELD_SCAN_OBSTACLE_MM) {
                    field_obstacle_confirm_count++;
                    if (field_obstacle_confirm_count >= FIELD_SCAN_CONFIRM_COUNT) {
                        field_scan_state = FIELD_SCAN_STATE_TURN_TO_SHIFT;
                        field_current_turn_target = field_heading_shift;
                        field_turn_stable_tick = 0U;
                        Car_Task_ResetFieldPid();
                    }
                } else {
                    field_scan_state = FIELD_SCAN_STATE_FORWARD;
                    field_obstacle_confirm_count = 0U;
                    Car_Task_ResetFieldPid();
                }
            }
            break;

        case FIELD_SCAN_STATE_TURN_TO_SHIFT:
            field_current_turn_target = field_heading_shift;
            if (Car_Task_FieldTurnStable(field_current_turn_target, current_heading, now_tick)) {
                field_scan_state = FIELD_SCAN_STATE_SHIFT_FORWARD;
                field_shift_start_tick = now_tick;
                Car_Task_SetRearMotorSpeed(0, 0);
                Car_Task_ResetFieldPid();
            } else {
                Car_Task_RunFieldTurn(field_current_turn_target, current_heading);
            }
            break;

        case FIELD_SCAN_STATE_SHIFT_FORWARD:
            field_current_turn_target = field_heading_shift;
            Car_Task_RunFieldHeadingHold(field_current_turn_target, current_heading, dt_s, FIELD_SCAN_SHIFT_PWM);
            if ((now_tick - field_shift_start_tick) >= FIELD_SCAN_SIDESTEP_TIME_MS) {
                field_scan_state = FIELD_SCAN_STATE_TURN_TO_ROW;
                field_current_turn_target = (field_row_index == 0U) ? field_heading_row_b : field_heading_row_a;
                field_turn_stable_tick = 0U;
                Car_Task_ResetFieldPid();
            }
            break;

        case FIELD_SCAN_STATE_TURN_TO_ROW:
            field_current_turn_target = (field_row_index == 0U) ? field_heading_row_b : field_heading_row_a;
            if (Car_Task_FieldTurnStable(field_current_turn_target, current_heading, now_tick)) {
                field_row_index ^= 1U;
                field_current_row_heading = (field_row_index == 0U) ? field_heading_row_a : field_heading_row_b;
                field_current_turn_target = field_current_row_heading;
                field_scan_state = FIELD_SCAN_STATE_FORWARD;
                field_obstacle_confirm_count = 0U;
                Car_Task_SetRearMotorSpeed(0, 0);
                Car_Task_ResetFieldPid();
            } else {
                Car_Task_RunFieldTurn(field_current_turn_target, current_heading);
            }
            break;

        case FIELD_SCAN_STATE_FAULT:
            Car_Task_SetRearMotorSpeed(0, 0);
            break;

        case FIELD_SCAN_STATE_IDLE:
        default:
            Car_Task_SetRearMotorSpeed(0, 0);
            break;
    }

    Car_Task_DisplayFieldInfo(current_heading);
}

/**
 * @brief 显示水田遍历任务运行信息
 * @param current_heading 当前航向
 * @retval 无
 */
static void Car_Task_DisplayFieldInfo(float current_heading)
{
    char str[24];

    OLED_Clear(0);
    OLED_ShowString(0, 0, "Field Task", 16);

    sprintf(str, "S:%s R:%u", Car_Task_GetFieldStateText(field_scan_state), (unsigned int)field_row_index);
    OLED_ShowString(0, 2, str, 12);

    sprintf(str, "Y:%5.1f T:%5.1f", current_heading, field_current_turn_target);
    OLED_ShowString(0, 3, str, 12);

    if (field_tof_valid) {
        sprintf(str, "TOF:%4u mm", (unsigned int)field_tof_distance_mm);
    } else {
        sprintf(str, "TOF:%s", Car_Task_GetFieldFaultText(field_scan_fault));
    }
    OLED_ShowString(0, 4, str, 12);

    sprintf(str, "L:%4d R:%4d", field_last_left_speed, field_last_right_speed);
    OLED_ShowString(0, 5, str, 12);

    sprintf(str, "V:%3.1f C:%1.2f", current_voltage, voltage_compensation);
    OLED_ShowString(0, 6, str, 12);

    OLED_Refresh();
}

/**
 * @brief 获取水田遍历状态字符串
 * @param state 当前状态
 * @retval 状态字符串
 */
static const char *Car_Task_GetFieldStateText(FieldScanState state)
{
    switch (state) {
        case FIELD_SCAN_STATE_FORWARD:
            return "FWD";
        case FIELD_SCAN_STATE_OBSTACLE_CHECK:
            return "CHK";
        case FIELD_SCAN_STATE_TURN_TO_SHIFT:
            return "TRN1";
        case FIELD_SCAN_STATE_SHIFT_FORWARD:
            return "SHIFT";
        case FIELD_SCAN_STATE_TURN_TO_ROW:
            return "TRN2";
        case FIELD_SCAN_STATE_FAULT:
            return "FAULT";
        case FIELD_SCAN_STATE_IDLE:
        default:
            return "IDLE";
    }
}

/**
 * @brief 获取水田遍历故障字符串
 * @param fault 当前故障
 * @retval 故障字符串
 */
static const char *Car_Task_GetFieldFaultText(FieldScanFault fault)
{
    switch (fault) {
        case FIELD_SCAN_FAULT_TOF_INIT:
            return "TOF_INIT";
        case FIELD_SCAN_FAULT_TOF_TIMEOUT:
            return "TOF_LOSS";
        case FIELD_SCAN_FAULT_NONE:
        default:
            return "OK";
    }
}

/**
 * @brief 小车任务初始化
 * @retval 无
 */
void Car_Task_Init(void)
{
    Motor_Init();
    Car_Task_InitADC();
    Car_Task_ResetStraightPid();
    Car_Task_ResetFieldRuntime();

    if (car_task_selected_mode == CAR_TASK_MODE_FIELD_SCAN) {
        Gryo_init();
        current_gyro_type = GYRO_TYPE_WIT;
        field_tof_ready = (VL53L0X_Init() == 0U) ? 1U : 0U;
    } else {
        if (ICM42670_Init()) {
            current_gyro_type = GYRO_TYPE_ICM42670;
            printf("ICM42670 initialized successfully!\r\n");
        } else {
            Gryo_init();
            current_gyro_type = GYRO_TYPE_WIT;
            printf("WIT gyro initialized!\r\n");
        }
        field_tof_ready = 0U;
    }

    car_task_running = 0U;
    car_task_active_mode = car_task_selected_mode;
    Car_Task_StopAllMotors();
}

/**
 * @brief 小车任务开始
 * @retval 无
 */
void Car_Task_Start(void)
{
    car_task_running = 1U;
    car_task_active_mode = car_task_selected_mode;
    Car_Task_StopAllMotors();

    if (car_task_active_mode == CAR_TASK_MODE_FIELD_SCAN) {
        Car_Task_UpdateGyro();
        Car_Task_ResetFieldRuntime();

        field_heading_row_a = Car_Task_NormalizeAngle(fAngle[2]);
        field_heading_shift = Car_Task_NormalizeAngle(field_heading_row_a + 90.0f);
        field_heading_row_b = Car_Task_NormalizeAngle(field_heading_row_a + 180.0f);
        field_current_row_heading = field_heading_row_a;
        field_current_turn_target = field_current_row_heading;
        field_scan_state = FIELD_SCAN_STATE_FORWARD;
        field_last_tof_valid_tick = HAL_GetTick();

        if (!field_tof_ready) {
            field_tof_ready = (VL53L0X_Init() == 0U) ? 1U : 0U;
        }
        if (!field_tof_ready) {
            Car_Task_FieldEnterFault(FIELD_SCAN_FAULT_TOF_INIT);
        }

        printf("Field task started: rowA=%.1f shift=%.1f rowB=%.1f\r\n",
               field_heading_row_a, field_heading_shift, field_heading_row_b);
    } else {
        Car_Task_ResetStraightPid();
        printf("Straight task started!\r\n");
    }
}

/**
 * @brief 小车任务停止
 * @retval 无
 */
void Car_Task_Stop(void)
{
    car_task_running = 0U;
    Car_Task_StopAllMotors();
    Car_Task_ResetStraightPid();
    Car_Task_ResetFieldRuntime();
    printf("Car task stopped!\r\n");
}

/**
 * @brief 小车任务主函数
 * @retval 无
 */
void Car_Task_Main(void)
{
    if (!car_task_running) {
        return;
    }

    if (car_task_active_mode == CAR_TASK_MODE_FIELD_SCAN) {
        Car_Task_RunFieldScanTask();
    } else {
        Car_Task_RunStraightTask();
    }
}

/**
 * @brief 获取小车任务运行状态
 * @retval 运行状态：1-运行中，0-停止
 */
uint8_t Car_Task_IsRunning(void)
{
    return car_task_running;
}

/**
 * @brief 设置小车目标角度
 * @param angle 目标角度（度）
 * @retval 无
 */
void Car_Task_SetTargetAngle(float angle)
{
    target_angle = angle;
}

/**
 * @brief 设置小车最大速度
 * @param speed 最大速度（-300到300）
 * @retval 无
 */
void Car_Task_SetMaxSpeed(int16_t speed)
{
    if (speed < 0) {
        speed = -speed;
    }

    if (speed > FIELD_SCAN_MAX_PWM) {
        max_speed = FIELD_SCAN_MAX_PWM;
    } else {
        max_speed = speed;
    }
}

/**
 * @brief 设置当前准备启动的小车任务模式
 * @param mode 任务模式
 * @retval 无
 */
void Car_Task_SetMode(CarTaskModeTypeDef mode)
{
    if (car_task_running) {
        return;
    }
    car_task_selected_mode = mode;
}

/**
 * @brief 获取当前选中的小车任务模式
 * @retval 任务模式
 */
CarTaskModeTypeDef Car_Task_GetMode(void)
{
    return car_task_selected_mode;
}
