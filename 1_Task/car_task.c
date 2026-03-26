#include "car_task.h"
#include "PID.h"
#include "ax_ps2.h"
#include "encoder.h"
#include "gyro_collision.h"
#include "icm42670.h"
#include "motor_tb6612.h"
#include "oled.h"
#include "wit_gyro_sdk.h"
#include "main.h"
// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_adc.h"

#include <math.h>
#include <stdio.h>

/* ========= 用户可调参数区 ========= */
/* 电压补偿参数 */
#define REFERENCE_VOLTAGE                8.4f      /* 电压补偿参考值（V），用于计算 PWM 放大/缩小比例 */
#define ADC_DIVIDER_RATIO                4.0f      /* 电压分压比：电池电压 = ADC引脚电压 * 该系数 */
#define DEBUG_SIMULATE_VOLTAGE           8.4f      /* 调试模拟电压（V），>0 时跳过真实 ADC 采样 */

/* 直行模式参数 */
#define STRAIGHT_TASK_BASE_SPEED         150       /* 直行模式基础速度（PWM） */
#define STRAIGHT_PID_KP                  2.2f      /* 直行航向 PID 比例系数 */
#define STRAIGHT_PID_KI                  0.02f     /* 直行航向 PID 积分系数 */
#define STRAIGHT_PID_KD                  0.12f     /* 直行航向 PID 微分系数 */
#define STRAIGHT_PID_OUT_MAX             120.0f    /* 直行航向 PID 输出上限（PWM） */
#define STRAIGHT_PID_OUT_MIN             (-120.0f) /* 直行航向 PID 输出下限（PWM） */

/* 弓字遍历：速度与运动参数 */
#define FIELD_SCAN_FORWARD_PWM           180       /* 弓字遍历直行阶段基础速度（PWM） */
#define FIELD_SCAN_SHIFT_PWM             170       /* 弓字遍历横移阶段基础速度（PWM） */
#define FIELD_SCAN_TURN_PWM              160       /* 弓字遍历原地转向速度（PWM） */
#define FIELD_SCAN_MAX_PWM               300       /* 弓字遍历模式速度总限幅（PWM） */

/* 弓字遍历：里程参数（编码器累计计数，需按实机轮径与减速比标定） */
#define FIELD_SCAN_ROW_DISTANCE_TICKS    9000U     /* 单行前进里程阈值，达到后触发掉头流程 */
#define FIELD_SCAN_SHIFT_DISTANCE_TICKS  1800U     /* 横移里程阈值，达到后触发第二次转向 */

/* 弓字遍历：转向容差与状态切换门限 */
#define FIELD_SCAN_ANGLE_TOLERANCE_DEG   3.0f      /* 转向到位角度容差（度） */
#define FIELD_SCAN_ANGLE_STABLE_MS       120U      /* 角度进入容差后需持续稳定时间（ms） */

/* 弓字遍历：航向 PID 参数 */
#define FIELD_PID_KP                     2.6f      /* 弓字遍历航向 PID 比例系数 */
#define FIELD_PID_KI                     0.03f     /* 弓字遍历航向 PID 积分系数 */
#define FIELD_PID_KD                     0.20f     /* 弓字遍历航向 PID 微分系数 */
#define FIELD_PID_OUT_MAX                140.0f    /* 弓字遍历航向 PID 输出上限（PWM） */
#define FIELD_PID_OUT_MIN                (-140.0f) /* 弓字遍历航向 PID 输出下限（PWM） */

/* 弓字遍历：左右轮速度 PID 参数（编码器闭环） */
#define FIELD_SPEED_TARGET_PER_PWM       2.7f      /* 目标速度映射：1PWM 约等效的编码器计数/秒 */
#define FIELD_SPEED_PID_KP               0.12f     /* 轮速环比例系数 */
#define FIELD_SPEED_PID_KI               0.01f     /* 轮速环积分系数 */
#define FIELD_SPEED_PID_KD               0.00f     /* 轮速环微分系数 */
#define FIELD_SPEED_PID_OUT_MAX          300.0f    /* 轮速环输出上限（PWM） */
#define FIELD_SPEED_PID_OUT_MIN          (-300.0f) /* 轮速环输出下限（PWM） */
#define FIELD_ENCODER_LEFT_SIGN          1.0f      /* 左编码器方向系数（方向反了改为 -1） */
#define FIELD_ENCODER_RIGHT_SIGN         1.0f      /* 右编码器方向系数（方向反了改为 -1） */

/* 远程控制：输入映射与速度环参数 */
#define REMOTE_CTRL_PERIOD_MS            20U       /* 远程控制循环周期（ms） */
#define REMOTE_STICK_DEADZONE            6         /* 右摇杆附加死区（基于去死区后残余抖动） */
#define REMOTE_TARGET_SPEED_MAX          800.0f    /* 目标速度上限（编码器计数/秒） */
#define REMOTE_TARGET_TURN_MAX           500.0f    /* 目标转向分量上限（编码器计数/秒） */
#define REMOTE_ENCODER_LEFT_SIGN         1.0f      /* 左编码器方向系数（方向反了改为 -1） */
#define REMOTE_ENCODER_RIGHT_SIGN        1.0f      /* 右编码器方向系数（方向反了改为 -1） */

#define REMOTE_SPEED_PID_KP              0.18f     /* 速度环 PID 比例系数 */
#define REMOTE_SPEED_PID_KI              0.02f     /* 速度环 PID 积分系数 */
#define REMOTE_SPEED_PID_KD              0.00f     /* 速度环 PID 微分系数 */
#define REMOTE_SPEED_PID_OUT_MAX         300.0f    /* 速度环 PID 输出上限（PWM） */
#define REMOTE_SPEED_PID_OUT_MIN         (-300.0f) /* 速度环 PID 输出下限（PWM） */

#define REMOTE_PS2_MODE_DIGITAL          0x41U
#define REMOTE_PS2_MODE_ANALOG_RED       0x73U
#define REMOTE_PS2_MODE_ANALOG_PRESSURE  0x79U
#define REMOTE_PS2_REINIT_INVALID_COUNT  15U

/**
 * @brief 弓字遍历状态机状态定义
 */
typedef enum {
    FIELD_SCAN_STATE_IDLE = 0,         /* 空闲态：任务未启动或已停止 */
    FIELD_SCAN_STATE_FORWARD,          /* 行进态：沿当前作业行保持航向前进 */
    FIELD_SCAN_STATE_TURN_TO_SHIFT,    /* 第一次转向态：转到固定横移航向 */
    FIELD_SCAN_STATE_SHIFT_FORWARD,    /* 横移态：按编码器里程侧向移动 */
    FIELD_SCAN_STATE_TURN_TO_NEXT_ROW  /* 第二次转向态：转到下一作业行航向 */
} FieldScanState;

typedef enum {
    CAR_TASK_GYRO_NONE = 0,
    CAR_TASK_GYRO_ICM42670,
    CAR_TASK_GYRO_WIT
} CarTaskGyroType;

static uint8_t car_task_running = 0U;                                     /* 任务运行标志：1运行，0停止 */
static CarTaskModeTypeDef car_task_selected_mode = CAR_TASK_MODE_STRAIGHT;/* 菜单选中的任务模式 */
static CarTaskModeTypeDef car_task_active_mode = CAR_TASK_MODE_STRAIGHT;  /* 当前已启动的任务模式 */
static CarTaskGyroType car_task_gyro_type = CAR_TASK_GYRO_NONE;           /* 当前任务陀螺仪来源 */

static float target_angle = 30.0f;                                        /* 直行模式目标航向角（度） */
static int16_t max_speed = FIELD_SCAN_MAX_PWM;                            /* 全局速度上限（PWM） */

static ADC_HandleTypeDef hadc1;                                           /* 电池电压采样 ADC 句柄 */
static uint8_t adc_initialized = 0U;                                      /* ADC 初始化完成标志 */
static float current_voltage = 0.0f;                                      /* 当前电池电压（V） */
static float voltage_compensation = 1.0f;                                 /* 电压补偿系数 */

static PID_t straight_heading_pid;                                        /* 直行模式航向 PID 控制器 */
static float straight_pid_output = 0.0f;                                  /* 直行模式航向 PID 输出 */

static PID_t field_heading_pid;                                           /* 弓字遍历航向 PID 控制器 */
static float field_pid_output = 0.0f;                                     /* 弓字遍历航向 PID 输出 */
static PID_t field_left_speed_pid;                                         /* 弓字遍历左轮速度 PID */
static PID_t field_right_speed_pid;                                        /* 弓字遍历右轮速度 PID */
static float field_target_left_speed = 0.0f;                               /* 弓字遍历左轮目标速度（计数/秒） */
static float field_target_right_speed = 0.0f;                              /* 弓字遍历右轮目标速度（计数/秒） */
static float field_measured_left_speed = 0.0f;                             /* 弓字遍历左轮实测速度（计数/秒） */
static float field_measured_right_speed = 0.0f;                            /* 弓字遍历右轮实测速度（计数/秒） */

static uint8_t field_row_index = 0U;                                      /* 当前作业行索引（0/1 交替） */
static uint32_t field_segment_distance_ticks = 0U;                        /* 当前阶段累计里程（编码器计数） */
static uint32_t field_segment_target_ticks = FIELD_SCAN_ROW_DISTANCE_TICKS;/* 当前阶段目标里程（编码器计数） */

static uint32_t field_last_control_tick = 0U;                             /* 最近一次控制时刻（ms节拍） */
static uint32_t field_turn_stable_tick = 0U;                              /* 转向进入容差后的稳定起点 */

static float field_heading_row_a = 0.0f;                                  /* A 行目标航向 */
static float field_heading_row_b = 180.0f;                                /* B 行目标航向 */
static float field_heading_shift = 90.0f;                                 /* 横移阶段目标航向 */
static float field_current_row_heading = 0.0f;                            /* 当前前进行目标航向 */
static float field_current_turn_target = 0.0f;                            /* 当前转向目标航向 */

static int16_t field_last_left_speed = 0;                                 /* 最近一次后左轮输出 PWM */
static int16_t field_last_right_speed = 0;                                /* 最近一次后右轮输出 PWM */

static FieldScanState field_scan_state = FIELD_SCAN_STATE_IDLE;           /* 弓字遍历状态机当前状态 */

static JOYSTICK_TypeDef remote_joystick;                                   /* PS2 手柄数据缓存 */
static PID_t remote_left_speed_pid;                                        /* 远程模式左轮速度环 PID */
static PID_t remote_right_speed_pid;                                       /* 远程模式右轮速度环 PID */

static uint8_t remote_ps2_ready = 0U;                                      /* PS2 是否已完成初始化 */
static uint8_t task_encoder_ready = 0U;                                    /* 编码器是否已完成初始化（弓字/遥控共用） */
static uint8_t remote_ps2_frame_valid = 0U;
static uint8_t remote_ps2_invalid_count = 0U;
static uint32_t remote_last_ctrl_tick = 0U;                                /* 远程模式上次控制时刻 */

static float remote_target_left_speed = 0.0f;                              /* 左轮目标速度（计数/秒） */
static float remote_target_right_speed = 0.0f;                             /* 右轮目标速度（计数/秒） */
static float remote_measured_left_speed = 0.0f;                            /* 左轮实测速度（计数/秒） */
static float remote_measured_right_speed = 0.0f;                           /* 右轮实测速度（计数/秒） */

static int16_t remote_left_pwm = 0;                                        /* 左轮当前 PWM 输出 */
static int16_t remote_right_pwm = 0;                                       /* 右轮当前 PWM 输出 */

static uint8_t Car_Task_InitADC(void);
static float Car_Task_ReadVoltage(void);
static float Car_Task_CalcCompensation(float voltage);

static void Car_Task_UpdateGyro(void);
static void Car_Task_ResetStraightPid(void);
static void Car_Task_ResetFieldPid(void);
static void Car_Task_ResetFieldRuntime(void);
static void Car_Task_ResetRemoteRuntime(void);

static void Car_Task_StopAllMotors(void);
static float Car_Task_NormalizeAngle(float angle);
static float Car_Task_CalcAngleError(float target, float current);

static int16_t Car_Task_ClampSpeed(int32_t speed);
static int16_t Car_Task_ApplyCompensation(int16_t speed);
static void Car_Task_SetRearMotorSpeed(int16_t left_speed, int16_t right_speed);
static void Car_Task_GetFieldEncoderStep(int32_t *left_delta, int32_t *right_delta, uint32_t *step_abs_avg);
static void Car_Task_RunFieldSpeedLoop(float target_left_speed, float target_right_speed,
                                       int32_t enc_left_delta, int32_t enc_right_delta, float dt_s);

static float Car_Task_RunHeadingPid(PID_t *pid, float target_heading, float current_heading);

static void Car_Task_RunStraightTask(void);
static void Car_Task_RunFieldHeadingHold(float target_heading, float current_heading, int16_t base_speed,
                                         int32_t enc_left_delta, int32_t enc_right_delta, float dt_s);
static void Car_Task_RunFieldTurn(float target_heading, float current_heading,
                                  int32_t enc_left_delta, int32_t enc_right_delta, float dt_s);
static uint8_t Car_Task_FieldTurnStable(float target_heading, float current_heading, uint32_t now_tick);
static void Car_Task_RunFieldScanTask(void);
static void Car_Task_RunRemoteTask(void);
static uint8_t Car_Task_IsRemotePs2FrameValid(const JOYSTICK_TypeDef *joystick);

static void Car_Task_DisplayFieldInfo(float current_heading);
static void Car_Task_DisplayRemoteInfo(void);
static void Car_Task_DrawStickBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t value);
static const char *Car_Task_GetFieldStateText(FieldScanState state);

/**
 * @brief 初始化电压采样 ADC（PC4）
 * @retval 1 初始化成功，0 初始化失败
 */
static uint8_t Car_Task_InitADC(void)
{
    GPIO_InitTypeDef gpio_init;
    ADC_ChannelConfTypeDef adc_config;

    if (adc_initialized != 0U) {
        return 1U;
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    gpio_init.Pin = GPIO_PIN_4;
    gpio_init.Mode = GPIO_MODE_ANALOG;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
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

    adc_config.Channel = ADC_CHANNEL_14;
    adc_config.Rank = 1;
    adc_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK) {
        return 0U;
    }

    adc_initialized = 1U;
    return 1U;
}

/**
 * @brief 读取当前电池电压
 * @retval 电池电压，单位 V
 */
static float Car_Task_ReadVoltage(void)
{
    uint32_t adc_value;

    if (DEBUG_SIMULATE_VOLTAGE > 0.0f) {
        return DEBUG_SIMULATE_VOLTAGE;
    }

    if (adc_initialized == 0U) {
        return 0.0f;
    }

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return ((float)adc_value * 3.3f / 4095.0f) * ADC_DIVIDER_RATIO;
}

/**
 * @brief 根据当前电压计算 PWM 补偿系数
 * @param voltage 当前电压（V）
 * @retval 补偿系数（无量纲）
 */
static float Car_Task_CalcCompensation(float voltage)
{
    if ((voltage < 6.0f) || (voltage > 10.0f)) {
        return 1.0f;
    }

    return REFERENCE_VOLTAGE / voltage;
}

/**
 * @brief 更新姿态角数据（ICM42670 优先，WIT 兜底）
 */
static void Car_Task_UpdateGyro(void)
{
    if (car_task_gyro_type == CAR_TASK_GYRO_ICM42670) {
        ICM42670_Gryo_Update();
        Gyro_Filter_Update(fAcc, fGyro, fAngle, &fYaw);
    } else {
        Gryo_Update();
    }
}

/**
 * @brief 复位直行模式 PID 内部状态
 */
static void Car_Task_ResetStraightPid(void)
{
    PID_Reset(&straight_heading_pid);
    straight_pid_output = 0.0f;
}

/**
 * @brief 复位弓字遍历模式 PID 内部状态
 */
static void Car_Task_ResetFieldPid(void)
{
    PID_Reset(&field_heading_pid);
    PID_Reset(&field_left_speed_pid);
    PID_Reset(&field_right_speed_pid);
    field_pid_output = 0.0f;
    field_target_left_speed = 0.0f;
    field_target_right_speed = 0.0f;
    field_measured_left_speed = 0.0f;
    field_measured_right_speed = 0.0f;
}

/**
 * @brief 复位弓字遍历运行时状态变量
 */
static void Car_Task_ResetFieldRuntime(void)
{
    field_row_index = 0U;
    field_segment_distance_ticks = 0U;
    field_segment_target_ticks = FIELD_SCAN_ROW_DISTANCE_TICKS;

    field_last_control_tick = 0U;
    field_turn_stable_tick = 0U;

    field_heading_row_a = 0.0f;
    field_heading_row_b = 180.0f;
    field_heading_shift = 90.0f;
    field_current_row_heading = 0.0f;
    field_current_turn_target = 0.0f;

    field_last_left_speed = 0;
    field_last_right_speed = 0;
    field_target_left_speed = 0.0f;
    field_target_right_speed = 0.0f;
    field_measured_left_speed = 0.0f;
    field_measured_right_speed = 0.0f;

    field_scan_state = FIELD_SCAN_STATE_IDLE;

    Car_Task_ResetFieldPid();
}

/**
 * @brief 复位远程控制运行时状态变量
 */
static void Car_Task_ResetRemoteRuntime(void)
{
    remote_last_ctrl_tick = 0U;

    remote_joystick.mode = 0U;
    remote_joystick.btn1 = 0U;
    remote_joystick.btn2 = 0U;
    remote_joystick.RJoy_LR = 128U;
    remote_joystick.RJoy_UD = 128U;
    remote_joystick.LJoy_LR = 128U;
    remote_joystick.LJoy_UD = 128U;

    remote_target_left_speed = 0.0f;
    remote_target_right_speed = 0.0f;
    remote_measured_left_speed = 0.0f;
    remote_measured_right_speed = 0.0f;

    remote_left_pwm = 0;
    remote_right_pwm = 0;
    remote_ps2_frame_valid = 0U;
    remote_ps2_invalid_count = 0U;

    PID_Reset(&remote_left_speed_pid);
    PID_Reset(&remote_right_speed_pid);
}

/**
 * @brief 停止全部电机并清空输出缓存
 */
static void Car_Task_StopAllMotors(void)
{
    Motor_StopAll();
    field_last_left_speed = 0;
    field_last_right_speed = 0;
    remote_left_pwm = 0;
    remote_right_pwm = 0;
}

/**
 * @brief 将角度归一化到 (-180, 180] 范围
 * @param angle 原始角度（度）
 * @retval 归一化后的角度（度）
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
 * @brief 计算目标角与当前角之间的最短角度误差
 * @param target 目标航向角（度）
 * @param current 当前航向角（度）
 * @retval 最短误差（度）
 */
static float Car_Task_CalcAngleError(float target, float current)
{
    return Car_Task_NormalizeAngle(target - current);
}

/**
 * @brief 对速度进行全局限幅
 * @param speed 输入速度（PWM）
 * @retval 限幅后速度（PWM）
 */
static int16_t Car_Task_ClampSpeed(int32_t speed)
{
    if (speed > (int32_t)max_speed) {
        speed = (int32_t)max_speed;
    } else if (speed < -(int32_t)max_speed) {
        speed = -(int32_t)max_speed;
    }

    return (int16_t)speed;
}

/**
 * @brief 对速度施加电压补偿并限幅
 * @param speed 输入速度（PWM）
 * @retval 补偿并限幅后速度（PWM）
 */
static int16_t Car_Task_ApplyCompensation(int16_t speed)
{
    float compensated = (float)speed * voltage_compensation;
    return Car_Task_ClampSpeed((int32_t)compensated);
}

/**
 * @brief 仅设置后轮速度，前轮固定为 0
 * @param left_speed 后左轮速度（PWM）
 * @param right_speed 后右轮速度（PWM）
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
 * @brief 读取弓字遍历本周期编码器增量并给出绝对值均值
 * @param left_delta 左轮增量（带符号）
 * @param right_delta 右轮增量（带符号）
 * @param step_abs_avg 双轮绝对值均值（用于里程累计）
 */
static void Car_Task_GetFieldEncoderStep(int32_t *left_delta, int32_t *right_delta, uint32_t *step_abs_avg)
{
    int32_t left_abs;
    int32_t right_abs;

    if ((left_delta == NULL) || (right_delta == NULL) || (step_abs_avg == NULL)) {
        return;
    }

    *left_delta = Encoder_GetCount(ENCODER_1);
    *right_delta = Encoder_GetCount(ENCODER_2);

    left_abs = *left_delta;
    right_abs = *right_delta;
    if (left_abs < 0) {
        left_abs = -left_abs;
    }
    if (right_abs < 0) {
        right_abs = -right_abs;
    }

    *step_abs_avg = (uint32_t)(((uint32_t)left_abs + (uint32_t)right_abs) / 2U);
}

/**
 * @brief 弓字遍历轮速闭环：编码器速度反馈 -> 左右轮 PWM
 * @param target_left_speed 左轮目标速度（计数/秒）
 * @param target_right_speed 右轮目标速度（计数/秒）
 * @param enc_left_delta 左轮编码器增量
 * @param enc_right_delta 右轮编码器增量
 * @param dt_s 控制周期（秒）
 */
static void Car_Task_RunFieldSpeedLoop(float target_left_speed, float target_right_speed,
                                       int32_t enc_left_delta, int32_t enc_right_delta, float dt_s)
{
    int16_t left_cmd;
    int16_t right_cmd;

    if (dt_s <= 0.0001f) {
        dt_s = 0.005f;
    }

    field_target_left_speed = target_left_speed;
    field_target_right_speed = target_right_speed;

    field_measured_left_speed = FIELD_ENCODER_LEFT_SIGN * ((float)enc_left_delta / dt_s);
    field_measured_right_speed = FIELD_ENCODER_RIGHT_SIGN * ((float)enc_right_delta / dt_s);

    left_cmd = Car_Task_ApplyCompensation(
        (int16_t)PID_Calc(&field_left_speed_pid, field_measured_left_speed, field_target_left_speed));
    right_cmd = Car_Task_ApplyCompensation(
        (int16_t)PID_Calc(&field_right_speed_pid, field_measured_right_speed, field_target_right_speed));

    Car_Task_SetRearMotorSpeed(left_cmd, right_cmd);
}

/**
 * @brief 执行航向 PID 计算
 * @param pid PID 控制器实例
 * @param target_heading 目标航向角（度）
 * @param current_heading 当前航向角（度）
 * @retval PID 输出（用于左右差速）
 */
static float Car_Task_RunHeadingPid(PID_t *pid, float target_heading, float current_heading)
{
    float angle_error;
    angle_error = Car_Task_CalcAngleError(target_heading, current_heading);
    /* PID_Calc 内部误差是“目标值减测量值”，这里传入 -angle_error 以得到期望方向。 */
    return PID_Calc(pid, -angle_error, 0.0f);
}

/**
 * @brief 执行直行模式主控制
 * @note 读取航向后通过 PID 做左右差速，四轮同速输出
 */
static void Car_Task_RunStraightTask(void)
{
    int16_t left_speed;
    int16_t right_speed;
    float current_heading;
    float angle_error;
    char str[24];

    Car_Task_UpdateGyro();

    current_voltage = Car_Task_ReadVoltage();
    voltage_compensation = Car_Task_CalcCompensation(current_voltage);

    current_heading = Car_Task_NormalizeAngle(fAngle[2]);
    angle_error = Car_Task_CalcAngleError(target_angle, current_heading);
    straight_pid_output = PID_Calc(&straight_heading_pid, -angle_error, 0.0f);

    left_speed = Car_Task_ApplyCompensation((int16_t)(STRAIGHT_TASK_BASE_SPEED - straight_pid_output));
    right_speed = Car_Task_ApplyCompensation((int16_t)(STRAIGHT_TASK_BASE_SPEED + straight_pid_output));

    Motor_SetSpeed(MOTOR_FRONT_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_FRONT_RIGHT, right_speed);
    Motor_SetSpeed(MOTOR_BACK_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_BACK_RIGHT, right_speed);

    OLED_Clear(0);

    (void)snprintf(str, sizeof(str), "Y:%5.1f", current_heading);
    OLED_ShowString(0, 2, str, 12);
    (void)snprintf(str, sizeof(str), "T:%5.1f", target_angle);
    OLED_ShowString(64, 2, str, 12);

    (void)snprintf(str, sizeof(str), "L:%4d R:%4d", left_speed, right_speed);
    OLED_ShowString(0, 4, str, 12);

    (void)snprintf(str, sizeof(str), "V:%3.1f C:%1.2f", current_voltage, voltage_compensation);
    OLED_ShowString(0, 6, str, 12);

    OLED_Refresh();
}

/**
 * @brief 弓字遍历前进控制：航向闭环 + 基础前进 PWM
 * @param target_heading 目标航向角（度）
 * @param current_heading 当前航向角（度）
 * @param base_speed 基础前进速度（PWM）
 */
static void Car_Task_RunFieldHeadingHold(float target_heading, float current_heading, int16_t base_speed,
                                         int32_t enc_left_delta, int32_t enc_right_delta, float dt_s)
{
    float left_target_speed;
    float right_target_speed;

    field_pid_output = Car_Task_RunHeadingPid(&field_heading_pid, target_heading, current_heading);
    left_target_speed = ((float)base_speed - field_pid_output) * FIELD_SPEED_TARGET_PER_PWM;
    right_target_speed = ((float)base_speed + field_pid_output) * FIELD_SPEED_TARGET_PER_PWM;

    Car_Task_RunFieldSpeedLoop(left_target_speed, right_target_speed, enc_left_delta, enc_right_delta, dt_s);
}

/**
 * @brief 弓字遍历转向控制：后轮反向输出实现原地转向
 * @param target_heading 目标航向角（度）
 * @param current_heading 当前航向角（度）
 */
static void Car_Task_RunFieldTurn(float target_heading, float current_heading,
                                  int32_t enc_left_delta, int32_t enc_right_delta, float dt_s)
{
    float angle_error;
    int16_t turn_speed;
    float left_target_speed;
    float right_target_speed;

    angle_error = Car_Task_CalcAngleError(target_heading, current_heading);
    turn_speed = FIELD_SCAN_TURN_PWM;

    if (fabsf(angle_error) < 15.0f) {
        turn_speed = (int16_t)(FIELD_SCAN_TURN_PWM / 2);
        if (turn_speed < 80) {
            turn_speed = 80;
        }
    }

    if (angle_error >= 0.0f) {
        left_target_speed = (float)(-turn_speed) * FIELD_SPEED_TARGET_PER_PWM;
        right_target_speed = (float)turn_speed * FIELD_SPEED_TARGET_PER_PWM;
    } else {
        left_target_speed = (float)turn_speed * FIELD_SPEED_TARGET_PER_PWM;
        right_target_speed = (float)(-turn_speed) * FIELD_SPEED_TARGET_PER_PWM;
    }

    Car_Task_RunFieldSpeedLoop(left_target_speed, right_target_speed, enc_left_delta, enc_right_delta, dt_s);
}

/**
 * @brief 判断转向是否稳定到位
 * @param target_heading 目标航向角（度）
 * @param current_heading 当前航向角（度）
 * @param now_tick 当前系统节拍（ms）
 * @retval 1 到位且稳定，0 未稳定
 */
static uint8_t Car_Task_FieldTurnStable(float target_heading, float current_heading, uint32_t now_tick)
{
    float angle_error;

    angle_error = Car_Task_CalcAngleError(target_heading, current_heading);
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
 * @brief 执行弓字遍历状态机主控制
 * @note 状态流：
 *       1) 前进(FORWARD) 里程到阈值
 *       2) 第一次转向(TURN_TO_SHIFT)
 *       3) 横移(SHIFT_FORWARD) 里程到阈值
 *       4) 第二次转向(TURN_TO_NEXT_ROW)
 *       5) 回到前进(FORWARD)
 */
static void Car_Task_RunFieldScanTask(void)
{
    uint32_t now_tick;
    uint32_t dt_ms;
    float dt_s;
    int32_t enc_left_delta;
    int32_t enc_right_delta;
    uint32_t encoder_step_ticks;
    float current_heading;

    now_tick = HAL_GetTick();

    Car_Task_UpdateGyro();
    current_heading = Car_Task_NormalizeAngle(fAngle[2]);

    current_voltage = Car_Task_ReadVoltage();
    voltage_compensation = Car_Task_CalcCompensation(current_voltage);

    if (field_last_control_tick == 0U) {
        dt_ms = 5U;
    } else {
        dt_ms = now_tick - field_last_control_tick;
        if (dt_ms == 0U) {
            dt_ms = 1U;
        }
    }
    field_last_control_tick = now_tick;
    dt_s = (float)dt_ms / 1000.0f;
    Car_Task_GetFieldEncoderStep(&enc_left_delta, &enc_right_delta, &encoder_step_ticks);

    switch (field_scan_state) {
        case FIELD_SCAN_STATE_FORWARD:
            field_current_turn_target = field_current_row_heading;
            Car_Task_RunFieldHeadingHold(field_current_row_heading, current_heading, FIELD_SCAN_FORWARD_PWM,
                                         enc_left_delta, enc_right_delta, dt_s);

            field_segment_distance_ticks += encoder_step_ticks;
            if (field_segment_distance_ticks >= field_segment_target_ticks) {
                field_scan_state = FIELD_SCAN_STATE_TURN_TO_SHIFT;
                field_current_turn_target = field_heading_shift;
                field_turn_stable_tick = 0U;
                Car_Task_SetRearMotorSpeed(0, 0);
                Car_Task_ResetFieldPid();
            }
            break;

        case FIELD_SCAN_STATE_TURN_TO_SHIFT:
            field_current_turn_target = field_heading_shift;
            if (Car_Task_FieldTurnStable(field_current_turn_target, current_heading, now_tick) != 0U) {
                field_scan_state = FIELD_SCAN_STATE_SHIFT_FORWARD;
                field_segment_distance_ticks = 0U;
                field_segment_target_ticks = FIELD_SCAN_SHIFT_DISTANCE_TICKS;
                Car_Task_SetRearMotorSpeed(0, 0);
                Car_Task_ResetFieldPid();
            } else {
                Car_Task_RunFieldTurn(field_current_turn_target, current_heading, enc_left_delta, enc_right_delta, dt_s);
            }
            break;

        case FIELD_SCAN_STATE_SHIFT_FORWARD:
            field_current_turn_target = field_heading_shift;
            Car_Task_RunFieldHeadingHold(field_current_turn_target, current_heading, FIELD_SCAN_SHIFT_PWM,
                                         enc_left_delta, enc_right_delta, dt_s);

            field_segment_distance_ticks += encoder_step_ticks;
            if (field_segment_distance_ticks >= field_segment_target_ticks) {
                field_scan_state = FIELD_SCAN_STATE_TURN_TO_NEXT_ROW;
                field_current_turn_target = (field_row_index == 0U) ? field_heading_row_b : field_heading_row_a;
                field_turn_stable_tick = 0U;
                Car_Task_SetRearMotorSpeed(0, 0);
                Car_Task_ResetFieldPid();
            }
            break;

        case FIELD_SCAN_STATE_TURN_TO_NEXT_ROW:
            field_current_turn_target = (field_row_index == 0U) ? field_heading_row_b : field_heading_row_a;
            if (Car_Task_FieldTurnStable(field_current_turn_target, current_heading, now_tick) != 0U) {
                field_row_index ^= 1U;
                field_current_row_heading = (field_row_index == 0U) ? field_heading_row_a : field_heading_row_b;
                field_current_turn_target = field_current_row_heading;
                field_scan_state = FIELD_SCAN_STATE_FORWARD;
                field_segment_distance_ticks = 0U;
                field_segment_target_ticks = FIELD_SCAN_ROW_DISTANCE_TICKS;
                Car_Task_SetRearMotorSpeed(0, 0);
                Car_Task_ResetFieldPid();
            } else {
                Car_Task_RunFieldTurn(field_current_turn_target, current_heading, enc_left_delta, enc_right_delta, dt_s);
            }
            break;

        case FIELD_SCAN_STATE_IDLE:
        default:
            Car_Task_SetRearMotorSpeed(0, 0);
            Car_Task_ResetFieldPid();
            break;
    }

    Car_Task_DisplayFieldInfo(current_heading);
}

/**
 * @brief 执行远程控制任务（PS2 右摇杆速度验证）
 * @note 右摇杆映射为差速左右轮目标速度，编码器反馈进入速度环 PID
 */
static void Car_Task_RunRemoteTask(void)
{
    uint32_t now_tick;
    uint32_t dt_ms;
    float dt_s;
    int16_t stick_forward;
    int16_t stick_turn;
    int32_t enc_left_delta;
    int32_t enc_right_delta;
    int16_t left_cmd;
    int16_t right_cmd;

    now_tick = HAL_GetTick();

    if (remote_last_ctrl_tick == 0U) {
        remote_last_ctrl_tick = now_tick;
    }

    dt_ms = now_tick - remote_last_ctrl_tick;
    if (dt_ms < REMOTE_CTRL_PERIOD_MS) {
        Car_Task_DisplayRemoteInfo();
        return;
    }

    if (dt_ms > 200U) {
        (void)Encoder_GetCount(ENCODER_1);
        (void)Encoder_GetCount(ENCODER_2);

        PID_Reset(&remote_left_speed_pid);
        PID_Reset(&remote_right_speed_pid);

        remote_target_left_speed = 0.0f;
        remote_target_right_speed = 0.0f;
        remote_measured_left_speed = 0.0f;
        remote_measured_right_speed = 0.0f;
        remote_left_pwm = 0;
        remote_right_pwm = 0;

        Car_Task_SetRearMotorSpeed(0, 0);
        remote_last_ctrl_tick = now_tick;
        Car_Task_DisplayRemoteInfo();
        return;
    }
    remote_last_ctrl_tick = now_tick;
    dt_s = (float)dt_ms / 1000.0f;

    current_voltage = Car_Task_ReadVoltage();
    voltage_compensation = Car_Task_CalcCompensation(current_voltage);

    AX_PS2_ScanKey_Deadzone(&remote_joystick);
    remote_ps2_frame_valid = Car_Task_IsRemotePs2FrameValid(&remote_joystick);

    if (remote_ps2_frame_valid == 0U) {
        if (remote_ps2_invalid_count < 0xFFU) {
            remote_ps2_invalid_count++;
        }
        if (remote_ps2_invalid_count >= REMOTE_PS2_REINIT_INVALID_COUNT) {
            AX_PS2_SetInit();
            remote_ps2_invalid_count = 0U;
        }

        (void)Encoder_GetCount(ENCODER_1);
        (void)Encoder_GetCount(ENCODER_2);

        remote_joystick.RJoy_LR = 128U;
        remote_joystick.RJoy_UD = 128U;
        remote_joystick.LJoy_LR = 128U;
        remote_joystick.LJoy_UD = 128U;

        PID_Reset(&remote_left_speed_pid);
        PID_Reset(&remote_right_speed_pid);

        remote_target_left_speed = 0.0f;
        remote_target_right_speed = 0.0f;
        remote_measured_left_speed = 0.0f;
        remote_measured_right_speed = 0.0f;
        remote_left_pwm = 0;
        remote_right_pwm = 0;

        Car_Task_SetRearMotorSpeed(0, 0);
        Car_Task_DisplayRemoteInfo();
        return;
    }

    remote_ps2_invalid_count = 0U;

    stick_forward = (int16_t)128 - (int16_t)remote_joystick.RJoy_UD;
    stick_turn = (int16_t)remote_joystick.RJoy_LR - (int16_t)128;

    if ((stick_forward < REMOTE_STICK_DEADZONE) && (stick_forward > -REMOTE_STICK_DEADZONE)) {
        stick_forward = 0;
    }
    if ((stick_turn < REMOTE_STICK_DEADZONE) && (stick_turn > -REMOTE_STICK_DEADZONE)) {
        stick_turn = 0;
    }

    remote_target_left_speed = ((float)stick_forward * REMOTE_TARGET_SPEED_MAX / 127.0f) +
                               ((float)stick_turn * REMOTE_TARGET_TURN_MAX / 127.0f);
    remote_target_right_speed = ((float)stick_forward * REMOTE_TARGET_SPEED_MAX / 127.0f) -
                                ((float)stick_turn * REMOTE_TARGET_TURN_MAX / 127.0f);

    if (remote_target_left_speed > REMOTE_TARGET_SPEED_MAX) {
        remote_target_left_speed = REMOTE_TARGET_SPEED_MAX;
    } else if (remote_target_left_speed < -REMOTE_TARGET_SPEED_MAX) {
        remote_target_left_speed = -REMOTE_TARGET_SPEED_MAX;
    }

    if (remote_target_right_speed > REMOTE_TARGET_SPEED_MAX) {
        remote_target_right_speed = REMOTE_TARGET_SPEED_MAX;
    } else if (remote_target_right_speed < -REMOTE_TARGET_SPEED_MAX) {
        remote_target_right_speed = -REMOTE_TARGET_SPEED_MAX;
    }

    enc_left_delta = Encoder_GetCount(ENCODER_1);
    enc_right_delta = Encoder_GetCount(ENCODER_2);

    remote_measured_left_speed = REMOTE_ENCODER_LEFT_SIGN * ((float)enc_left_delta / dt_s);
    remote_measured_right_speed = REMOTE_ENCODER_RIGHT_SIGN * ((float)enc_right_delta / dt_s);

    left_cmd = Car_Task_ApplyCompensation(
        (int16_t)PID_Calc(&remote_left_speed_pid, remote_measured_left_speed, remote_target_left_speed));
    right_cmd = Car_Task_ApplyCompensation(
        (int16_t)PID_Calc(&remote_right_speed_pid, remote_measured_right_speed, remote_target_right_speed));

    remote_left_pwm = left_cmd;
    remote_right_pwm = right_cmd;

    Car_Task_SetRearMotorSpeed(remote_left_pwm, remote_right_pwm);
    Car_Task_DisplayRemoteInfo();
}

static uint8_t Car_Task_IsRemotePs2FrameValid(const JOYSTICK_TypeDef *joystick)
{
    if ((joystick->mode != REMOTE_PS2_MODE_DIGITAL) &&
        (joystick->mode != REMOTE_PS2_MODE_ANALOG_RED) &&
        (joystick->mode != REMOTE_PS2_MODE_ANALOG_PRESSURE)) {
        return 0U;
    }

    if ((joystick->RJoy_LR == 0xFFU) && (joystick->RJoy_UD == 0xFFU) &&
        (joystick->LJoy_LR == 0xFFU) && (joystick->LJoy_UD == 0xFFU)) {
        return 0U;
    }

    if ((joystick->RJoy_LR == 0x00U) && (joystick->RJoy_UD == 0x00U) &&
        (joystick->LJoy_LR == 0x00U) && (joystick->LJoy_UD == 0x00U)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 刷新远程控制调试显示
 */
static void Car_Task_DisplayRemoteInfo(void)
{
    char str[24];

    OLED_Clear(0);

    (void)snprintf(str, sizeof(str), "RX:%3u", (unsigned int)remote_joystick.RJoy_LR);
    OLED_ShowString(0, 0, str, 12);
    Car_Task_DrawStickBar(38, 1, 90, 6, remote_joystick.RJoy_LR);

    (void)snprintf(str, sizeof(str), "RY:%3u", (unsigned int)remote_joystick.RJoy_UD);
    OLED_ShowString(0, 1, str, 12);
    Car_Task_DrawStickBar(38, 9, 90, 6, remote_joystick.RJoy_UD);

    (void)snprintf(str, sizeof(str), "P%.2f I%.2f D%.2f",
                   REMOTE_SPEED_PID_KP, REMOTE_SPEED_PID_KI, REMOTE_SPEED_PID_KD);
    OLED_ShowString(0, 2, str, 12);

    (void)snprintf(str, sizeof(str), "TGT:%+4d %+4d",
                   (int16_t)remote_target_left_speed,
                   (int16_t)remote_target_right_speed);
    OLED_ShowString(0, 3, str, 12);

    (void)snprintf(str, sizeof(str), "ENC/s:%+4d %+4d",
                   (int16_t)remote_measured_left_speed,
                   (int16_t)remote_measured_right_speed);
    OLED_ShowString(0, 4, str, 12);

    (void)snprintf(str, sizeof(str), "PWM:%+4d %+4d", remote_left_pwm, remote_right_pwm);
    OLED_ShowString(0, 5, str, 12);

    (void)snprintf(str, sizeof(str), "V:%3.1f C:%1.2f", current_voltage, voltage_compensation);
    OLED_ShowString(0, 6, str, 12);

    if (remote_ps2_frame_valid != 0U) {
        OLED_ShowString(0, 7, "PS2:OK PC6 stop", 12);
    } else {
        OLED_ShowString(0, 7, "PS2:LOSS PC6 stop", 12);
    }
    OLED_Refresh();
}

static void Car_Task_DrawStickBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t value)
{
    uint8_t center_x;
    uint8_t half_width;
    uint8_t fill_width;

    if ((width < 4U) || (height < 3U)) {
        return;
    }

    half_width = (uint8_t)(width / 2U);
    center_x = (uint8_t)(x + half_width);

    OLED_DrawRectangle(x, y, (uint8_t)(x + width - 1U), (uint8_t)(y + height - 1U), 1);

    if (value > 128U) {
        fill_width = (uint8_t)(((uint16_t)(value - 128U) * (uint16_t)half_width) / 127U);
        if (fill_width > 0U) {
            OLED_DrawFillRectangle((uint8_t)(center_x + 1U), (uint8_t)(y + 1U),
                                   (uint8_t)(center_x + fill_width), (uint8_t)(y + height - 2U), 1);
        }
    } else if (value < 128U) {
        fill_width = (uint8_t)(((uint16_t)(128U - value) * (uint16_t)half_width) / 128U);
        if (fill_width > 0U) {
            OLED_DrawFillRectangle((uint8_t)(center_x - fill_width), (uint8_t)(y + 1U),
                                   (uint8_t)(center_x - 1U), (uint8_t)(y + height - 2U), 1);
        }
    }

    OLED_DrawLine(center_x, y, center_x, (uint8_t)(y + height - 1U), 1);
}

/**
 * @brief 刷新弓字遍历调试显示
 * @param current_heading 当前航向角（度）
 */
static void Car_Task_DisplayFieldInfo(float current_heading)
{
    char str[24];

    OLED_Clear(0);
    OLED_ShowString(0, 0, "Field Task", 16);

    (void)snprintf(str, sizeof(str), "S:%s R:%u", Car_Task_GetFieldStateText(field_scan_state),
                   (unsigned int)field_row_index);
    OLED_ShowString(0, 2, str, 12);

    (void)snprintf(str, sizeof(str), "Y:%5.1f T:%5.1f", current_heading, field_current_turn_target);
    OLED_ShowString(0, 3, str, 12);

    (void)snprintf(str, sizeof(str), "D:%5lu/%5lu",
                   (unsigned long)field_segment_distance_ticks,
                   (unsigned long)field_segment_target_ticks);
    OLED_ShowString(0, 4, str, 12);

    (void)snprintf(str, sizeof(str), "L:%4d R:%4d", field_last_left_speed, field_last_right_speed);
    OLED_ShowString(0, 5, str, 12);

    (void)snprintf(str, sizeof(str), "V:%3.1f C:%1.2f", current_voltage, voltage_compensation);
    OLED_ShowString(0, 6, str, 12);

    OLED_Refresh();
}

/**
 * @brief 获取状态机状态短文本
 * @param state 当前状态
 * @retval 状态文本常量指针
 */
static const char *Car_Task_GetFieldStateText(FieldScanState state)
{
    switch (state) {
        case FIELD_SCAN_STATE_FORWARD:
            return "FWD";
        case FIELD_SCAN_STATE_TURN_TO_SHIFT:
            return "TRN1";
        case FIELD_SCAN_STATE_SHIFT_FORWARD:
            return "SHIFT";
        case FIELD_SCAN_STATE_TURN_TO_NEXT_ROW:
            return "TRN2";
        case FIELD_SCAN_STATE_IDLE:
        default:
            return "IDLE";
    }
}

/**
 * @brief 任务模块初始化
 * @note 初始化电机、ADC、陀螺仪、PID 及运行时状态
 */
void Car_Task_Init(void)
{
    Motor_Init();
    (void)Car_Task_InitADC();
    if (ICM42670_Init() != 0U) {
        Gyro_Filter_Reset();
        car_task_gyro_type = CAR_TASK_GYRO_ICM42670;
        printf("Car task gyro: ICM42670\r\n");
    } else {
        Gryo_init();
        car_task_gyro_type = CAR_TASK_GYRO_WIT;
        printf("Car task gyro: WIT fallback\r\n");
    }

    PID_Init(&straight_heading_pid, STRAIGHT_PID_KP, STRAIGHT_PID_KI, STRAIGHT_PID_KD,
             STRAIGHT_PID_OUT_MAX, STRAIGHT_PID_OUT_MIN);
    PID_Init(&field_heading_pid, FIELD_PID_KP, FIELD_PID_KI, FIELD_PID_KD,
             FIELD_PID_OUT_MAX, FIELD_PID_OUT_MIN);
    PID_Init(&field_left_speed_pid, FIELD_SPEED_PID_KP, FIELD_SPEED_PID_KI, FIELD_SPEED_PID_KD,
             FIELD_SPEED_PID_OUT_MAX, FIELD_SPEED_PID_OUT_MIN);
    PID_Init(&field_right_speed_pid, FIELD_SPEED_PID_KP, FIELD_SPEED_PID_KI, FIELD_SPEED_PID_KD,
             FIELD_SPEED_PID_OUT_MAX, FIELD_SPEED_PID_OUT_MIN);
    PID_Init(&remote_left_speed_pid, REMOTE_SPEED_PID_KP, REMOTE_SPEED_PID_KI, REMOTE_SPEED_PID_KD,
             REMOTE_SPEED_PID_OUT_MAX, REMOTE_SPEED_PID_OUT_MIN);
    PID_Init(&remote_right_speed_pid, REMOTE_SPEED_PID_KP, REMOTE_SPEED_PID_KI, REMOTE_SPEED_PID_KD,
             REMOTE_SPEED_PID_OUT_MAX, REMOTE_SPEED_PID_OUT_MIN);

    Car_Task_ResetStraightPid();
    Car_Task_ResetFieldRuntime();
    Car_Task_ResetRemoteRuntime();

    remote_ps2_ready = 0U;
    task_encoder_ready = 0U;

    car_task_running = 0U;
    car_task_active_mode = car_task_selected_mode;
    Car_Task_StopAllMotors();
}

/**
 * @brief 启动任务
 * @note 根据当前模式进入直行、弓字遍历或远程控制流程
 */
void Car_Task_Start(void)
{
    car_task_running = 1U;
    car_task_active_mode = car_task_selected_mode;
    Car_Task_StopAllMotors();

    Car_Task_UpdateGyro();

    if (car_task_active_mode == CAR_TASK_MODE_FIELD_SCAN) {
        Car_Task_ResetFieldRuntime();// 重置

        field_heading_row_a = Car_Task_NormalizeAngle(fAngle[2]);
        field_heading_shift = Car_Task_NormalizeAngle(field_heading_row_a + 90.0f);
        field_heading_row_b = Car_Task_NormalizeAngle(field_heading_row_a + 180.0f);

        field_current_row_heading = field_heading_row_a;
        field_current_turn_target = field_current_row_heading;
        field_scan_state = FIELD_SCAN_STATE_FORWARD;
        field_segment_distance_ticks = 0U;
        field_segment_target_ticks = FIELD_SCAN_ROW_DISTANCE_TICKS;

        if (task_encoder_ready == 0U) {
            Encoder_Init();
            task_encoder_ready = 1U;
        }
        (void)Encoder_GetCount(ENCODER_1);
        (void)Encoder_GetCount(ENCODER_2);

        printf("Field task started: rowA=%.1f shift=%.1f rowB=%.1f\r\n",
               field_heading_row_a, field_heading_shift, field_heading_row_b);
    } else if (car_task_active_mode == CAR_TASK_MODE_REMOTE) {
        Car_Task_ResetRemoteRuntime();

        if (remote_ps2_ready == 0U) {
            AX_PS2_Init();
            AX_PS2_SetInit();
            remote_ps2_ready = 1U;
        }

        if (task_encoder_ready == 0U) {
            Encoder_Init();
            task_encoder_ready = 1U;
        }

        AX_PS2_ScanKey_Deadzone(&remote_joystick);

        remote_last_ctrl_tick = HAL_GetTick();
        printf("Remote task started\r\n");
    } else {
        Car_Task_ResetStraightPid();
        printf("Straight task started\r\n");
    }
}

/**
 * @brief 停止任务并清空控制状态
 */
void Car_Task_Stop(void)
{
    car_task_running = 0U;
    Car_Task_StopAllMotors();
    Car_Task_ResetStraightPid();
    Car_Task_ResetFieldRuntime();
    Car_Task_ResetRemoteRuntime();
    printf("Car task stopped\r\n");
}

/**
 * @brief 任务主循环入口
 * @note 由上层周期调用（例如菜单主循环）
 */
void Car_Task_Main(void)
{
    if (car_task_running == 0U) {
        return;
    }

    if (car_task_active_mode == CAR_TASK_MODE_FIELD_SCAN) {
        Car_Task_RunFieldScanTask();
    } else if (car_task_active_mode == CAR_TASK_MODE_REMOTE) {
        Car_Task_RunRemoteTask();
    } else {
        Car_Task_RunStraightTask();
    }
}

/**
 * @brief 查询任务运行状态
 * @retval 1 正在运行，0 已停止
 */
uint8_t Car_Task_IsRunning(void)
{
    return car_task_running;
}

/**
 * @brief 设置直行模式目标角度
 * @param angle 目标角度（度）
 */
void Car_Task_SetTargetAngle(float angle)
{
    target_angle = Car_Task_NormalizeAngle(angle);
}

/**
 * @brief 设置全局速度上限
 * @param speed 速度上限（PWM，支持传入负值）
 */
void Car_Task_SetMaxSpeed(int16_t speed)
{
    if (speed < 0) {
        speed = (int16_t)(-speed);
    }

    if (speed > FIELD_SCAN_MAX_PWM) {
        max_speed = FIELD_SCAN_MAX_PWM;
    } else {
        max_speed = speed;
    }
}

/**
 * @brief 设置任务模式（仅在未运行时生效）
 * @param mode 任务模式
 */
void Car_Task_SetMode(CarTaskModeTypeDef mode)
{
    if (car_task_running != 0U) {
        return;
    }

    car_task_selected_mode = mode;
}

/**
 * @brief 获取当前选中的任务模式
 * @retval 当前模式
 */
CarTaskModeTypeDef Car_Task_GetMode(void)
{
    return car_task_selected_mode;
}
