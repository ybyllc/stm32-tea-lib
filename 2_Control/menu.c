/**
 * @file menu.c
 * @brief 菜单系统核心实现
 * @note 负责子菜单管理和菜单系统的整体逻辑
 */

/*
 * ==================== Menu方案风格约定（开发/AI参考）====================
 * 1) 页面职责单一：一个页面函数只负责“读取状态 + 渲染显示”，不做重型业务逻辑。
 * 2) 初始化懒加载：进入页面时再初始化模块（Menu_InitXXX），避免上电一次性初始化全部外设。
 * 3) 统一离线降级：驱动未链接/外设离线时，页面必须显示可读提示，不崩溃、不死循环。
 * 4) 输入/输出互斥：同一外设复用端口（如PPM/SBUS）时，页面内只启用一种模式并明确提示。
 * 5) 显示风格统一：标题在第0行，状态在第1行，数据主体在中间，最后一行保留返回提示。
 * 6) 摇杆可视化统一：通道值先归一化，再用DrawStickBar绘制，128为中位，左右对称填充。
 * 7) 参数边界保护：所有通道值在显示前先限幅，防止异常值导致UI错位或数值溢出。
 * 8) 可维护优先：公共逻辑提取为静态函数，禁止在多个页面复制粘贴同一段处理代码。
 * 9) 修改原则：新增页面先补函数声明，再补初始化、渲染、按键处理、菜单入口四处映射。
 * 10) 调试友好：错误信息尽量给出“检查方向”（引脚/波特率/中断/驱动是否加入工程）。
 * ======================================================================
 */

#include "menu.h"
#include "menu_button.h"
#include "oled_ui_mode.h"
#include "wit_gyro_sdk.h"
#include "icm42670.h"
#include "TOF_VL53L0X.h"
#include "ultrasonic_sr04.h"
#include "remote_key.h"
#include "ppm_input.h"
#include "sbus_input.h"
#include "Cam_uart.h"
#include "motor_tb6612.h"
#include "motor_standard.h"
#include "PWM_standard.h"
#include "servo_pwm.h"
#include "key_pc6.h"
#include "encoder.h"
#include "car_task.h"
#include "gyro_collision.h"
#include "PID.h"
#include "debug_uart.h"
#include "oled.h"
#include "ax_ps2.h"
// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_adc.h"
// #include "stm32f4xx_hal_gpio.h"
#include <stdio.h>
#include <stdlib.h>

/* ===== 菜单闭环测试任务参数 ===== */
#define MENU_SPEED_TEST_PERIOD_MS            20U
#define MENU_SPEED_TEST_TARGET_CPS_DEFAULT   520.0f
#define MENU_SPEED_PID_KP_DEFAULT            0.16f
#define MENU_SPEED_PID_KI_DEFAULT            0.02f
#define MENU_SPEED_PID_KD_DEFAULT            0.00f
#define MENU_SPEED_PID_OUT_MAX               700.0f
#define MENU_SPEED_PID_OUT_MIN               (-700.0f)
#define MENU_SPEED_ENCODER_LEFT_SIGN         1.0f
#define MENU_SPEED_ENCODER_RIGHT_SIGN        1.0f
#define MENU_SPEED_TARGET_CPS_MIN            (-1200.0f)
#define MENU_SPEED_TARGET_CPS_MAX            (1200.0f)
#define MENU_SPEED_PID_KP_MIN                0.0f
#define MENU_SPEED_PID_KP_MAX                3.0f
#define MENU_SPEED_PID_KI_MIN                0.0f
#define MENU_SPEED_PID_KI_MAX                0.8f
#define MENU_SPEED_PID_KD_MIN                0.0f
#define MENU_SPEED_PID_KD_MAX                1.5f
#define MENU_SPEED_TARGET_CPS_STEP           20.0f
#define MENU_SPEED_PID_KP_STEP               0.01f
#define MENU_SPEED_PID_KI_STEP               0.002f
#define MENU_SPEED_PID_KD_STEP               0.01f

#define MENU_CIRCLE_CTRL_PERIOD_MS           20U
#define MENU_CIRCLE_TARGET_RADIUS_CM         20.0f
#define MENU_CIRCLE_BASE_PWM                 220
#define MENU_CIRCLE_TARGET_RATE_DPS          20.0f
#define MENU_CIRCLE_PID_KP                   2.8f
#define MENU_CIRCLE_PID_KI                   0.03f
#define MENU_CIRCLE_PID_KD                   0.08f
#define MENU_CIRCLE_PID_OUT_MAX              120.0f
#define MENU_CIRCLE_PID_OUT_MIN              (-120.0f)
#define MENU_CIRCLE_PID_KP_MIN               0.0f
#define MENU_CIRCLE_PID_KP_MAX               20.0f
#define MENU_CIRCLE_PID_KI_MIN               0.0f
#define MENU_CIRCLE_PID_KI_MAX               5.0f
#define MENU_CIRCLE_PID_KD_MIN               0.0f
#define MENU_CIRCLE_PID_KD_MAX               5.0f
#define MENU_CIRCLE_HEADING_LPF_ALPHA        0.25f

//相机追踪 运动参数
#define MENU_CAM_TRACK_CENTER_X              (-100)
#define MENU_CAM_TRACK_X_DEADBAND            10
#define MENU_CAM_TRACK_BASE_PWM              320
#define MENU_CAM_TRACK_STEER_KP              0.1f
#define MENU_CAM_TRACK_STEER_KI              0.0f
#define MENU_CAM_TRACK_STEER_KD              0.0f
#define MENU_CAM_TRACK_STEER_OUT_MAX         30.0f
#define MENU_CAM_TRACK_STEER_OUT_MIN         (-30.0f)
#define MENU_CAM_TRACK_TARGET_STEP_MAX_DEG   2.0f
#define MENU_CAM_TRACK_X_MAX_DEG             45.0f
#define MENU_CAM_TRACK_HEADING_KP            2.8f
#define MENU_CAM_TRACK_HEADING_OUT_MAX       200.0f
#define MENU_CAM_TRACK_HEADING_OUT_MIN       (-200.0f)


// 陀螺仪类型枚举
typedef enum {
    GYRO_TYPE_NONE,        // 无陀螺仪
    GYRO_TYPE_WIT,         // 原来的陀螺仪
    GYRO_TYPE_ICM42670     // ICM42670 陀螺仪
} GyroType;

typedef enum {
    MENU_SPEED_PID_ITEM_TARGET = 0,
    MENU_SPEED_PID_ITEM_KP,
    MENU_SPEED_PID_ITEM_KI,
    MENU_SPEED_PID_ITEM_KD,
    MENU_SPEED_PID_ITEM_RUN,
    MENU_SPEED_PID_ITEM_COUNT
} MenuSpeedPidItemType;

// 模块初始化状态标志
uint8_t gyro_initialized = 0;    // 陀螺仪是否已初始化
uint8_t tof_initialized = 0;     // TOF是否已初始化
uint8_t sr04_initialized = 0;    // SR04是否已初始化
uint8_t remote_key_initialized = 0; // 遥控器是否已初始化
uint8_t ppm_initialized = 0;      // PPM输入是否已初始化
uint8_t sbus_initialized = 0;     // SBUS输入是否已初始化
uint8_t motor_initialized = 0;   // 电机是否已初始化
uint8_t adc_initialized = 0;     // ADC是否已初始化
uint8_t ps2_initialized = 0;     // PS2手柄是否已初始化
uint8_t gpio_initialized = 0;    // GPIO测试是否已初始化
static uint8_t cam_initialized = 0U; // 摄像头调试页面是否已初始化

// ADC句柄
extern ADC_HandleTypeDef hadc1;

// 电机测试状态
int16_t motor_speeds[4] = {0, 0, 0, 0}; // 四个电机的速度值（-1000到1000）
uint8_t current_motor_index = 0; // 当前选中的电机索引（0-3）
uint8_t motor_speed_adjust_mode = 0; // 速度调整模式：0-选择电机模式，1-调整速度模式

// 系统状态变量
uint8_t system_mode = 0; // 0-菜单模式, 1-任务模式
uint32_t task_start_time = 0; // 任务开始时间

// 编码器累计值
int32_t encoder1_total_count = 0; // 编码器1的累计值
int32_t encoder2_total_count = 0; // 编码器2的累计值

// 当前使用的陀螺仪类型
static GyroType current_gyro_type = GYRO_TYPE_NONE;
static MenuPageType menu_last_page = MENU_PAGE_MAIN;

// 速度环直线测试状态
static uint8_t speed_straight_task_active = 0U;
static uint32_t speed_straight_last_tick = 0U;
static int32_t speed_straight_last_enc_left = 0;
static int32_t speed_straight_last_enc_right = 0;
static float speed_straight_meas_left = 0.0f;
static float speed_straight_meas_right = 0.0f;
static int16_t speed_straight_pwm_left = 0;
static int16_t speed_straight_pwm_right = 0;
static PID_t speed_straight_left_pid;
static PID_t speed_straight_right_pid;
static float speed_straight_target_cps = MENU_SPEED_TEST_TARGET_CPS_DEFAULT;
static float speed_straight_pid_kp = MENU_SPEED_PID_KP_DEFAULT;
static float speed_straight_pid_ki = MENU_SPEED_PID_KI_DEFAULT;
static float speed_straight_pid_kd = MENU_SPEED_PID_KD_DEFAULT;
static uint8_t speed_pid_item_index = 0U;
static uint8_t speed_pid_edit_mode = 0U;
static uint8_t speed_pid_output_enable = 1U;

// 角度环画圆测试状态
static uint8_t gyro_circle_task_active = 0U;
static uint32_t gyro_circle_last_tick = 0U;
static float gyro_circle_target_heading = 0.0f;
static float gyro_circle_filtered_heading = 0.0f;
static float gyro_circle_error = 0.0f;
static float gyro_circle_turn_output = 0.0f;
static int16_t gyro_circle_pwm_left = 0;
static int16_t gyro_circle_pwm_right = 0;
static PID_t gyro_circle_heading_pid;

// Cam Info 追踪状态（短按EC11启停）
static uint8_t cam_track_active = 0U;
static uint32_t cam_track_last_tick = 0U;
static float cam_track_filtered_heading = 0.0f;
static float cam_track_target_heading = 0.0f;
static float cam_track_x_error_filtered = 0.0f;
static float cam_track_steer_output = 0.0f;
static float cam_track_error = 0.0f;
static float cam_track_turn_output = 0.0f;
static int16_t cam_track_pwm_left = 0;
static int16_t cam_track_pwm_right = 0;
static PID_t cam_track_steer_pid;
static PID_t cam_track_heading_pid;

// 外部函数声明
extern void Gryo_Update(void);
extern u8 VL53L0X_IsReady(void);
extern u16 TOF_QuickTest(void);

// 静态函数声明
static void Menu_DisplayGyroPage(void);
static void Menu_DisplayEc11TestPage(void);
static void Menu_DisplayTofTestPage(void);
static void Menu_DisplayUltrasonicTestPage(void);
static void Menu_DisplayFieldTaskPage(void);
static void Menu_DisplayRemoteTaskPage(void);
static void Menu_DisplaySpeedStraightTaskPage(void);
static void Menu_DisplayGyroCircleTaskPage(void);
static void Menu_DisplayRemoteKeyTestPage(void);
static void Menu_DisplayPPMInputTestPage(void);
static void Menu_DisplaySBUSInputTestPage(void);
static void Menu_DisplayCamInfoPage(void);
static void Menu_DisplayMotorTestPage(void);
static void Menu_DisplayEncoderTestPage(void);
static void Menu_DisplayAdcTestPage(void);
static void Menu_DisplayPS2TestPage(void);
static void Menu_DisplayPS2ControlPage(void);
static void Menu_DisplayGPIOTestPage(void);
static void Menu_DisplayMainMenu(void);
static void Menu_DisplayCurrentPage(void);
static u8 Menu_IsPPMDriverLinked(void);
static u8 Menu_IsSBUSDriverLinked(void);
static CarTaskModeTypeDef Menu_GetPendingTaskMode(void);
static int16_t Menu_ClampMotorPwm(int32_t pwm);
static float Menu_ClampFloat(float value, float min_value, float max_value);
static float Menu_NormalizeAngle(float angle);
static void Menu_SpeedPid_SetOutputEnable(uint8_t enable);
static void Menu_StopSpeedStraightTask(void);
static void Menu_StopGyroCircleTask(void);
static void Menu_StopCamTrack(void);
static void Menu_UpdateCamTrack(void);

/**
 * @brief 判断PPM驱动是否已链接到当前工程
 * @retval 1-已链接, 0-未链接
 */
static u8 Menu_IsPPMDriverLinked(void) {
    return 1U;
}

/**
 * @brief 判断SBUS输入驱动是否已链接到当前工程
 * @retval 1-已链接, 0-未链接
 */
static u8 Menu_IsSBUSDriverLinked(void) {
    return 1U;
}

static int16_t Menu_ClampMotorPwm(int32_t pwm) {
    if (pwm > 1000) {
        pwm = 1000;
    } else if (pwm < -1000) {
        pwm = -1000;
    }
    return (int16_t)pwm;
}

static float Menu_ClampFloat(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }
    return value;
}

static float Menu_NormalizeAngle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 采集调试串口需要的状态快照
 * @note 仅做只读拷贝，不做耗时计算
 */
void Menu_Debug_GetState(MenuDebugState_t *out) {
    const Cam_Recv_Data_t *cam;

    if (out == NULL) {
        return;
    }

    /* 摄像头在线状态单独取一次，避免重复调用 */
    cam = Cam_Uart_Get();

    out->tick_ms = HAL_GetTick();
    out->system_mode = system_mode;
    out->page = (uint8_t)menuState.currentPage;
    out->gyro_ready = gyro_initialized;
    out->cam_online = (cam != NULL) ? cam->is_online : 0U;

    out->yaw_deg = fYaw;
    out->gyro_x_dps = fGyro[0];
    out->gyro_y_dps = fGyro[1];
    out->gyro_z_dps = fGyro[2];
    out->acc_x_g = fAcc[0];
    out->acc_y_g = fAcc[1];
    out->acc_z_g = fAcc[2];

    /* 电机调试页使用的四路目标速度 */
    out->motor_fl = motor_speeds[0];
    out->motor_fr = motor_speeds[1];
    out->motor_bl = motor_speeds[2];
    out->motor_br = motor_speeds[3];

    /* 速度环核心状态：目标、测量、输出、PID参数 */
    out->speed_target_cps = speed_straight_target_cps;
    out->speed_meas_left_cps = speed_straight_meas_left;
    out->speed_meas_right_cps = speed_straight_meas_right;
    out->speed_pwm_left = speed_straight_pwm_left;
    out->speed_pwm_right = speed_straight_pwm_right;
    out->speed_kp = speed_straight_pid_kp;
    out->speed_ki = speed_straight_pid_ki;
    out->speed_kd = speed_straight_pid_kd;
    out->speed_output_enable = speed_pid_output_enable;

    /* 画圆角度环核心状态 */
    out->circle_target_heading = gyro_circle_target_heading;
    out->circle_filtered_heading = gyro_circle_filtered_heading;
    out->circle_error = gyro_circle_error;
    out->circle_pwm_left = gyro_circle_pwm_left;
    out->circle_pwm_right = gyro_circle_pwm_right;
    out->circle_kp = gyro_circle_heading_pid.kp;
    out->circle_ki = gyro_circle_heading_pid.ki;
    out->circle_kd = gyro_circle_heading_pid.kd;
}

/**
 * @brief 外部调试接口：动态更新速度环PID参数
 * @param mask 控制本次需要更新的字段
 */
uint8_t Menu_Debug_SetSpeedPid(float kp, float ki, float kd, float target_cps,
                               uint8_t output_enable, uint8_t mask) {
    /* 按位掩码更新参数，便于上位机一次只改一个参数 */
    if ((mask & MENU_DEBUG_PID_MASK_KP) != 0U) {
        speed_straight_pid_kp = Menu_ClampFloat(kp, MENU_SPEED_PID_KP_MIN, MENU_SPEED_PID_KP_MAX);
    }
    if ((mask & MENU_DEBUG_PID_MASK_KI) != 0U) {
        speed_straight_pid_ki = Menu_ClampFloat(ki, MENU_SPEED_PID_KI_MIN, MENU_SPEED_PID_KI_MAX);
    }
    if ((mask & MENU_DEBUG_PID_MASK_KD) != 0U) {
        speed_straight_pid_kd = Menu_ClampFloat(kd, MENU_SPEED_PID_KD_MIN, MENU_SPEED_PID_KD_MAX);
    }
    if ((mask & MENU_DEBUG_PID_MASK_TARGET) != 0U) {
        speed_straight_target_cps = Menu_ClampFloat(target_cps, MENU_SPEED_TARGET_CPS_MIN, MENU_SPEED_TARGET_CPS_MAX);
    }
    if ((mask & MENU_DEBUG_PID_MASK_ENABLE) != 0U) {
        /* 切换使能时走统一入口，保证内部状态一起复位 */
        Menu_SpeedPid_SetOutputEnable(output_enable);
    } else {
        /* 未切换使能时，直接热更新当前PID结构体系数 */
        speed_straight_left_pid.kp = speed_straight_pid_kp;
        speed_straight_left_pid.ki = speed_straight_pid_ki;
        speed_straight_left_pid.kd = speed_straight_pid_kd;
        speed_straight_right_pid.kp = speed_straight_pid_kp;
        speed_straight_right_pid.ki = speed_straight_pid_ki;
        speed_straight_right_pid.kd = speed_straight_pid_kd;
    }

    return 1U;
}

/**
 * @brief 外部调试接口：动态更新画圆角度环PID参数
 * @param mask 控制本次需要更新的字段
 */
uint8_t Menu_Debug_SetCirclePid(float kp, float ki, float kd, uint8_t mask) {
    /* 画圆角度环同样支持按位热更新 */
    if ((mask & MENU_DEBUG_PID_MASK_KP) != 0U) {
        gyro_circle_heading_pid.kp = Menu_ClampFloat(kp, MENU_CIRCLE_PID_KP_MIN, MENU_CIRCLE_PID_KP_MAX);
    }
    if ((mask & MENU_DEBUG_PID_MASK_KI) != 0U) {
        gyro_circle_heading_pid.ki = Menu_ClampFloat(ki, MENU_CIRCLE_PID_KI_MIN, MENU_CIRCLE_PID_KI_MAX);
    }
    if ((mask & MENU_DEBUG_PID_MASK_KD) != 0U) {
        gyro_circle_heading_pid.kd = Menu_ClampFloat(kd, MENU_CIRCLE_PID_KD_MIN, MENU_CIRCLE_PID_KD_MAX);
    }

    return 1U;
}

static void Menu_SpeedPid_SetOutputEnable(uint8_t enable) {
    speed_pid_output_enable = (enable != 0U) ? 1U : 0U;

    PID_Reset(&speed_straight_left_pid);
    PID_Reset(&speed_straight_right_pid);
    speed_straight_meas_left = 0.0f;
    speed_straight_meas_right = 0.0f;
    speed_straight_pwm_left = 0;
    speed_straight_pwm_right = 0;
    speed_straight_last_enc_left = 0;
    speed_straight_last_enc_right = 0;
    speed_straight_last_tick = HAL_GetTick();
    (void)Encoder_GetCount(ENCODER_1);
    (void)Encoder_GetCount(ENCODER_2);

    if (speed_pid_output_enable == 0U) {
        Motor_StopAll();
    }
}

static void Menu_StopSpeedStraightTask(void) {
    if (speed_straight_task_active != 0U) {
        speed_straight_task_active = 0U;
        speed_straight_last_tick = 0U;
        speed_straight_last_enc_left = 0;
        speed_straight_last_enc_right = 0;
        speed_straight_meas_left = 0.0f;
        speed_straight_meas_right = 0.0f;
        speed_straight_pwm_left = 0;
        speed_straight_pwm_right = 0;
        PID_Reset(&speed_straight_left_pid);
        PID_Reset(&speed_straight_right_pid);
    }
    speed_pid_item_index = MENU_SPEED_PID_ITEM_TARGET;
    speed_pid_edit_mode = 0U;
    speed_pid_output_enable = 1U;
    Motor_StopAll();
}

static void Menu_StopGyroCircleTask(void) {
    if (gyro_circle_task_active != 0U) {
        gyro_circle_task_active = 0U;
        gyro_circle_last_tick = 0U;
        gyro_circle_target_heading = 0.0f;
        gyro_circle_filtered_heading = 0.0f;
        gyro_circle_error = 0.0f;
        gyro_circle_turn_output = 0.0f;
        gyro_circle_pwm_left = 0;
        gyro_circle_pwm_right = 0;
        PID_Reset(&gyro_circle_heading_pid);
    }
    Motor_StopAll();
}

static void Menu_StopCamTrack(void) {
    cam_track_active = 0U;
    cam_track_last_tick = 0U;
    cam_track_filtered_heading = 0.0f;
    cam_track_target_heading = 0.0f;
    cam_track_x_error_filtered = 0.0f;
    cam_track_steer_output = 0.0f;
    cam_track_error = 0.0f;
    cam_track_turn_output = 0.0f;
    cam_track_pwm_left = 0;
    cam_track_pwm_right = 0;
    PID_Reset(&cam_track_steer_pid);
    PID_Reset(&cam_track_heading_pid);
    Motor_StopAll();
}

static void Menu_UpdateCamTrack(void) {
    const Cam_Recv_Data_t *cam;
    uint32_t now_tick;
    uint32_t dt_ms;
    float heading_raw;
    float heading_delta;
    float target_delta;
    float x_error;
    float desired_offset_deg;
    float desired_target_heading;

    if (cam_track_active == 0U) {
        return;
    }

    now_tick = HAL_GetTick();
    dt_ms = now_tick - cam_track_last_tick;
    if (dt_ms < MENU_CIRCLE_CTRL_PERIOD_MS) {
        return;
    }

    if (dt_ms > 300U) {
        PID_Reset(&cam_track_steer_pid);
        PID_Reset(&cam_track_heading_pid);
        cam_track_target_heading = cam_track_filtered_heading;
        cam_track_x_error_filtered = 0.0f;
        cam_track_steer_output = 0.0f;
        cam_track_error = 0.0f;
        cam_track_turn_output = 0.0f;
        cam_track_pwm_left = 0;
        cam_track_pwm_right = 0;
        Motor_StopAll();
        cam_track_last_tick = now_tick;
        return;
    }

    Menu_Gryo_Update();
    heading_raw = Menu_NormalizeAngle(fAngle[2]);
    heading_delta = Menu_NormalizeAngle(heading_raw - cam_track_filtered_heading);
    cam_track_filtered_heading = Menu_NormalizeAngle(
        cam_track_filtered_heading + heading_delta * MENU_CIRCLE_HEADING_LPF_ALPHA);

    cam = Cam_Uart_Get();
    if ((cam->is_online != 0U) &&
        (cam->class_id == CAM_CLASS_BOTTLE) &&
        ((cam->flag == CAM_FLAG_TRACKING) || (cam->flag == CAM_FLAG_PREDICTING))) {
        x_error = (float)cam->x - (float)MENU_CAM_TRACK_CENTER_X;
        if ((x_error > -(float)MENU_CAM_TRACK_X_DEADBAND) &&
            (x_error < (float)MENU_CAM_TRACK_X_DEADBAND)) {
            x_error = 0.0f;
        }
        cam_track_x_error_filtered += (x_error - cam_track_x_error_filtered) * 0.25f;

        // 外环：摄像头转向环（x误差 -> 目标偏航角）
        cam_track_steer_output = PID_Calc(&cam_track_steer_pid, cam_track_x_error_filtered, 0.0f);
        desired_offset_deg = Menu_ClampFloat(
            cam_track_steer_output,
            -MENU_CAM_TRACK_X_MAX_DEG, MENU_CAM_TRACK_X_MAX_DEG);
        desired_target_heading = Menu_NormalizeAngle(cam_track_filtered_heading + desired_offset_deg);
        target_delta = Menu_NormalizeAngle(desired_target_heading - cam_track_target_heading);
        target_delta = Menu_ClampFloat(
            target_delta, -MENU_CAM_TRACK_TARGET_STEP_MAX_DEG, MENU_CAM_TRACK_TARGET_STEP_MAX_DEG);
        cam_track_target_heading = Menu_NormalizeAngle(cam_track_target_heading + target_delta);

        // 内环：陀螺仪角度环（目标偏航角 -> 差速输出）
        cam_track_error = Menu_NormalizeAngle(cam_track_target_heading - cam_track_filtered_heading);
        cam_track_turn_output = PID_Calc(&cam_track_heading_pid, -cam_track_error, 0.0f);

        cam_track_pwm_left = Menu_ClampMotorPwm((int32_t)((float)MENU_CAM_TRACK_BASE_PWM - cam_track_turn_output));
        cam_track_pwm_right = Menu_ClampMotorPwm((int32_t)((float)MENU_CAM_TRACK_BASE_PWM + cam_track_turn_output));

        Motor_SetSpeed(MOTOR_FRONT_LEFT, 0);
        Motor_SetSpeed(MOTOR_FRONT_RIGHT, 0);
        Motor_SetSpeed(MOTOR_BACK_LEFT, cam_track_pwm_left);
        Motor_SetSpeed(MOTOR_BACK_RIGHT, cam_track_pwm_right);
    } else {
        // 丢目标静止，不再原地转圈
        PID_Reset(&cam_track_steer_pid);
        PID_Reset(&cam_track_heading_pid);
        cam_track_target_heading = cam_track_filtered_heading;
        cam_track_x_error_filtered = 0.0f;
        cam_track_steer_output = 0.0f;
        cam_track_error = 0.0f;
        cam_track_turn_output = 0.0f;
        cam_track_pwm_left = 0;
        cam_track_pwm_right = 0;
        Motor_StopAll();
    }

    cam_track_last_tick = now_tick;
}

/**
 * @brief 根据当前菜单位置决定即将启动的任务模式
 * @retval 小车任务模式
 * @note 仅当主菜单光标停留在 Field Task 项时，PC6 启动水田遍历任务
 */
static CarTaskModeTypeDef Menu_GetPendingTaskMode(void) {
    if ((menuState.currentPage == MENU_PAGE_MAIN) &&
        (mainMenuItems[menuState.currentItem].page == MENU_PAGE_FIELD_TASK)) {
        return CAR_TASK_MODE_FIELD_SCAN;
    }

    if ((menuState.currentPage == MENU_PAGE_MAIN) &&
        (mainMenuItems[menuState.currentItem].page == MENU_PAGE_REMOTE_TASK)) {
        return CAR_TASK_MODE_REMOTE;
    }

    if (menuState.currentPage == MENU_PAGE_FIELD_TASK) {
        return CAR_TASK_MODE_FIELD_SCAN;
    }

    if (menuState.currentPage == MENU_PAGE_REMOTE_TASK) {
        return CAR_TASK_MODE_REMOTE;
    }

    return CAR_TASK_MODE_STRAIGHT;
}

/**
 * @brief 绘制摇杆进度条（带中心点）
 * @param x: 起始x坐标
 * @param y: 起始y坐标
 * @param width: 进度条总宽度
 * @param value: 摇杆值 (0-255, 128为中位)
 */
static void DrawStickBar(u8 x, u8 y, u8 width, u8 value) {
    u8 height = 9;  // 进度条高度
    u8 center_x = x + width / 2;  // 中心位置（中位点）
    
    // 绘制外框
    OLED_DrawRectangle(x, y, x + width - 1, y + height - 1, 1);
    
    // 计算填充位置（value: 0-255 -> 0-width）
    // 128为中位，小于128向左填充，大于128向右填充
    if (value > 128) {
        // 向右填充
        u8 fill_width = ((value - 128) * (width / 2)) / 128;
        if (fill_width > 0) {
            OLED_DrawFillRectangle(center_x, y + 1, center_x + fill_width, y + height - 2, 1);
        }
    } else if (value < 128) {
        // 向左填充
        u8 fill_width = ((128 - value) * (width / 2)) / 128;
        if (fill_width > 0) {
            OLED_DrawFillRectangle(center_x - fill_width, y + 1, center_x, y + height - 2, 1);
        }
    }
    
    // 绘制中心线
    OLED_DrawLine(center_x, y, center_x, y + height - 1, 1);
}

/**
 * @brief 初始化ADC（仅在第一次进入ADC页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitAdc(void) {
    if (adc_initialized) {
        return 1;  // 已经初始化过
    }
    
    // 启用GPIOC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // 启用ADC1时钟
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    // 配置PC4引脚为ADC输入
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = GPIO_PIN_4,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL
    };
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 配置ADC
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;  // 单次转换模式
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return 0;  // 初始化失败
    }
    
    // 配置ADC通道
    ADC_ChannelConfTypeDef sConfig = {
        .Channel = ADC_CHANNEL_14,  // PC4对应ADC1通道14
        .Rank = 1,
        .SamplingTime = ADC_SAMPLETIME_480CYCLES  // 使用较长的采样时间
    };
    
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0;  // 通道配置失败
    }
    
    adc_initialized = 1;
    return 1;
}

/**
 * @brief 初始化PS2手柄（仅在第一次进入PS2页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitPS2(void) {
    if (ps2_initialized) {
        return 1;  // 已经初始化过
    }
    
    // 初始化PS2手柄
    AX_PS2_Init();
    AX_PS2_SetInit();
    
    ps2_initialized = 1;
    return 1;
}

/**
 * @brief 初始化陀螺仪（仅在第一次进入Gyro页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitGyro(void) {
    if (gyro_initialized) {
        return 1;  // 已经初始化过
    }
    
    // 尝试初始化 ICM42670
    if (ICM42670_Init()) {
        Gyro_Filter_Reset();  // 进入陀螺仪页面时重置控制层滤波状态
        current_gyro_type = GYRO_TYPE_ICM42670;
        gyro_initialized = 1;
        return 1;
    }
    
    // 如果 ICM42670 失败，尝试初始化串口陀螺仪（WIT）
    Gryo_init();  // 串口陀螺仪初始化
    Gyro_Filter_Reset();
    current_gyro_type = GYRO_TYPE_WIT;
    gyro_initialized = 1;
    return 1;
}

/**
 * @brief 初始化TOF（仅在第一次进入TOF页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitTof(void) {
    if (tof_initialized) {
        return 1;  // 已经初始化过
    }
    
    if (VL53L0X_Init() == 0) {
        tof_initialized = 1;
        return 1;
    }
    
    return 0;  // 初始化失败
}

/**
 * @brief 初始化遥控器（仅在第一次进入Remote Key页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitRemoteKey(void) {
    if (remote_key_initialized) {
        return 1;  // 已经初始化过
    }

    RemoteKey_Init();
    remote_key_initialized = 1;
    return 1;
}

/**
 * @brief 初始化PPM输入（仅在第一次进入PPM页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitPPM(void) {
    if (ppm_initialized) {
        return 1;
    }

    if (Menu_IsPPMDriverLinked() == 0U) {
        return 0;
    }

    PPM_In_Init();
    ppm_initialized = 1;
    return 1;
}

/**
 * @brief 初始化SBUS输入（仅在第一次进入SBUS页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitSBUS(void) {
    if (sbus_initialized) {
        return 1;
    }

    if (Menu_IsSBUSDriverLinked() == 0U) {
        return 0;
    }

    SBUS_In_Init();
    sbus_initialized = 1;
    return 1;
}

/**
 * @brief 初始化电机（仅在第一次进入Motor Test页面时调用）
 * @retval 0-失败, 1-成功
 */
uint8_t Menu_InitMotor(void) {
    if (motor_initialized) {
        return 1;  // 已经初始化过
    }
    
    Motor_Init();
    motor_initialized = 1;
    return 1;
}

/**
 * @brief  通用陀螺仪更新函数，根据当前使用的陀螺仪类型调用对应的更新函数
 * @param  无
 * @retval 无
 */
void Menu_Gryo_Update(void) {
    switch (current_gyro_type) {
        case GYRO_TYPE_ICM42670:
            // 使用 ICM42670 陀螺仪
            ICM42670_Gryo_Update();
            // 在控制层做二次滤波与姿态融合，抑制抖动和积分漂移
            Gyro_Filter_Update(fAcc, fGyro, fAngle, &fYaw);
            break;
        case GYRO_TYPE_WIT:
            // 使用原来的陀螺仪
            Gryo_Update();
            break;
        case GYRO_TYPE_NONE:
        default:
            // 无陀螺仪，不做任何操作
            break;
    }
}

// 主菜单项目定义
const MenuItem mainMenuItems[] = {
    {"Gyro Info", MENU_PAGE_GYRO},
    {"Cam Info", MENU_PAGE_CAM_INFO},
    {"EC11 Test", MENU_PAGE_EC11_TEST},
    {"TOF Test", MENU_PAGE_TOF_TEST},
    {"SR04 Test", MENU_PAGE_SR04_TEST},
    {"Task IF", MENU_PAGE_FIELD_TASK},
    {"Remote Task", MENU_PAGE_REMOTE_TASK},
    {"PID Tune", MENU_PAGE_SPEED_STRAIGHT_TASK},
    {"Gyro Circle", MENU_PAGE_GYRO_CIRCLE_TASK},
    {"Remote Key Test", MENU_PAGE_REMOTE_KEY_TEST},
    {"PPM In Test", MENU_PAGE_PPM_INPUT_TEST},
    {"SBUS In Test", MENU_PAGE_SBUS_INPUT_TEST},
    {"Motor Test", MENU_PAGE_MOTOR_TEST},
    {"Encoder Test", MENU_PAGE_ENCODER_TEST},
    {"ADC Test", MENU_PAGE_ADC_TEST},
    {"PS2 Test", MENU_PAGE_PS2_TEST},
    {"PS2 Control", MENU_PAGE_PS2_CONTROL},
    {"Servo PWM", MENU_PAGE_SERVO_PWM},
    {"GPIO Test", MENU_PAGE_GPIO_TEST}
};

// 主菜单项目数量
uint8_t MAIN_MENU_ITEM_COUNT = (sizeof(mainMenuItems) / sizeof(MenuItem));

// 菜单状态
MenuState menuState;

/**
 * @brief 菜单系统初始化
 */
void Menu_Init(void) {
    menuState.currentPage = MENU_PAGE_MAIN;
    menuState.currentItem = 0;
    menuState.lastEncoderCount = 0;
    menuState.isInMenu = 1;
    menu_last_page = MENU_PAGE_MAIN;

    // 初始化输入处理
    MenuInput_Init();

    // 传统模式初始化OLED
    OLED_Init();
    OLED_Clear(0);
    
    // 显示初始页面
    Menu_DisplayMainMenu();
}

/**
 * @brief 主任务函数
 * @note 启动car_task，让陀螺仪z轴保持在30度来笔直前进
 */
static void Menu_RunMainTask(void) {
    CarTaskModeTypeDef task_mode = Menu_GetPendingTaskMode();

    OLED_Clear(0);
    
    // 在屏幕中间显示反色的 "Task Start"
    OLED_ShowString_Reverse(40, 3, "Task Start", 16);
    
    OLED_Refresh();
    
    // 初始化car_task
    Car_Task_SetMode(task_mode);
    Car_Task_Init();
    
    // 启动car_task
    Car_Task_Start();
    
    // 记录任务开始时间
    task_start_time = HAL_GetTick();
    
    // 切换到任务模式
    system_mode = 1;
}

/**
 * @brief 菜单系统运行
 * @note 在主循环中调用
 */
void Menu_Run(void) {
#if OLED_UI_MODE == OLED_UI_MODE_LVGL
    MenuUiMenLvg_Run();
    Debug_Uart_Task();
    return;
#endif

    // 初始化PC6按键
    static uint8_t key_pc6_initialized = 0;
    if (!key_pc6_initialized) {
        KeyPC6_Init();
        key_pc6_initialized = 1;
    }
    
    // 检测PC6按键事件
    uint8_t key_pc6_event = KeyPC6_CheckEvent();
    if (key_pc6_event != 0) {
        if (key_pc6_event == 1) {
            if (system_mode == 0) {
                // 短按：启动主任务
                Menu_StopSpeedStraightTask();
                Menu_StopGyroCircleTask();
                Menu_StopCamTrack();
                Menu_RunMainTask();
            } else {
                // 任务模式下短按：停止主任务
                Car_Task_Stop();
                system_mode = 0;
                OLED_Clear(0);
                OLED_ShowString(40, 3, "Task Stop", 16);
                OLED_Refresh();
                HAL_Delay(1000);
                OLED_Clear(0);
            }
        } else if (key_pc6_event == 2) {
            // 长按：系统重启
            HAL_NVIC_SystemReset();
        }
    }
    
    // 检查任务模式
    if (system_mode == 1) {
        if (Car_Task_IsRunning() == 0U) {
            system_mode = 0;
            OLED_Clear(0);
            Debug_Uart_Task();
            return;
        }

        // 执行car_task主函数
        Car_Task_Main();

        if (Car_Task_IsRunning() == 0U) {
            system_mode = 0;
            OLED_Clear(0);
            Debug_Uart_Task();
            return;
        }

        Debug_Uart_Task();
        // 短暂延迟，避免执行过快
        HAL_Delay(5);
        
        return; // 任务模式下不执行菜单操作
    }
    
    // 获取编码器计数
    int32_t encoderCount = MenuInput_GetEncoderCount();
    
    // 处理编码器输入
    if (encoderCount != menuState.lastEncoderCount) {
        MenuInput_HandleEncoder(encoderCount);
        menuState.lastEncoderCount = encoderCount;
    }
    
    // 获取按键事件
    uint8_t keyEvent = MenuInput_GetEncoderKeyEvent();
    if (keyEvent != 0) {
        MenuInput_HandleKeyEvent(keyEvent);
    }

    if (menu_last_page != menuState.currentPage) {
        if (menu_last_page == MENU_PAGE_SPEED_STRAIGHT_TASK) {
            Menu_StopSpeedStraightTask();
        } else if (menu_last_page == MENU_PAGE_GYRO_CIRCLE_TASK) {
            Menu_StopGyroCircleTask();
        } else if (menu_last_page == MENU_PAGE_CAM_INFO) {
            Menu_StopCamTrack();
        }
        menu_last_page = menuState.currentPage;
    }
    
    // 扫描遥控器按键状态
    MenuInput_ScanRemoteKey();
    
    // 显示当前页面
    Menu_DisplayCurrentPage();

    Debug_Uart_Task();
    
    // 短暂延迟，避免显示刷新过快
    HAL_Delay(5);
}

/**
 * @brief 获取当前菜单状态
 * @return 菜单状态指针
 */
MenuState* Menu_GetState(void) {
    return &menuState;
}

/**
 * @brief 添加新的子菜单
 * @param name 菜单项名称
 * @param page 菜单项对应的页面
 * @return 0-失败, 1-成功
 * @note 此函数需要在 Menu_Init() 之前调用
 */
uint8_t Menu_AddItem(const char* name, MenuPageType page) {
    // 注意：此函数目前仅作为接口，实际添加子菜单需要修改 mainMenuItems 数组
    // 后续可以实现动态添加子菜单的功能
    printf("Menu_AddItem: %s, page: %d\r\n", name, page);
    return 1;
}

/**
 * @brief 获取主菜单项数量
 * @return 主菜单项数量
 */
uint8_t Menu_GetMainItemCount(void) {
    return MAIN_MENU_ITEM_COUNT;
}

/**
 * @brief 获取主菜单项
 * @param index 菜单项索引
 * @return 菜单项指针
 */
const MenuItem* Menu_GetMainItem(uint8_t index) {
    if (index < MAIN_MENU_ITEM_COUNT) {
        return &mainMenuItems[index];
    }
    return NULL;
}

/**
 * @brief PID调试页面处理编码器增量
 * @param delta 编码器增量（正值顺时针，负值逆时针）
 */
void Menu_SpeedPid_HandleEncoderDelta(int32_t delta) {
    int8_t step_dir = 0;
    uint8_t prev_output_enable;

    if (delta > 0) {
        step_dir = 1;
    } else if (delta < 0) {
        step_dir = -1;
    } else {
        return;
    }

    if (speed_pid_edit_mode == 0U) {
        if (step_dir > 0) {
            speed_pid_item_index = (uint8_t)((speed_pid_item_index + 1U) % MENU_SPEED_PID_ITEM_COUNT);
        } else {
            speed_pid_item_index = (uint8_t)((speed_pid_item_index + MENU_SPEED_PID_ITEM_COUNT - 1U) %
                                             MENU_SPEED_PID_ITEM_COUNT);
        }
        return;
    }

    prev_output_enable = speed_pid_output_enable;

    switch ((MenuSpeedPidItemType)speed_pid_item_index) {
        case MENU_SPEED_PID_ITEM_TARGET:
            speed_straight_target_cps = Menu_ClampFloat(
                speed_straight_target_cps + (float)step_dir * MENU_SPEED_TARGET_CPS_STEP,
                MENU_SPEED_TARGET_CPS_MIN, MENU_SPEED_TARGET_CPS_MAX);
            break;
        case MENU_SPEED_PID_ITEM_KP:
            speed_straight_pid_kp = Menu_ClampFloat(
                speed_straight_pid_kp + (float)step_dir * MENU_SPEED_PID_KP_STEP,
                MENU_SPEED_PID_KP_MIN, MENU_SPEED_PID_KP_MAX);
            break;
        case MENU_SPEED_PID_ITEM_KI:
            speed_straight_pid_ki = Menu_ClampFloat(
                speed_straight_pid_ki + (float)step_dir * MENU_SPEED_PID_KI_STEP,
                MENU_SPEED_PID_KI_MIN, MENU_SPEED_PID_KI_MAX);
            break;
        case MENU_SPEED_PID_ITEM_KD:
            speed_straight_pid_kd = Menu_ClampFloat(
                speed_straight_pid_kd + (float)step_dir * MENU_SPEED_PID_KD_STEP,
                MENU_SPEED_PID_KD_MIN, MENU_SPEED_PID_KD_MAX);
            break;
        case MENU_SPEED_PID_ITEM_RUN:
            speed_pid_output_enable = (step_dir > 0) ? 1U : 0U;
            break;
        case MENU_SPEED_PID_ITEM_COUNT:
        default:
            break;
    }

    if (prev_output_enable != speed_pid_output_enable) {
        Menu_SpeedPid_SetOutputEnable(speed_pid_output_enable);
    }
}

/**
 * @brief PID调试页面处理短按事件
 */
void Menu_SpeedPid_HandleShortPress(void) {
    if (speed_pid_item_index == MENU_SPEED_PID_ITEM_RUN) {
        Menu_SpeedPid_SetOutputEnable((uint8_t)(!speed_pid_output_enable));
        return;
    }

    speed_pid_edit_mode = (uint8_t)(!speed_pid_edit_mode);
}

/**
 * @brief Cam Info 页面处理短按事件（开始/停止追踪）
 */
void Menu_CamTrack_HandleShortPress(void) {
    float heading_raw;

    if (cam_track_active != 0U) {
        Menu_StopCamTrack();
        return;
    }

    Motor_Init();
    if (!gyro_initialized) {
        (void)Menu_InitGyro();
    }

    Menu_Gryo_Update();
    heading_raw = Menu_NormalizeAngle(fAngle[2]);
    cam_track_filtered_heading = heading_raw;
    cam_track_target_heading = heading_raw;
    cam_track_x_error_filtered = 0.0f;

    PID_Init(&cam_track_steer_pid,
             MENU_CAM_TRACK_STEER_KP, MENU_CAM_TRACK_STEER_KI, MENU_CAM_TRACK_STEER_KD,
             MENU_CAM_TRACK_STEER_OUT_MAX, MENU_CAM_TRACK_STEER_OUT_MIN);
    PID_Reset(&cam_track_steer_pid);
    PID_Init(&cam_track_heading_pid, MENU_CAM_TRACK_HEADING_KP, MENU_CIRCLE_PID_KI, MENU_CIRCLE_PID_KD,
             MENU_CAM_TRACK_HEADING_OUT_MAX, MENU_CAM_TRACK_HEADING_OUT_MIN);
    PID_Reset(&cam_track_heading_pid);

    cam_track_steer_output = 0.0f;
    cam_track_error = 0.0f;
    cam_track_turn_output = 0.0f;
    cam_track_pwm_left = 0;
    cam_track_pwm_right = 0;
    cam_track_last_tick = HAL_GetTick();
    cam_track_active = 1U;
}

/**
 * @brief 显示陀螺仪信息页面
 */
static void Menu_DisplayGyroPage(void) {
    OLED_Clear(0);
    
    // 首次进入时初始化陀螺仪
    if (!gyro_initialized) {
        Menu_InitGyro();
    }
    
    // 更新陀螺仪数据
    Menu_Gryo_Update();
    
    char str[20];
    char str1[20];
    
    // 改为竖排，与原来的显示样式一致
    sprintf(str, "servo:");
    OLED_ShowString(0, 0, str, 16);
    sprintf(str1, "%.1f", fYaw);
    OLED_ShowString_Reverse(strlen(str)*8, 0, str1, 16);
    
    sprintf(str, " gx:%.1f", fGyro[0]);
    OLED_ShowString(63, 2, str, 12);
    sprintf(str, " gy:%.1f", fGyro[1]);
    OLED_ShowString(63, 3, str, 12);
    sprintf(str, " gz:%.1f", fGyro[2]);
    OLED_ShowString(63, 4, str, 12);

    sprintf(str, " ax:%.1f", fAcc[0]*10);
    OLED_ShowString(63, 5, str, 12);
    sprintf(str, " ay:%.1f", fAcc[1]*10);
    OLED_ShowString(63, 6, str, 12);
    sprintf(str, " az:%.1f", fAcc[2]*10);
    OLED_ShowString(63, 7, str, 12);
	
	
	sprintf(str, "x:%.1f", fAngle[0]);
    OLED_ShowString(0, 2, str, 16);
    sprintf(str, "y:%.1f", fAngle[1]);
    OLED_ShowString(0, 4, str, 16);
    sprintf(str, "z:%.1f", fAngle[2]);
    OLED_ShowString(0, 6, str, 16);
    
    // 舵机控制 - 每次最多移动0.5度，2度死区
    extern float servo_current_angle;
    extern float servo_target_angle;
    float target_angle = servo_current_angle + fAngle[1];
    servo_target_angle = target_angle;
    
    sprintf(str1, "%.1f", target_angle);
    OLED_ShowString_Reverse(strlen(str)*8, 0, str1, 16);
    
    OLED_Refresh();
}

/**
 * @brief 显示EC11测试页面
 */
static void Menu_DisplayEc11TestPage(void) {
    OLED_Clear(0);
    
    // 显示标题
    OLED_ShowString(0, 0, "EC11 Test", 16);
    
    // 初始化PC6按键
    static uint8_t key_pc6_initialized = 0;
    if (!key_pc6_initialized) {
        KeyPC6_Init();
        key_pc6_initialized = 1;
    }
    
    // 获取当前计数
    int32_t currentCount = MenuInput_GetEncoderCount();
    
    // 显示计数值
    char str[20];
    sprintf(str, "Count:%4d", currentCount);
    OLED_ShowString(0, 2, str, 16);
    
    // 显示编码器按键状态
    char str1[20];
    sprintf(str1, "EncKey:%s", MenuInput_GetEncoderKeyState() ? "R" : "P");
    OLED_ShowString(0, 4, str1, 12);
    
    // 显示PC6按键状态
    sprintf(str1, "PC6Key:%s", KeyPC6_GetState() ? "R" : "P");
    OLED_ShowString(0, 5, str1, 12);
    
    // 显示提示信息
    OLED_ShowString(0, 7, "Turn & Press!", 12);
    
    OLED_Refresh();
}

/**
 * @brief 显示TOF测试页面
 */
static void Menu_DisplayTofTestPage(void) {
    OLED_Clear(0);
    
    // 显示标题
    OLED_ShowString(0, 0, "TOF Test", 16);
    
    // 首次进入时初始化TOF
    if (!tof_initialized) {
        Menu_InitTof();
    }
    
    // 读取距离（使用快速测试函数）
    u16 distance_mm = TOF_QuickTest();
    
    // 显示距离值
    char str[20];
    if (distance_mm == 0xFFFF) {
        sprintf(str, "Dist: Error");
    } else if (distance_mm <= 44) {
        // 低于 44mm，显示超出范围
        sprintf(str, "Dist: <4.4cm");
    } else if (distance_mm >= 8190) {
        // 高于 8190mm，显示超出范围
        sprintf(str, "Dist: >120cm");
    } else {
        // 正常范围，转换为 cm 显示
        float distance_cm = distance_mm / 10.0f;
        sprintf(str, "Dist:%5.1f cm", distance_cm);
    }
    OLED_ShowString(0, 2, str, 16);
    
    // 显示状态
    u8 ready = VL53L0X_IsReady();
    sprintf(str, "Status:%s", ready ? "OK" : "ERR");
    OLED_ShowString(0, 4, str, 16);
    
    // 显示提示
    OLED_ShowString(0, 6, "Long press back", 12);
    
    OLED_Refresh();
}

/**
 * @brief 显示 SR04 超声波测试页面
 */
static void Menu_DisplayUltrasonicTestPage(void) {
    OLED_Clear(0);

    // 首次进入时初始化 SR04
    if (!sr04_initialized) {
        SR04_Init();
        sr04_initialized = 1U;
    }
	
	u16 distance_mm = 0U;
	u16 distance_mm_raw = 0U;
    u8 sr04_status = 0;
    char str[20];
	// 读取距离
//	sr04_status = SR04_ReadDistanceMm(&distance_mm);
	sr04_status = SR04_ReadDistanceMmFiltered(&distance_mm_raw,&distance_mm);


    if (sr04_status == SR04_STATUS_OK) {
        sprintf(str, "Dist:%4d mm", distance_mm_raw);
    } else {
        sprintf(str, "Dist:   -- mm");
    }
    OLED_ShowString(0, 0, str, 16);
	
    if (sr04_status == SR04_STATUS_OK) {
        sprintf(str, "DistFL:%4d mm", distance_mm);
    } else {
        sprintf(str, "DistFL:   -- mm");
    }
    OLED_ShowString(0, 2, str, 16);

    sprintf(str, "Status:%d", sr04_status);
    OLED_ShowString(0, 4, str, 16);

    // 显示接线与返回提示
#if SR04_SINGLE_WIRE_ENABLE
    OLED_ShowString(0, 6, "1W C12 T/E", 12);
#else
    OLED_ShowString(0, 6, "C12:T D2:E", 12);
#endif
    OLED_ShowString(0, 7, "Long press back", 12);

    OLED_Refresh();
}

/**
 * @brief 显示水田遍历任务说明页面
 * @note 用于在不改 PC6 按键语义的前提下，给用户一个明确的新任务选择入口
 */
static void Menu_DisplayFieldTaskPage(void) {
    OLED_Clear(0);

    OLED_ShowString(0, 0, "Task Interface", 16);
    OLED_ShowString(0, 2, "PC6:Start/Stop", 12);
    OLED_ShowString(0, 3, "Mode:Field Scan", 12);
    OLED_ShowString(0, 4, "Gyro:WIT PID", 12);
    OLED_ShowString(0, 5, "ENC:Fixed Dist", 12);
    OLED_ShowString(0, 6, "Auto U-turn", 12);
    OLED_ShowString(0, 7, "Long press back", 12);

    OLED_Refresh();
}

/**
 * @brief 显示远程控制任务说明页面
 * @note 通过 PC6 启动后进入 car_task 的 remote 模式，使用 PS2 右摇杆验证运动算法
 */
static void Menu_DisplayRemoteTaskPage(void) {
    OLED_Clear(0);

    OLED_ShowString(0, 0, "Remote Task", 16);
    OLED_ShowString(0, 2, "PC6:Start/Stop", 12);
    OLED_ShowString(0, 3, "PS2 Right Stick", 12);
    OLED_ShowString(0, 4, "Speed PID Loop", 12);
    OLED_ShowString(0, 5, "Show PWM/SPD", 12);
    OLED_ShowString(0, 7, "Long press back", 12);

    OLED_Refresh();
}

/**
 * @brief 编码器速度环闭环直线测试页面
 * @note 支持在线调节后轮速度环 PID 和目标转速，用于双后轮匀速前进调试
 */
static void Menu_DisplaySpeedStraightTaskPage(void) {
    char str[24];
    char marker;
    uint32_t now_tick;
    uint32_t dt_ms;
    float dt_s;
    int32_t enc_left_delta;
    int32_t enc_right_delta;

    if (speed_straight_task_active == 0U) {
        Motor_Init();
        Encoder_Init();

        PID_Init(&speed_straight_left_pid, speed_straight_pid_kp, speed_straight_pid_ki, speed_straight_pid_kd,
                 MENU_SPEED_PID_OUT_MAX, MENU_SPEED_PID_OUT_MIN);
        PID_Init(&speed_straight_right_pid, speed_straight_pid_kp, speed_straight_pid_ki, speed_straight_pid_kd,
                 MENU_SPEED_PID_OUT_MAX, MENU_SPEED_PID_OUT_MIN);
        Menu_SpeedPid_SetOutputEnable(speed_pid_output_enable);
        speed_straight_task_active = 1U;
    }

    now_tick = HAL_GetTick();
    dt_ms = now_tick - speed_straight_last_tick;
    if (dt_ms >= MENU_SPEED_TEST_PERIOD_MS) {
        if (dt_ms > 300U) {
            PID_Reset(&speed_straight_left_pid);
            PID_Reset(&speed_straight_right_pid);
            speed_straight_meas_left = 0.0f;
            speed_straight_meas_right = 0.0f;
            speed_straight_pwm_left = 0;
            speed_straight_pwm_right = 0;
            speed_straight_last_enc_left = 0;
            speed_straight_last_enc_right = 0;
            (void)Encoder_GetCount(ENCODER_1);
            (void)Encoder_GetCount(ENCODER_2);
            Motor_StopAll();
        } else if (speed_pid_output_enable != 0U) {
            dt_s = (float)dt_ms / 1000.0f;
            enc_left_delta = Encoder_GetCount(ENCODER_1);
            enc_right_delta = Encoder_GetCount(ENCODER_2);

            speed_straight_last_enc_left = enc_left_delta;
            speed_straight_last_enc_right = enc_right_delta;

            speed_straight_meas_left = MENU_SPEED_ENCODER_LEFT_SIGN * ((float)enc_left_delta / dt_s);
            speed_straight_meas_right = MENU_SPEED_ENCODER_RIGHT_SIGN * ((float)enc_right_delta / dt_s);

            speed_straight_left_pid.kp = speed_straight_pid_kp;
            speed_straight_left_pid.ki = speed_straight_pid_ki;
            speed_straight_left_pid.kd = speed_straight_pid_kd;
            speed_straight_right_pid.kp = speed_straight_pid_kp;
            speed_straight_right_pid.ki = speed_straight_pid_ki;
            speed_straight_right_pid.kd = speed_straight_pid_kd;

            speed_straight_pwm_left = Menu_ClampMotorPwm((int32_t)PID_Calc(
                &speed_straight_left_pid, speed_straight_meas_left, speed_straight_target_cps));
            speed_straight_pwm_right = Menu_ClampMotorPwm((int32_t)PID_Calc(
                &speed_straight_right_pid, speed_straight_meas_right, speed_straight_target_cps));

            Motor_SetSpeed(MOTOR_FRONT_LEFT, 0);
            Motor_SetSpeed(MOTOR_FRONT_RIGHT, 0);
            Motor_SetSpeed(MOTOR_BACK_LEFT, speed_straight_pwm_left);
            Motor_SetSpeed(MOTOR_BACK_RIGHT, speed_straight_pwm_right);
        } else {
            (void)Encoder_GetCount(ENCODER_1);
            (void)Encoder_GetCount(ENCODER_2);
            speed_straight_meas_left = 0.0f;
            speed_straight_meas_right = 0.0f;
            speed_straight_last_enc_left = 0;
            speed_straight_last_enc_right = 0;
            speed_straight_pwm_left = 0;
            speed_straight_pwm_right = 0;
            Motor_StopAll();
        }
        speed_straight_last_tick = now_tick;
    }

    OLED_Clear(0);
    OLED_ShowString(0, 0, "PID Motor Tune", 12);
    OLED_ShowString(0, 1, "SP:Sel/Edit LP:Back", 12);

    marker = (speed_pid_item_index == MENU_SPEED_PID_ITEM_TARGET) ?
        ((speed_pid_edit_mode != 0U) ? '*' : '>') : ' ';
    (void)snprintf(str, sizeof(str), "%cT:%+4d cps", marker, (int16_t)speed_straight_target_cps);
    OLED_ShowString(0, 2, str, 12);

    marker = (speed_pid_item_index == MENU_SPEED_PID_ITEM_KP) ?
        ((speed_pid_edit_mode != 0U) ? '*' : '>') : ' ';
    (void)snprintf(str, sizeof(str), "%cKp:%1.3f", marker, speed_straight_pid_kp);
    OLED_ShowString(0, 3, str, 12);

    marker = (speed_pid_item_index == MENU_SPEED_PID_ITEM_KI) ?
        ((speed_pid_edit_mode != 0U) ? '*' : '>') : ' ';
    (void)snprintf(str, sizeof(str), "%cKi:%1.3f", marker, speed_straight_pid_ki);
    OLED_ShowString(0, 4, str, 12);

    marker = (speed_pid_item_index == MENU_SPEED_PID_ITEM_KD) ?
        ((speed_pid_edit_mode != 0U) ? '*' : '>') : ' ';
    (void)snprintf(str, sizeof(str), "%cKd:%1.3f", marker, speed_straight_pid_kd);
    OLED_ShowString(0, 5, str, 12);

    (void)snprintf(str, sizeof(str), "M:%+4d %+4d",
                   (int16_t)speed_straight_meas_left, (int16_t)speed_straight_meas_right);
    OLED_ShowString(0, 6, str, 12);

    marker = (speed_pid_item_index == MENU_SPEED_PID_ITEM_RUN) ?
        ((speed_pid_edit_mode != 0U) ? '*' : '>') : ' ';
    (void)snprintf(str, sizeof(str), "%cR:%s P:%+3d %+3d",
                   marker,
                   (speed_pid_output_enable != 0U) ? "ON " : "OFF",
                   speed_straight_pwm_left, speed_straight_pwm_right);
    OLED_ShowString(0, 7, str, 12);
    OLED_Refresh();
}

/**
 * @brief 陀螺仪角度环画圆测试页面
 * @note 目标航向按固定角速度连续变化，通过角度环驱动差速画圆
 */
static void Menu_DisplayGyroCircleTaskPage(void) {
    char str[24];
    uint32_t now_tick;
    uint32_t dt_ms;
    float dt_s;
    float heading_raw;
    float heading_delta;

    if (gyro_circle_task_active == 0U) {
        Motor_Init();

        if (!gyro_initialized) {
            (void)Menu_InitGyro();
        }

        Menu_Gryo_Update();
        heading_raw = Menu_NormalizeAngle(fAngle[2]);
        gyro_circle_filtered_heading = heading_raw;
        gyro_circle_target_heading = heading_raw;
        gyro_circle_error = 0.0f;
        gyro_circle_turn_output = 0.0f;
        gyro_circle_pwm_left = 0;
        gyro_circle_pwm_right = 0;

        PID_Init(&gyro_circle_heading_pid, MENU_CIRCLE_PID_KP, MENU_CIRCLE_PID_KI, MENU_CIRCLE_PID_KD,
                 MENU_CIRCLE_PID_OUT_MAX, MENU_CIRCLE_PID_OUT_MIN);
        PID_Reset(&gyro_circle_heading_pid);

        gyro_circle_last_tick = HAL_GetTick();
        gyro_circle_task_active = 1U;
    }

    now_tick = HAL_GetTick();
    dt_ms = now_tick - gyro_circle_last_tick;
    if (dt_ms >= MENU_CIRCLE_CTRL_PERIOD_MS) {
        if (dt_ms > 300U) {
            PID_Reset(&gyro_circle_heading_pid);
            gyro_circle_turn_output = 0.0f;
            gyro_circle_pwm_left = 0;
            gyro_circle_pwm_right = 0;
            Motor_StopAll();
        } else {
            dt_s = (float)dt_ms / 1000.0f;

            Menu_Gryo_Update();
            heading_raw = Menu_NormalizeAngle(fAngle[2]);
            heading_delta = Menu_NormalizeAngle(heading_raw - gyro_circle_filtered_heading);
            gyro_circle_filtered_heading = Menu_NormalizeAngle(
                gyro_circle_filtered_heading + heading_delta * MENU_CIRCLE_HEADING_LPF_ALPHA);

            gyro_circle_target_heading = Menu_NormalizeAngle(
                gyro_circle_target_heading + MENU_CIRCLE_TARGET_RATE_DPS * dt_s);

            gyro_circle_error = Menu_NormalizeAngle(gyro_circle_target_heading - gyro_circle_filtered_heading);
            gyro_circle_turn_output = PID_Calc(&gyro_circle_heading_pid, -gyro_circle_error, 0.0f);

            gyro_circle_pwm_left = Menu_ClampMotorPwm((int32_t)((float)MENU_CIRCLE_BASE_PWM - gyro_circle_turn_output));
            gyro_circle_pwm_right = Menu_ClampMotorPwm((int32_t)((float)MENU_CIRCLE_BASE_PWM + gyro_circle_turn_output));

            Motor_SetSpeed(MOTOR_FRONT_LEFT, 0);
            Motor_SetSpeed(MOTOR_FRONT_RIGHT, 0);
            Motor_SetSpeed(MOTOR_BACK_LEFT, gyro_circle_pwm_left);
            Motor_SetSpeed(MOTOR_BACK_RIGHT, gyro_circle_pwm_right);
        }
        gyro_circle_last_tick = now_tick;
    }

    OLED_Clear(0);
    OLED_ShowString(0, 0, "Gyro Circle", 16);

    (void)snprintf(str, sizeof(str), "Y:%5.1f T:%5.1f",
                   gyro_circle_filtered_heading, gyro_circle_target_heading);
    OLED_ShowString(0, 2, str, 12);

    (void)snprintf(str, sizeof(str), "E:%6.2f U:%6.1f", gyro_circle_error, gyro_circle_turn_output);
    OLED_ShowString(0, 3, str, 12);

    (void)snprintf(str, sizeof(str), "PWM:%+4d %+4d", gyro_circle_pwm_left, gyro_circle_pwm_right);
    OLED_ShowString(0, 4, str, 12);

    (void)snprintf(str, sizeof(str), "R:%2dcm W:%3d", (int16_t)MENU_CIRCLE_TARGET_RADIUS_CM,
                   (int16_t)MENU_CIRCLE_TARGET_RATE_DPS);
    OLED_ShowString(0, 5, str, 12);

    OLED_ShowString(0, 7, "Long press back", 12);
    OLED_Refresh();
}

/**
 * @brief 显示遥控器测试页面
 */
static void Menu_DisplayRemoteKeyTestPage(void) {
    OLED_Clear(0);
    
    // 显示标题
    OLED_ShowString(0, 0, "Remote Key Test", 16);

    // 首次进入时初始化遥控器
    if (!remote_key_initialized) {
        Menu_InitRemoteKey();
    }
    
    // 显示按键状态
    char str[20];
    for (uint8_t i = 0; i < REMOTE_KEY_MAX; i++) {
        uint8_t state = RemoteKey_GetState((RemoteKeyType)i);
        RemoteKeyEventType event = RemoteKey_GetEvent((RemoteKeyType)i);
        
        switch (i) {
            case REMOTE_KEY_D0:
                sprintf(str, "D0:%s", state ? "P" : "R");
                break;
            case REMOTE_KEY_D1:
                sprintf(str, "D1:%s", state ? "P" : "R");
                break;
            case REMOTE_KEY_D2:
                sprintf(str, "D2:%s", state ? "P" : "R");
                break;
            case REMOTE_KEY_D3:
                sprintf(str, "D3:%s", state ? "P" : "R");
                break;
            default:
                sprintf(str, "Unknown");
                break;
        }
        
        // 显示事件信息
        if (event != REMOTE_KEY_EVENT_NONE) {
            char event_str[10];
            switch (event) {
                case REMOTE_KEY_EVENT_SINGLE_CLICK:
                    sprintf(event_str, "S");
                    break;
                case REMOTE_KEY_EVENT_LONG_PRESS:
                    sprintf(event_str, "L");
                    break;
                case REMOTE_KEY_EVENT_LONG_PRESS_RELEASE:
                    sprintf(event_str, "LR");
                    break;
                default:
                    sprintf(event_str, "N");
                    break;
            }
            sprintf(str + strlen(str), " %s", event_str);
        }
        
        OLED_ShowString(0, (i + 1), str, 12);
    }
    
    // 显示提示
    OLED_ShowString(0, 7, "Press keys!", 12);

    OLED_Refresh();
}
/**
 * @brief 显示PPM输入测试页面
 */
static void Menu_DisplayPPMInputTestPage(void) {
    u16 channels[PPM_IN_MAX_CHANNELS] = {0};
    u8 channel_count = 0;
    u8 stick_value;
    
    OLED_Clear(0);
    OLED_ShowString(0, 0, "PPM In Test", 16);
    
    if (!ppm_initialized) {
        Menu_InitPPM();
    }
    
    if (Menu_IsPPMDriverLinked() == 0U) {
        OLED_ShowString(0, 1, "Drv:Not Linked", 12);
        OLED_ShowString(0, 3, "Add ppm_input.c", 12);
        OLED_ShowString(0, 4, "to Keil target", 12);
        OLED_Refresh();
        return;
    }
    
    if (PPM_In_GetFrame(channels, &channel_count) && PPM_In_IsOnline()) {
        // 显示状态
        OLED_ShowString(0, 1, "State:Online", 12);
        
        // 绘制前4个通道的摇杆栏
        // CH1
        if (channel_count > 0) {
            // 转换脉宽（1000-2000us）为摇杆值（0-255，128为中位）
            stick_value = (u8)((channels[0] - 1000) * 255 / 1000);
            OLED_ShowString(0, 2, "CH1:", 12);
            DrawStickBar(32, 2, 96, stick_value);
        }
        
        // CH2
        if (channel_count > 1) {
            stick_value = (u8)((channels[1] - 1000) * 255 / 1000);
            OLED_ShowString(0, 3, "CH2:", 12);
            DrawStickBar(32, 3, 96, stick_value);
        }
        
        // CH3
        if (channel_count > 2) {
            stick_value = (u8)((channels[2] - 1000) * 255 / 1000);
            OLED_ShowString(0, 4, "CH3:", 12);
            DrawStickBar(32, 4, 96, stick_value);
        }
        
        // CH4
        if (channel_count > 3) {
            stick_value = (u8)((channels[3] - 1000) * 255 / 1000);
            OLED_ShowString(0, 5, "CH4:", 12);
            DrawStickBar(32, 5, 96, stick_value);
        }
        
        // 显示通道数
        char str[24];
        sprintf(str, "Count:%u", channel_count);
        OLED_ShowString(0, 6, str, 12);
    } else {
        // 离线状态
        OLED_ShowString(0, 1, "State:Offline", 12);
        OLED_ShowString(0, 3, "Check PPM wire", 12);
        OLED_ShowString(0, 4, "and EXTI cfg", 12);
    }
    
    OLED_ShowString(0, 7, "Long press back", 12);
    OLED_Refresh();
}

/**
 * @brief 显示SBUS输入测试页面
 */
static void Menu_DisplaySBUSInputTestPage(void) {
    SBUS_InFrameTypeDef frame;
    char str[24];

    OLED_Clear(0);
    OLED_ShowString(0, 0, "SBUS In Test", 16);

    if (!sbus_initialized) {
        Menu_InitSBUS();
    }

    if (Menu_IsSBUSDriverLinked() == 0U) {
        OLED_ShowString(0, 1, "Drv:Not Linked", 12);
        OLED_ShowString(0, 3, "Add sbus_input.c", 12);
        OLED_ShowString(0, 4, "to Keil target", 12);
        OLED_Refresh();
        return;
    }

    if (SBUS_In_GetFrame(&frame) && SBUS_In_IsOnline()) {
        OLED_ShowString(0, 1, "State:Online", 12);

        sprintf(str, "FS:%u Lost:%u", frame.failsafe, frame.frame_lost);
        OLED_ShowString(0, 2, str, 12);

        sprintf(str, "C1:%4u C2:%4u", frame.channels[0], frame.channels[1]);
        OLED_ShowString(0, 3, str, 12);
        sprintf(str, "C3:%4u C4:%4u", frame.channels[2], frame.channels[3]);
        OLED_ShowString(0, 4, str, 12);

        sprintf(str, "C17:%u C18:%u", frame.ch17, frame.ch18);
        OLED_ShowString(0, 6, str, 12);
    } else {
        OLED_ShowString(0, 1, "State:Offline", 12);
        OLED_ShowString(0, 3, "Check UART", 12);
        OLED_ShowString(0, 4, "100k 8E2 INV", 12);
    }

    OLED_Refresh();
}

/**
 * @brief 显示摄像头调试页面（UART1）
 */
static void Menu_DisplayCamInfoPage(void) {
    Cam_Recv_Data_t cam;
    const char *class_str = "NO";
    const char *flag_str = "NO";
    char str[32];

    if (cam_initialized == 0U) {
        Cam_Uart_Init();
        cam_initialized = 1U;
    }

    /* 200ms未收到有效帧即判离线 */
    Cam_Uart_Check_Timeout(200U);
    Menu_UpdateCamTrack();
    (void)Cam_Uart_Fetch(&cam);

    switch (cam.class_id) {
        case CAM_CLASS_BOTTLE:
            class_str = "BOTTLE";
            break;
        case CAM_CLASS_GRASS:
            class_str = "GRASS";
            break;
        case CAM_CLASS_OTHER:
            class_str = "OTHER";
            break;
        default:
            class_str = "NO";
            break;
    }

    switch (cam.flag) {
        case CAM_FLAG_TRACKING:
            flag_str = "TRK";
            break;
        case CAM_FLAG_PREDICTING:
            flag_str = "PRE";
            break;
        default:
            flag_str = "NO";
            break;
    }

    OLED_Clear(0);
    snprintf(str, sizeof(str), "OK:%lu T:%s SP:Move",
             (unsigned long)cam.frame_ok,
             (cam_track_active != 0U) ? "ON" : "OFF");
    OLED_ShowString(0, 0, str, 12);

    snprintf(str, sizeof(str), "C:%s F:%s", class_str, flag_str);
    OLED_ShowString(0, 1, str, 16);

    snprintf(str, sizeof(str), "X:%6d", cam.x);
    OLED_ShowString(0, 3, str, 16);

    snprintf(str, sizeof(str), "Y:%6d", cam.y);
    OLED_ShowString(0, 5, str, 16);

    snprintf(str, sizeof(str), "L:%+4d R:%+4d", cam_track_pwm_left, cam_track_pwm_right);
    OLED_ShowString(0, 7, str, 12);

    OLED_Refresh();
}

/**
 * @brief 显示电机测试页面
 */
static void Menu_DisplayMotorTestPage(void) {
    OLED_Clear(0);
    
    // 首次进入时初始化电机
    if (!motor_initialized) {
        Menu_InitMotor();
    }
    
    char str[20];
    
    // 前左电机
    if (current_motor_index == 0) {
        OLED_ShowString(0, 0, ">", 16);
        OLED_ShowString(20, 0, "FL:", 16);
        sprintf(str, "%4d", motor_speeds[0]);
        if (motor_speed_adjust_mode && current_motor_index == 0) {
            // 速度调整模式下，反色显示速度数字
            OLED_ShowString_Reverse(52, 0, str, 16);
        } else {
            OLED_ShowString(52, 0, str, 16);
        }
    } else {
        OLED_ShowString(20, 0, "FL:", 16);
        sprintf(str, "%4d", motor_speeds[0]);
        OLED_ShowString(52, 0, str, 16);
    }
    
    // 前右电机
    if (current_motor_index == 1) {
        OLED_ShowString(0, 2, ">", 16);
        OLED_ShowString(20, 2, "FR:", 16);
        sprintf(str, "%4d", motor_speeds[1]);
        if (motor_speed_adjust_mode && current_motor_index == 1) {
            // 速度调整模式下，反色显示速度数字
            OLED_ShowString_Reverse(52, 2, str, 16);
        } else {
            OLED_ShowString(52, 2, str, 16);
        }
    } else {
        OLED_ShowString(20, 2, "FR:", 16);
        sprintf(str, "%4d", motor_speeds[1]);
        OLED_ShowString(52, 2, str, 16);
    }
    
    // 后左电机
    if (current_motor_index == 2) {
        OLED_ShowString(0, 4, ">", 16);
        OLED_ShowString(20, 4, "BL:", 16);
        sprintf(str, "%4d", motor_speeds[2]);
        if (motor_speed_adjust_mode && current_motor_index == 2) {
            // 速度调整模式下，反色显示速度数字
            OLED_ShowString_Reverse(52, 4, str, 16);
        } else {
            OLED_ShowString(52, 4, str, 16);
        }
    } else {
        OLED_ShowString(20, 4, "BL:", 16);
        sprintf(str, "%4d", motor_speeds[2]);
        OLED_ShowString(52, 4, str, 16);
    }
    
    // 后右电机
    if (current_motor_index == 3) {
        OLED_ShowString(0, 6, ">", 16);
        OLED_ShowString(20, 6, "BR:", 16);
        sprintf(str, "%4d", motor_speeds[3]);
        if (motor_speed_adjust_mode && current_motor_index == 3) {
            // 速度调整模式下，反色显示速度数字
            OLED_ShowString_Reverse(52, 6, str, 16);
        } else {
            OLED_ShowString(52, 6, str, 16);
        }
    } else {
        OLED_ShowString(20, 6, "BR:", 16);
        sprintf(str, "%4d", motor_speeds[3]);
        OLED_ShowString(52, 6, str, 16);
    }
    
    OLED_Refresh();
}

/**
 * @brief 显示编码器测试页面
 */
static void Menu_DisplayEncoderTestPage(void) {
    OLED_Clear(0);
    
    // 显示标题
    OLED_ShowString(0, 0, "Encoder Test", 16);
    
    // 初始化编码器（电机编码器，使用TIM2和TIM5）
    static uint8_t encoder_initialized = 0;
    if (!encoder_initialized) {
        Encoder_Init();
        encoder_initialized = 1;
    }
    
    // 更新编码器累计值
    encoder1_total_count += Encoder_GetCount(ENCODER_1);
    encoder2_total_count += Encoder_GetCount(ENCODER_2);
    
    // 显示编码器累计值
    char str[20];
    sprintf(str, "Enc1:%6d", encoder1_total_count);
    OLED_ShowString(0, 2, str, 16);
    
    sprintf(str, "Enc2:%6d", encoder2_total_count);
    OLED_ShowString(0, 4, str, 16);
    
    // 显示提示信息
    OLED_ShowString(0, 6, "Turn the motors", 12);
    
    OLED_Refresh();
}

/**
 * @brief 显示ADC测试页面
 */
static void Menu_DisplayAdcTestPage(void) {
    OLED_Clear(0);
    
    // 显示标题
    OLED_ShowString(0, 0, "ADC Test", 16);
    
#
    // 首次进入时初始化ADC
    if (!adc_initialized) {
        Menu_InitAdc();
    }
    
    // 启动ADC转换
    HAL_ADC_Start(&hadc1);
    
    // 等待转换完成
    HAL_ADC_PollForConversion(&hadc1, 100);
    
    // 读取ADC值
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
    
    // 停止ADC
    HAL_ADC_Stop(&hadc1);
    
    // 转换为电压值（V）
    float voltage = (float)adc_value * 3.3f / 4095.0f;
    
    // 显示电压值
    char str[20];
    sprintf(str, "Voltage:%.2f V", voltage);
    OLED_ShowString(0, 2, str, 16);
    
    // 显示ADC原始值
    sprintf(str, "ADC Value:%4u", adc_value);
    OLED_ShowString(0, 4, str, 16);
    
    // 显示提示信息
    OLED_ShowString(0, 6, "PC4 as ADC input", 12);
    
    OLED_Refresh();
}

// PS2手柄数据结构
static JOYSTICK_TypeDef ps2_joystick;

/**
 * @brief 显示PS2手柄测试页面
 * @note 使用ax_ps2驱动，只显示按键值
 */
static void Menu_DisplayPS2TestPage(void) {
    OLED_Clear(0);
    
    // 首次进入时初始化PS2
    if (!ps2_initialized) {
        Menu_InitPS2();
    }
    
    // 按键震动反馈状态
    static uint8_t last_btn1 = 0xFF;
    static uint8_t last_btn2 = 0xFF;
    static uint32_t vibration_start_time = 0;
    static uint8_t is_vibrating = 0;
    static uint8_t motor1 = 0;
    static uint8_t motor2 = 0;
    
    // 检测按键变化
    if ((ps2_joystick.btn1 != last_btn1) || (ps2_joystick.btn2 != last_btn2)) {
        // 有按键按下时震动
        if ((ps2_joystick.btn1 != 0xFF) || (ps2_joystick.btn2 != 0xFF)) {
            motor1 = 0xFF;  // 大电机全速
            motor2 = 0x01;  // 小电机开启
            vibration_start_time = HAL_GetTick();
            is_vibrating = 1;
        }
    }
    
    // 震动持续100ms后关闭
    if (is_vibrating && (HAL_GetTick() - vibration_start_time > 100)) {
        motor1 = 0x00;  // 关闭大电机
        motor2 = 0x00;  // 关闭小电机
        is_vibrating = 0;
    }
    
    last_btn1 = ps2_joystick.btn1;
    last_btn2 = ps2_joystick.btn2;
    
    // 读取PS2数据（带震动）
    AX_PS2_ScanKeyVibration(&ps2_joystick, motor1, motor2);
    
    // 显示模式和原始键值 (12号字体，第0行)
    char str[20];
    sprintf(str, "M:%02X B1:%02X B2:%02X", ps2_joystick.mode, ps2_joystick.btn1, ps2_joystick.btn2);
    OLED_ShowString(0, 0, str, 12);
    
    // 显示右摇杆数值 (第1行)
    sprintf(str, "RX:%3d RY:%3d", ps2_joystick.RJoy_LR, ps2_joystick.RJoy_UD);
    OLED_ShowString(0, 1, str, 12);
    
    // 显示左摇杆数值 (第2行)
    sprintf(str, "LX:%3d LY:%3d", ps2_joystick.LJoy_LR, ps2_joystick.LJoy_UD);
    OLED_ShowString(0, 2, str, 12);
    
    // 显示方向键状态 (第3行)
    // btn1: B0:SLCT B1:JR  B2:JL B3:STRT B4:UP B5:R B6:DOWN B7:L
    OLED_ShowString(0, 3, "DPad:", 12);
    if (ps2_joystick.btn1 & 0x10) OLED_ShowString(36, 3, "U", 12);
    else OLED_ShowString(36, 3, "-", 12);
    if (ps2_joystick.btn1 & 0x40) OLED_ShowString(48, 3, "D", 12);
    else OLED_ShowString(48, 3, "-", 12);
    if (ps2_joystick.btn1 & 0x80) OLED_ShowString(60, 3, "L", 12);
    else OLED_ShowString(60, 3, "-", 12);
    if (ps2_joystick.btn1 & 0x20) OLED_ShowString(72, 3, "R", 12);
    else OLED_ShowString(72, 3, "-", 12);
    
    // 显示按键状态 (第4行)
    // btn2: B0:L2 B1:R2 B2:L1 B3:R1 B4:Y B5:B B6:A B7:X
    OLED_ShowString(0, 4, "Key:", 12);
    if (ps2_joystick.btn2 & 0x10) OLED_ShowString(30, 4, "Y", 12);
    else OLED_ShowString(30, 4, "-", 12);
    if (ps2_joystick.btn2 & 0x20) OLED_ShowString(42, 4, "B", 12);
    else OLED_ShowString(42, 4, "-", 12);
    if (ps2_joystick.btn2 & 0x40) OLED_ShowString(54, 4, "A", 12);
    else OLED_ShowString(54, 4, "-", 12);
    if (ps2_joystick.btn2 & 0x80) OLED_ShowString(66, 4, "X", 12);
    else OLED_ShowString(66, 4, "-", 12);
    
    // 显示肩键 (第5行) - 顺序：L1 L2 R1 R2
    // btn2: B0:L2   B1:R2  B2:L1  B3:R1
    OLED_ShowString(0, 5, "L1:", 12);
    OLED_ShowString(21, 5, (ps2_joystick.btn2 & 0x04) ? "1" : "0", 12);
    OLED_ShowString(36, 5, "L2:", 12);
    OLED_ShowString(57, 5, (ps2_joystick.btn2 & 0x01) ? "1" : "0", 12);
    OLED_ShowString(69, 5, "R1:", 12);
    OLED_ShowString(90, 5, (ps2_joystick.btn2 & 0x08) ? "1" : "0", 12);
    OLED_ShowString(102, 5, "R2:", 12);
    OLED_ShowString(123, 5, (ps2_joystick.btn2 & 0x02) ? "1" : "0", 12);
    
    // 显示摇杆按下按键 (第6行) - 顺序：JR JL ST SL
    // btn1: B0:SLCT B1:JR  B2:JL B3:STRT
    OLED_ShowString(0, 6, "JR:", 12);
    OLED_ShowString(24, 6, (ps2_joystick.btn1 & 0x02) ? "1" : "0", 12);
    OLED_ShowString(36, 6, "JL:", 12);
    OLED_ShowString(60, 6, (ps2_joystick.btn1 & 0x04) ? "1" : "0", 12);
    OLED_ShowString(72, 6, "ST:", 12);
    OLED_ShowString(96, 6, (ps2_joystick.btn1 & 0x08) ? "1" : "0", 12);
    OLED_ShowString(108, 6, "SL:", 12);
    OLED_ShowString(120, 6, (ps2_joystick.btn1 & 0x01) ? "1" : "0", 12);
    
    OLED_Refresh();
}

/**
 * @brief 显示PS2控制模式页面
 * @note 使用PS2手柄摇杆控制PWM输出（航模PWM模式：50Hz，1000-2000us）
 *       左摇杆Y->B1, 左摇杆X->B0, 右摇杆Y->A7, 右摇杆X->A6
 */
static void Menu_DisplayPS2ControlPage(void) {
    static MenuPageType last_page = MENU_PAGE_MAIN;
    static uint8_t servo_initialized = 0;
    
    // 检测页面切换（从 PS2_CONTROL 切换到其他页面）
    if (last_page == MENU_PAGE_PS2_CONTROL && menuState.currentPage != MENU_PAGE_PS2_CONTROL) {
        Servo_Pwm_StopAll();
    }
    last_page = menuState.currentPage;
    
    // 如果当前不是 PS2_CONTROL 页面，直接返回
    if (menuState.currentPage != MENU_PAGE_PS2_CONTROL) {
        return;
    }
    
    OLED_Clear(0);
    
    // 首次进入时初始化PS2和舵机PWM
    if (!ps2_initialized) {
        Menu_InitPS2();
    }
    if (!servo_initialized) {
        Servo_Pwm_Init();
        servo_initialized = 1;
    }
    
    // 读取PS2数据，带死区处理
    AX_PS2_ScanKey_Deadzone(&ps2_joystick);

    // 摇杆值(0-255)转换为PWM脉宽(1000-2000us)，128为中位
    // 公式: pulse = 1500 + (stick - 128) * 500 / 127
    // 左摇杆Y -> B1
    uint16_t left_y_pwm = 1500 + (int16_t)(ps2_joystick.LJoy_UD - 128) * 500 / 127;
    // 左摇杆X -> B0
    uint16_t left_x_pwm = 1500 + (int16_t)(ps2_joystick.LJoy_LR - 128) * 500 / 127;
    // 右摇杆Y -> A7
    uint16_t right_y_pwm = 1500 + (int16_t)(ps2_joystick.RJoy_UD - 128) * 500 / 127;
    // 右摇杆X -> A6
    uint16_t right_x_pwm = 1500 + (int16_t)(ps2_joystick.RJoy_LR - 128) * 500 / 127;
    
    // 设置死区，避免摇杆中位时的抖动
    // 死区设置在摇杆里

    // 设置PWM输出
    Servo_Pwm_SetPulse(SERVO_PWM_CH_B1, left_y_pwm);
    Servo_Pwm_SetPulse(SERVO_PWM_CH_B0, left_x_pwm);
    Servo_Pwm_SetPulse(SERVO_PWM_CH_A7, right_y_pwm);
    Servo_Pwm_SetPulse(SERVO_PWM_CH_A6, right_x_pwm);
    
    // 使用16号字体，每行占2个page（16像素高度）
    // 第0-1行：LX（左摇杆X轴）
    OLED_ShowString(0, 0, "LX:", 16);
    DrawStickBar(28, 4, 100, ps2_joystick.LJoy_LR);
    
    // 第2-3行：LY（左摇杆Y轴）
    OLED_ShowString(0, 2, "LY:", 16);
    DrawStickBar(28, 20, 100, ps2_joystick.LJoy_UD);
    
    // 第4-5行：RX（右摇杆X轴）
    OLED_ShowString(0, 4, "RX:", 16);
    DrawStickBar(28, 36, 100, ps2_joystick.RJoy_LR);
    
    // 第6-7行：RY（右摇杆Y轴）
    OLED_ShowString(0, 6, "RY:", 16);
    DrawStickBar(28, 52, 100, ps2_joystick.RJoy_UD);
    
    OLED_Refresh();
}
/**
 * @brief 显示舵机控制界面
 * @note 舵机运动，按时间转一圈
 *     （航模PWM模式：50Hz，1000-2000us）
 *       左摇杆Y->B1, 左摇杆X->B0, 右摇杆Y->A7, 右摇杆X->A6
 */
static void Menu_DisplayServoPWMPage(void) {
    static MenuPageType last_page = MENU_PAGE_MAIN;
    static uint8_t servo_initialized = 0;
    
    // 检测页面退出（退出到其他页面）
    if (last_page == MENU_PAGE_SERVO_PWM && menuState.currentPage != MENU_PAGE_SERVO_PWM) {
        Servo_Pwm_StopAll();
    }
    last_page = menuState.currentPage;
    
    // 如果当前不是 SERVO_PWM 页面，直接返回
    if (menuState.currentPage != MENU_PAGE_SERVO_PWM) {
        return;
    }
    
    OLED_Clear(0);
    
    // 首次进入时初始化PS2和舵机PWM
    if (!ps2_initialized) {
        Menu_InitPS2();
    }
    if (!servo_initialized) {
        Servo_Pwm_Init();
        servo_initialized = 1;
    }
    
    // 定义舵机角度变量
    static float servo_angle = 90.0f;
    static float servo_speed = 0.3f;

    // 周期旋转，换向
    if (servo_angle >= 180 || servo_angle <= 0) servo_speed = -servo_speed;

    servo_angle += servo_speed;

    // 转换为PWM脉宽(500-2500us)
    uint16_t servo_pwm = 500 + (int16_t)(servo_angle * 2000 / 180);

    // 设置PWM输出
    Servo_Pwm_SetPulse(SERVO_PWM_CH_B1, servo_pwm);
    Servo_Pwm_SetPulse(SERVO_PWM_CH_B0, servo_pwm);
    Servo_Pwm_SetPulse(SERVO_PWM_CH_A7, servo_pwm);
    Servo_Pwm_SetPulse(SERVO_PWM_CH_A6, servo_pwm);
    
    char str[30];
    // 使用16号字体，每行占2个page（16像素高度）
    // 第0-1行：Ag（舵机角度）
    snprintf(str, sizeof(str), "Ag:%.1f", servo_angle);
    OLED_ShowString(0, 0, str, 16);

    snprintf(str, sizeof(str), "sp:%.1f", servo_speed);
    OLED_ShowString(0, 2, str, 16);

    // 第4-5行：Ag（舵机角度）
    DrawStickBar(0, 40 , 128, (servo_angle)/180*256);

    OLED_Refresh();
}

// GPIO测试引脚状态（用函数调用次数计数）
static uint8_t gpio_cycle_counter[4] = {0, 0, 0, 0};    // 周期计数器（0-9）

// 每个引脚在10次函数调用中的高电平次数
static const uint8_t gpio_pin_high_times[4] = {1, 2, 3, 4};
#define GPIO_CYCLE_MAX  10  // 10次函数调用为一个周期

/**
 * @brief 初始化GPIO测试引脚（PC0-PC3）
 */
static void Menu_InitGPIOTest(void) {
    if (gpio_initialized) return;
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 启用GPIOC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // 配置PC0-PC3为推挽输出
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 初始状态为低电平
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
    
    // 初始化计数器
    for (int i = 0; i < 4; i++) {
        gpio_cycle_counter[i] = 0;
    }
    
    gpio_initialized = 1;
}

/**
 * @brief 显示GPIO测试页面
 * @note 测试PC0-PC3引脚，10次函数调用一个周期，不同引脚不同高电平次数
 */
static void Menu_DisplayGPIOTestPage(void) {
    static MenuPageType last_page = MENU_PAGE_MAIN;
    
    // 检测页面切换（从 GPIO_TEST 切换到其他页面）
    if (last_page == MENU_PAGE_GPIO_TEST && menuState.currentPage != MENU_PAGE_GPIO_TEST) {
        // 停止 PWM，将引脚设为高电平（PS2通信默认状态）
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
        gpio_initialized = 0;
    }
    last_page = menuState.currentPage;
    
    // 如果当前不是 GPIO_TEST 页面，直接返回
    if (menuState.currentPage != MENU_PAGE_GPIO_TEST) {
        return;
    }
    
    // 初始化GPIO（首次进入）
    if (!gpio_initialized) {
        Menu_InitGPIOTest();
    }
    
    // 先更新GPIO输出
    uint16_t pins[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
    
    for (int i = 0; i < 4; i++) {
        // 根据计数器决定输出：计数器小于高电平次数则为高，否则为低
        if (gpio_cycle_counter[i] < gpio_pin_high_times[i]) {
            HAL_GPIO_WritePin(GPIOC, pins[i], GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, pins[i], GPIO_PIN_RESET);
        }
        
        // 计数器递增
        gpio_cycle_counter[i]++;
        if (gpio_cycle_counter[i] >= GPIO_CYCLE_MAX) {
            gpio_cycle_counter[i] = 0;
        }
    }
    
    // 显示界面
    OLED_Clear(0);
    
    // 显示标题
    OLED_ShowString(0, 0, "GPIO Test", 16);
    
    // 显示引脚信息
    OLED_ShowString(0, 2, "10 cycles PWM", 16);

    // 显示各引脚高电平次数
    char str[30];

    // 显示各引脚高电平次数
    sprintf(str, "PC0:%d   PC1:%d", 
            gpio_pin_high_times[0], gpio_pin_high_times[1]);
    OLED_ShowString(0, 4, str, 16);

    sprintf(str, "PC2:%d   PC3:%d", 
        gpio_pin_high_times[2], gpio_pin_high_times[3]);
    OLED_ShowString(0, 6, str, 16);
    
    
    OLED_Refresh();
}

/**
 * @brief 显示主菜单
 * @note 原来的样式，垂直列表布局，支持滚动
 */
static void Menu_DisplayMainMenu(void) {
    OLED_Clear(0);
    
    // 垂直行布局参数
    #define MAX_VISIBLE_ITEMS 4  // 16字大小对应4行

    // 使用记忆滚动窗口，避免上滑时光标长期停留在最底行
    static uint8_t scroll_offset = 0;
    static uint8_t last_item = 0xFF;

    if (MAIN_MENU_ITEM_COUNT <= MAX_VISIBLE_ITEMS) {
        scroll_offset = 0;
    } else {
        uint8_t max_scroll_offset = MAIN_MENU_ITEM_COUNT - MAX_VISIBLE_ITEMS;

        if (last_item == 0xFF) {
            if (menuState.currentItem > max_scroll_offset) {
                scroll_offset = max_scroll_offset;
            } else {
                scroll_offset = menuState.currentItem;
            }
        }

        if (menuState.currentItem < scroll_offset) {
            scroll_offset = menuState.currentItem;
        } else if (menuState.currentItem >= (uint8_t)(scroll_offset + MAX_VISIBLE_ITEMS)) {
            scroll_offset = menuState.currentItem - MAX_VISIBLE_ITEMS + 1;
        }

        if (scroll_offset > max_scroll_offset) {
            scroll_offset = max_scroll_offset;
        }
    }
    last_item = menuState.currentItem;
    
    // 垂直行布局，每个菜单项占一行，16字大小
    for (uint8_t i = 0; i < MAIN_MENU_ITEM_COUNT; i++) {
        // 只显示在可见范围内的菜单项
        if (i >= scroll_offset && i < scroll_offset + MAX_VISIBLE_ITEMS) {
            uint8_t display_row = i - scroll_offset;
            if (i == menuState.currentItem) {
                // 显示选中的菜单项（带方框标记）
                OLED_ShowString(0, display_row * 2, "[", 16);
                OLED_ShowString(12, display_row * 2, mainMenuItems[i].name, 16);
                OLED_ShowString(112, display_row * 2, "]", 16);
            } else {
                // 显示未选中的菜单项
                OLED_ShowString(12, display_row * 2, mainMenuItems[i].name, 16);
            }
        }
    }
    
    OLED_Refresh();
}

/**
 * @brief 显示当前页面
 */
static void Menu_DisplayCurrentPage(void) {
    switch (menuState.currentPage) {
        case MENU_PAGE_MAIN:
            Menu_DisplayMainMenu();
            break;
        case MENU_PAGE_GYRO:
            Menu_DisplayGyroPage();
            break;
        case MENU_PAGE_CAM_INFO:
            Menu_DisplayCamInfoPage();
            break;
        case MENU_PAGE_EC11_TEST:
            Menu_DisplayEc11TestPage();
            break;
        case MENU_PAGE_TOF_TEST:
            Menu_DisplayTofTestPage();
            break;
        case MENU_PAGE_SR04_TEST:
            Menu_DisplayUltrasonicTestPage();
            break;
        case MENU_PAGE_FIELD_TASK:
            Menu_DisplayFieldTaskPage();
            break;
        case MENU_PAGE_REMOTE_TASK:
            Menu_DisplayRemoteTaskPage();
            break;
        case MENU_PAGE_SPEED_STRAIGHT_TASK:
            Menu_DisplaySpeedStraightTaskPage();
            break;
        case MENU_PAGE_GYRO_CIRCLE_TASK:
            Menu_DisplayGyroCircleTaskPage();
            break;
        case MENU_PAGE_REMOTE_KEY_TEST:
            Menu_DisplayRemoteKeyTestPage();
            break;
        case MENU_PAGE_PPM_INPUT_TEST:
            Menu_DisplayPPMInputTestPage();
            break;
        case MENU_PAGE_SBUS_INPUT_TEST:
            Menu_DisplaySBUSInputTestPage();
            break;
        case MENU_PAGE_MOTOR_TEST:
            Menu_DisplayMotorTestPage();
            break;
        case MENU_PAGE_ENCODER_TEST:
            Menu_DisplayEncoderTestPage();
            break;
        case MENU_PAGE_ADC_TEST:
            Menu_DisplayAdcTestPage();
            break;
        case MENU_PAGE_PS2_TEST:
            Menu_DisplayPS2TestPage();
            break;
        case MENU_PAGE_PS2_CONTROL:
            Menu_DisplayPS2ControlPage();
            break;
        case MENU_PAGE_SERVO_PWM:
            Menu_DisplayServoPWMPage();
            break;
        case MENU_PAGE_GPIO_TEST:
            Menu_DisplayGPIOTestPage();
            break;
        default:
            Menu_DisplayMainMenu();
            break;
    }
}



