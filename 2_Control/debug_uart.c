#include "debug_uart.h"

#include "menu.h"
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

/* 接收命令行最大长度（包含结束符） */
#define DEBUG_UART_RX_LINE_MAX           128U
/* 单次发送缓存长度 */
#define DEBUG_UART_TX_BUF_MAX            256U
/* 默认周期上报间隔（ms） */
#define DEBUG_UART_STREAM_PERIOD_DEFAULT 100U
/* 周期上报最小/最大间隔，避免异常配置影响主循环 */
#define DEBUG_UART_STREAM_PERIOD_MIN     20U
#define DEBUG_UART_STREAM_PERIOD_MAX     2000U


/* 选择调试串口 句柄（只用于发送，接收字节由中断回调喂入） */
static UART_HandleTypeDef *s_debug_huart = NULL;

/* ISR中逐字节拼接命令行缓存 */
static char s_isr_line_buf[DEBUG_UART_RX_LINE_MAX];
static uint8_t s_isr_line_len = 0U;

/* 主循环待处理命令缓存（ISR填充，主循环消费） */
static volatile uint8_t s_cmd_ready = 0U;
static char s_cmd_line_buf[DEBUG_UART_RX_LINE_MAX];

/* 周期上报控制 */
static uint8_t s_stream_enable = 1U;
static uint16_t s_stream_period_ms = DEBUG_UART_STREAM_PERIOD_DEFAULT;
static uint32_t s_last_stream_tick = 0U;
static uint8_t s_send_once = 0U;

/* 发送文本缓存 */
static char s_tx_buf[DEBUG_UART_TX_BUF_MAX];

/* 统一串口发送：发送失败时静默丢弃，避免阻塞控制链路 */
static void Debug_Uart_SendLine(const char *fmt, ...)
{
    va_list ap;
    int len;

    if ((s_debug_huart == NULL) || (fmt == NULL)) {
        return;
    }

    va_start(ap, fmt);
    len = vsnprintf(s_tx_buf, sizeof(s_tx_buf), fmt, ap);
    va_end(ap);

    if (len <= 0) {
        return;
    }
    if (len >= (int)sizeof(s_tx_buf)) {
        len = (int)sizeof(s_tx_buf) - 1;
    }

    (void)HAL_UART_Transmit(s_debug_huart, (uint8_t *)s_tx_buf, (uint16_t)len, 20U);
}

/* 字符串转 float，成功返回1，失败返回0 */
static uint8_t Debug_Uart_ParseFloat(const char *str, float *out)
{
    char *endptr;
    float value;

    if ((str == NULL) || (out == NULL)) {
        return 0U;
    }

    value = strtof(str, &endptr);
    if ((endptr == str) || (*endptr != '\0')) {
        return 0U;
    }

    *out = value;
    return 1U;
}

/* 字符串转 int32，成功返回1，失败返回0 */
static uint8_t Debug_Uart_ParseInt(const char *str, int32_t *out)
{
    char *endptr;
    long value;

    if ((str == NULL) || (out == NULL)) {
        return 0U;
    }

    value = strtol(str, &endptr, 10);
    if ((endptr == str) || (*endptr != '\0')) {
        return 0U;
    }

    *out = (int32_t)value;
    return 1U;
}

/* 发送一次完整状态快照，供AI侧按行解析 */
static void Debug_Uart_SendSnapshot(void)
{
    MenuDebugState_t st;

    memset(&st, 0, sizeof(st));
    Menu_Debug_GetState(&st);

    Debug_Uart_SendLine(
        "$ST,t=%lu,mode=%u,page=%u,gyro=%u,cam=%u,yaw=%.2f,gx=%.2f,gy=%.2f,gz=%.2f,ax=%.3f,ay=%.3f,az=%.3f\r\n",
        st.tick_ms, st.system_mode, st.page, st.gyro_ready, st.cam_online,
        st.yaw_deg, st.gyro_x_dps, st.gyro_y_dps, st.gyro_z_dps,
        st.acc_x_g, st.acc_y_g, st.acc_z_g);

    Debug_Uart_SendLine(
        "$SPD,kp=%.4f,ki=%.4f,kd=%.4f,tar=%.1f,ml=%.1f,mr=%.1f,pl=%d,pr=%d,en=%u\r\n",
        st.speed_kp, st.speed_ki, st.speed_kd, st.speed_target_cps,
        st.speed_meas_left_cps, st.speed_meas_right_cps,
        st.speed_pwm_left, st.speed_pwm_right, st.speed_output_enable);

    Debug_Uart_SendLine(
        "$CIR,kp=%.4f,ki=%.4f,kd=%.4f,ref=%.2f,yaw=%.2f,err=%.2f,pl=%d,pr=%d\r\n",
        st.circle_kp, st.circle_ki, st.circle_kd,
        st.circle_target_heading, st.circle_filtered_heading, st.circle_error,
        st.circle_pwm_left, st.circle_pwm_right);

    Debug_Uart_SendLine("$MOT,fl=%d,fr=%d,bl=%d,br=%d\r\n",
                        st.motor_fl, st.motor_fr, st.motor_bl, st.motor_br);
}

/* 响应帮助信息，便于串口工具直接查看支持的指令 */
static void Debug_Uart_SendHelp(void)
{
    Debug_Uart_SendLine("$HELP,GET,CFG,SET\r\n");
    Debug_Uart_SendLine("$HELP,GET,ONCE\r\n");
    Debug_Uart_SendLine("$HELP,CFG,STREAM,0|1\r\n");
    Debug_Uart_SendLine("$HELP,CFG,PERIOD,20~2000(ms)\r\n");
    Debug_Uart_SendLine("$HELP,SET,SPD,KP|KI|KD|TAR|EN,value\r\n");
    Debug_Uart_SendLine("$HELP,SET,SPD,ALL,kp,ki,kd,target,en\r\n");
    Debug_Uart_SendLine("$HELP,SET,CIR,KP|KI|KD,value\r\n");
    Debug_Uart_SendLine("$HELP,SET,CIR,ALL,kp,ki,kd\r\n");
}

/* 处理 SET,SPD 指令 */
static void Debug_Uart_HandleSetSpeed(char *field, char *v1, char *v2, char *v3, char *v4, char *v5)
{
    float kp, ki, kd, tar;
    int32_t en_i32;
    uint8_t ok = 0U;

    if ((field == NULL) || (v1 == NULL)) {
        Debug_Uart_SendLine("$ERR,msg=SET,SPD,missing_param\r\n");
        return;
    }

    if (strcmp(field, "KP") == 0) {
        if (Debug_Uart_ParseFloat(v1, &kp) != 0U) {
            ok = Menu_Debug_SetSpeedPid(kp, 0.0f, 0.0f, 0.0f, 0U, MENU_DEBUG_PID_MASK_KP);
        }
    } else if (strcmp(field, "KI") == 0) {
        if (Debug_Uart_ParseFloat(v1, &ki) != 0U) {
            ok = Menu_Debug_SetSpeedPid(0.0f, ki, 0.0f, 0.0f, 0U, MENU_DEBUG_PID_MASK_KI);
        }
    } else if (strcmp(field, "KD") == 0) {
        if (Debug_Uart_ParseFloat(v1, &kd) != 0U) {
            ok = Menu_Debug_SetSpeedPid(0.0f, 0.0f, kd, 0.0f, 0U, MENU_DEBUG_PID_MASK_KD);
        }
    } else if (strcmp(field, "TAR") == 0) {
        if (Debug_Uart_ParseFloat(v1, &tar) != 0U) {
            ok = Menu_Debug_SetSpeedPid(0.0f, 0.0f, 0.0f, tar, 0U, MENU_DEBUG_PID_MASK_TARGET);
        }
    } else if (strcmp(field, "EN") == 0) {
        if (Debug_Uart_ParseInt(v1, &en_i32) != 0U) {
            ok = Menu_Debug_SetSpeedPid(0.0f, 0.0f, 0.0f, 0.0f, (en_i32 != 0) ? 1U : 0U, MENU_DEBUG_PID_MASK_ENABLE);
        }
    } else if (strcmp(field, "ALL") == 0) {
        if ((v2 != NULL) && (v3 != NULL) && (v4 != NULL) && (v5 != NULL) &&
            (Debug_Uart_ParseFloat(v1, &kp) != 0U) &&
            (Debug_Uart_ParseFloat(v2, &ki) != 0U) &&
            (Debug_Uart_ParseFloat(v3, &kd) != 0U) &&
            (Debug_Uart_ParseFloat(v4, &tar) != 0U) &&
            (Debug_Uart_ParseInt(v5, &en_i32) != 0U)) {
            ok = Menu_Debug_SetSpeedPid(
                kp, ki, kd, tar, (en_i32 != 0) ? 1U : 0U,
                (MENU_DEBUG_PID_MASK_KP | MENU_DEBUG_PID_MASK_KI | MENU_DEBUG_PID_MASK_KD |
                 MENU_DEBUG_PID_MASK_TARGET | MENU_DEBUG_PID_MASK_ENABLE));
        }
    } else {
        Debug_Uart_SendLine("$ERR,msg=SET,SPD,unknown_field\r\n");
        return;
    }

    if (ok != 0U) {
        s_send_once = 1U;
        Debug_Uart_SendLine("$ACK,msg=SET,SPD,ok\r\n");
    } else {
        Debug_Uart_SendLine("$ERR,msg=SET,SPD,invalid_value\r\n");
    }
}

/* 处理 SET,CIR 指令 */
static void Debug_Uart_HandleSetCircle(char *field, char *v1, char *v2, char *v3)
{
    float kp, ki, kd;
    uint8_t ok = 0U;

    if ((field == NULL) || (v1 == NULL)) {
        Debug_Uart_SendLine("$ERR,msg=SET,CIR,missing_param\r\n");
        return;
    }

    if (strcmp(field, "KP") == 0) {
        if (Debug_Uart_ParseFloat(v1, &kp) != 0U) {
            ok = Menu_Debug_SetCirclePid(kp, 0.0f, 0.0f, MENU_DEBUG_PID_MASK_KP);
        }
    } else if (strcmp(field, "KI") == 0) {
        if (Debug_Uart_ParseFloat(v1, &ki) != 0U) {
            ok = Menu_Debug_SetCirclePid(0.0f, ki, 0.0f, MENU_DEBUG_PID_MASK_KI);
        }
    } else if (strcmp(field, "KD") == 0) {
        if (Debug_Uart_ParseFloat(v1, &kd) != 0U) {
            ok = Menu_Debug_SetCirclePid(0.0f, 0.0f, kd, MENU_DEBUG_PID_MASK_KD);
        }
    } else if (strcmp(field, "ALL") == 0) {
        if ((v2 != NULL) && (v3 != NULL) &&
            (Debug_Uart_ParseFloat(v1, &kp) != 0U) &&
            (Debug_Uart_ParseFloat(v2, &ki) != 0U) &&
            (Debug_Uart_ParseFloat(v3, &kd) != 0U)) {
            ok = Menu_Debug_SetCirclePid(kp, ki, kd,
                                         (MENU_DEBUG_PID_MASK_KP | MENU_DEBUG_PID_MASK_KI | MENU_DEBUG_PID_MASK_KD));
        }
    } else {
        Debug_Uart_SendLine("$ERR,msg=SET,CIR,unknown_field\r\n");
        return;
    }

    if (ok != 0U) {
        s_send_once = 1U;
        Debug_Uart_SendLine("$ACK,msg=SET,CIR,ok\r\n");
    } else {
        Debug_Uart_SendLine("$ERR,msg=SET,CIR,invalid_value\r\n");
    }
}

/* 主循环命令处理（耗时逻辑统一放到主循环，不放ISR） */
static void Debug_Uart_ProcessCommand(char *line)
{
    char *cmd;
    char *arg1;
    char *arg2;
    char *arg3;
    char *arg4;
    char *arg5;
    char *arg6;
    char *arg7;
    int32_t value_i32;

    if (line == NULL) {
        return;
    }

    cmd = strtok(line, ",");
    if (cmd == NULL) {
        return;
    }

    if ((strcmp(cmd, "$HELP") == 0) || (strcmp(cmd, "$H") == 0)) {
        Debug_Uart_SendHelp();
        return;
    }

    if (strcmp(cmd, "$GET") == 0) {
        arg1 = strtok(NULL, ",");
        if ((arg1 != NULL) && (strcmp(arg1, "ONCE") == 0)) {
            s_send_once = 1U;
            Debug_Uart_SendLine("$ACK,msg=GET,ONCE,ok\r\n");
        } else {
            Debug_Uart_SendLine("$ERR,msg=GET,unknown\r\n");
        }
        return;
    }

    if (strcmp(cmd, "$CFG") == 0) {
        arg1 = strtok(NULL, ",");
        arg2 = strtok(NULL, ",");
        if ((arg1 == NULL) || (arg2 == NULL)) {
            Debug_Uart_SendLine("$ERR,msg=CFG,missing_param\r\n");
            return;
        }

        if (strcmp(arg1, "STREAM") == 0) {
            if (Debug_Uart_ParseInt(arg2, &value_i32) != 0U) {
                s_stream_enable = (value_i32 != 0) ? 1U : 0U;
                Debug_Uart_SendLine("$ACK,msg=CFG,STREAM,ok\r\n");
            } else {
                Debug_Uart_SendLine("$ERR,msg=CFG,STREAM,invalid_value\r\n");
            }
            return;
        }

        if (strcmp(arg1, "PERIOD") == 0) {
            if (Debug_Uart_ParseInt(arg2, &value_i32) != 0U) {
                if (value_i32 < (int32_t)DEBUG_UART_STREAM_PERIOD_MIN) {
                    value_i32 = (int32_t)DEBUG_UART_STREAM_PERIOD_MIN;
                } else if (value_i32 > (int32_t)DEBUG_UART_STREAM_PERIOD_MAX) {
                    value_i32 = (int32_t)DEBUG_UART_STREAM_PERIOD_MAX;
                }
                s_stream_period_ms = (uint16_t)value_i32;
                Debug_Uart_SendLine("$ACK,msg=CFG,PERIOD,ok\r\n");
            } else {
                Debug_Uart_SendLine("$ERR,msg=CFG,PERIOD,invalid_value\r\n");
            }
            return;
        }

        Debug_Uart_SendLine("$ERR,msg=CFG,unknown_field\r\n");
        return;
    }

    if (strcmp(cmd, "$SET") == 0) {
        arg1 = strtok(NULL, ",");
        arg2 = strtok(NULL, ",");
        arg3 = strtok(NULL, ",");
        arg4 = strtok(NULL, ",");
        arg5 = strtok(NULL, ",");
        arg6 = strtok(NULL, ",");
        arg7 = strtok(NULL, ",");

        if ((arg1 != NULL) && (strcmp(arg1, "SPD") == 0)) {
            Debug_Uart_HandleSetSpeed(arg2, arg3, arg4, arg5, arg6, arg7);
            return;
        }
        if ((arg1 != NULL) && (strcmp(arg1, "CIR") == 0)) {
            Debug_Uart_HandleSetCircle(arg2, arg3, arg4, arg5);
            return;
        }

        Debug_Uart_SendLine("$ERR,msg=SET,unknown_module\r\n");
        return;
    }

    Debug_Uart_SendLine("$ERR,msg=unknown_cmd\r\n");
}

void Debug_Uart_Init(UART_HandleTypeDef *huart_tx)
{
    s_debug_huart = huart_tx;
    s_isr_line_len = 0U;
    s_cmd_ready = 0U;
    s_stream_enable = 1U;
    s_stream_period_ms = DEBUG_UART_STREAM_PERIOD_DEFAULT;
    s_last_stream_tick = HAL_GetTick();
    s_send_once = 0U;
    memset(s_isr_line_buf, 0, sizeof(s_isr_line_buf));
    memset(s_cmd_line_buf, 0, sizeof(s_cmd_line_buf));
}

void Debug_Uart_Parse_Byte(uint8_t byte)
{
    /* 仅接收ASCII控制命令。二进制数据流（如摄像头协议）会被自然过滤。 */
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if ((s_isr_line_len > 0U) && (s_cmd_ready == 0U)) {
            s_isr_line_buf[s_isr_line_len] = '\0';
            memcpy(s_cmd_line_buf, s_isr_line_buf, s_isr_line_len + 1U);
            s_cmd_ready = 1U;
        }
        s_isr_line_len = 0U;
        return;
    }

    /* 每条命令必须以 '$' 开头，减少误判 */
    if ((s_isr_line_len == 0U) && (byte != '$')) {
        return;
    }

    /* 只允许可打印ASCII，其他字符直接丢弃并重置当前命令 */
    if (isprint(byte) == 0) {
        s_isr_line_len = 0U;
        return;
    }

    if (s_isr_line_len < (DEBUG_UART_RX_LINE_MAX - 1U)) {
        s_isr_line_buf[s_isr_line_len++] = (char)byte;
    } else {
        /* 超长命令直接丢弃，等待下一行 */
        s_isr_line_len = 0U;
    }
}

void Debug_Uart_Task(void)
{
    uint32_t now_tick;

    if (s_debug_huart == NULL) {
        return;
    }

    /* 先消费命令，再发状态，保证调参反馈及时 */
    if (s_cmd_ready != 0U) {
        char local_line[DEBUG_UART_RX_LINE_MAX];

        __disable_irq();
        memcpy(local_line, s_cmd_line_buf, sizeof(local_line));
        s_cmd_ready = 0U;
        __enable_irq();

        Debug_Uart_ProcessCommand(local_line);
    }

    now_tick = HAL_GetTick();
    if ((s_send_once != 0U) ||
        ((s_stream_enable != 0U) && ((now_tick - s_last_stream_tick) >= s_stream_period_ms))) {
        Debug_Uart_SendSnapshot();
        s_last_stream_tick = now_tick;
        s_send_once = 0U;
    }
}
