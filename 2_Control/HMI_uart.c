#include "HMI_uart.h"
#include <string.h>

// 串口屏上位机的通信程序

HMI_Send_Data_t HMI_SendData;
HMI_Recv_Data_t HMI_RecvData;

// 协议解析状态机枚举
typedef enum {
    STATE_HEAD1 = 0,
    STATE_HEAD2,
    STATE_CMD,
    STATE_LEN,
    STATE_DATA,
    STATE_CHECKSUM,
    STATE_TAIL1,
    STATE_TAIL2
} HMI_ParseState_t;

static HMI_ParseState_t rx_state = STATE_HEAD1;
static uint8_t rx_cmd = 0;
static uint8_t rx_len = 0;
static uint8_t rx_data[128];
static uint8_t rx_data_cnt = 0;
static uint8_t rx_checksum = 0;

// 初始化
void HMI_Init(void) {
    memset(&HMI_SendData, 0, sizeof(HMI_SendData));
    memset(&HMI_RecvData, 0, sizeof(HMI_RecvData));
    HMI_SendData.heart = 1;
    HMI_RecvData.stop_flag = 1; // 默认不停止 (1=正常)
}
/*=================发送函数 上报信息给中控屏=======================*/

// 发送整数变量给串口屏（基于淘晶驰屏协议格式，结尾\xff\xff\xff）
void HMI_Send_Var(UART_HandleTypeDef *huart, const char *varName, int value) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%s=%d\xff\xff\xff", varName, value);
    if (len > 0 && huart != NULL) {
        HAL_UART_Transmit(huart, (uint8_t *)buf, len, 100);
    }
}

// 发送字符串变量给串口屏（基于淘晶驰屏协议格式，结尾\xff\xff\xff）
void HMI_Send_Str(UART_HandleTypeDef *huart, const char *compName, const char *str) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "%s=\"%s\"\xff\xff\xff", compName, str);
    if (len > 0 && huart != NULL) {
        HAL_UART_Transmit(huart, (uint8_t *)buf, len, 100);
    }
}

// 定期同步所有参数给屏幕
void HMI_Sync_To_Screen(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;

    HMI_Send_Var(huart, "heart", HMI_SendData.heart);
    HMI_Send_Var(huart, "bat",   HMI_SendData.bat);
    HMI_Send_Var(huart, "sta",   HMI_SendData.sta);
    HMI_Send_Var(huart, "speed", HMI_SendData.speed);
    HMI_Send_Var(huart, "u1",    HMI_SendData.u1);
    HMI_Send_Var(huart, "u2",    HMI_SendData.u2);
    HMI_Send_Var(huart, "u3",    HMI_SendData.u3);
    HMI_Send_Var(huart, "u4",    HMI_SendData.u4);
    HMI_Send_Var(huart, "u5",    HMI_SendData.u5);
    HMI_Send_Var(huart, "vol",   HMI_SendData.vol);
    HMI_Send_Var(huart, "km",    HMI_SendData.km);
    HMI_Send_Var(huart, "G4",    HMI_SendData.G4);
    HMI_Send_Var(huart, "led",   HMI_SendData.led);
    HMI_Send_Var(huart, "bell",  HMI_SendData.bell);
}


/*======================接收函数 接收中控屏的控制======================*/

// 处理完整的一帧数据
void HMI_Process_Frame(uint8_t cmd, uint8_t *data, uint8_t len) {
    // 收到合法数据帧，更新在线状态和心跳时间戳
    HMI_RecvData.is_online = 1;
    HMI_RecvData.last_tick = HAL_GetTick();

    switch (cmd) {
        case HMI_CMD_DRIVE_STATE: // 0x01 行驶状态控制 / 心跳
            if (len >= 1) {
                if (data[0] == 0) {
                    HMI_RecvData.stop_flag = 0; // 0=强制停止
                    HMI_RecvData.func_cmd |= 0x01; // func_cmd的bit0同步置1表示急停
                } else {
                    HMI_RecvData.stop_flag = 1; // 解除停止
                    HMI_RecvData.func_cmd &= ~0x01;
                }
            }
            break;

        case HMI_CMD_HORN: // 0xF1 喇叭控制
            if (len >= 1) {
                HMI_RecvData.horn_req = data[0];
                if (data[0]) HMI_RecvData.func_cmd |= 0x04; // bit2置1
                else HMI_RecvData.func_cmd &= ~0x04;
            }
            break;

        case HMI_CMD_LIGHT: // 0xF2 警示灯控制
            if (len >= 1) {
                HMI_RecvData.light_req = data[0];
                if (data[0]) HMI_RecvData.func_cmd |= 0x02; // bit1置1
                else HMI_RecvData.func_cmd &= ~0x02;
            }
            break;

        case HMI_CMD_CONTROL: // 0x02 综合控制指令 (扩展自数据协议.md)
            if (len >= 4) {
                HMI_RecvData.speed_cmd = (int8_t)data[0];
                HMI_RecvData.steer_cmd = (int8_t)data[1];
                HMI_RecvData.func_cmd  = data[2];
                HMI_RecvData.mode_cmd  = data[3];

                // 同步急停标志 (func_cmd bit0=1为急停)
                if (HMI_RecvData.func_cmd & 0x01) {
                    HMI_RecvData.stop_flag = 0;
                } else {
                    HMI_RecvData.stop_flag = 1;
                }
            }
            break;

        default:
            break;
    }
}

// 状态机解析单字节
void HMI_Parse_Byte(uint8_t byte) {
    switch (rx_state) {
        case STATE_HEAD1:
            if (byte == HMI_FRAME_HEAD1) {
                rx_state = STATE_HEAD2;
                rx_checksum = 0;
            }
            break;

        case STATE_HEAD2:
            if (byte == HMI_FRAME_HEAD2) {
                rx_state = STATE_CMD;
            } else {
                rx_state = STATE_HEAD1;
            }
            break;

        case STATE_CMD:
            rx_cmd = byte;
            rx_checksum += byte;
            rx_state = STATE_LEN;
            break;

        case STATE_LEN:
            rx_len = byte;
            rx_checksum += byte;
            rx_data_cnt = 0;
            if (rx_len > 0) {
                rx_state = STATE_DATA;
            } else {
                rx_state = STATE_CHECKSUM;
            }
            break;

        case STATE_DATA:
            rx_data[rx_data_cnt++] = byte;
            rx_checksum += byte;
            if (rx_data_cnt >= rx_len) {
                rx_state = STATE_CHECKSUM;
            }
            break;

        case STATE_CHECKSUM:
            // 此处可添加校验逻辑：if (byte != rx_checksum) { rx_state = STATE_HEAD1; break; }
            rx_state = STATE_TAIL1;
            break;

        case STATE_TAIL1:
            if (byte == HMI_FRAME_TAIL1) {
                rx_state = STATE_TAIL2;
            } else {
                rx_state = STATE_HEAD1;
            }
            break;

        case STATE_TAIL2:
            if (byte == HMI_FRAME_TAIL2) {
                // 成功收到完整的一帧，进入业务处理
                HMI_Process_Frame(rx_cmd, rx_data, rx_len);
            }
            rx_state = STATE_HEAD1;
            break;

        default:
            rx_state = STATE_HEAD1;
            break;
    }
}

// 批量解析缓冲
void HMI_Parse_Buffer(uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        HMI_Parse_Byte(buf[i]);
    }
}
