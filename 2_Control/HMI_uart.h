#ifndef _HMI_UART_H
#define _HMI_UART_H

#include "common.h"

// ==========================================
// 协议帧定义 (基于通用协议.md)
// ==========================================
#define HMI_FRAME_HEAD1 0xAA	//帧头
#define HMI_FRAME_HEAD2 0x55
#define HMI_FRAME_TAIL1 0x0D	//帧尾
#define HMI_FRAME_TAIL2 0x0A

// ==========================================
// 命令字定义
// ==========================================
#define HMI_CMD_DRIVE_STATE 0x01  // 行驶状态 (心跳/强制停止)
#define HMI_CMD_ULTRASONIC  0x11  // 超声波
#define HMI_CMD_HORN        0xF1  // 喇叭
#define HMI_CMD_LIGHT       0xF2  // 警示灯
#define HMI_CMD_CONTROL     0x02  // 综合控制指令(根据数据协议.md补充)

// ==========================================
// 数据结构定义
// ==========================================

// 发送给屏幕的数据结构 (单片机 -> 屏幕)
typedef struct {
    int heart;      // 心跳包 1
    int bat;        // 电池电量 0-100
    int sta;        // 车辆状态 0=静止，1=运动，2=避障
    int speed;      // 车速km 0-30
    int u1, u2, u3, u4, u5; // 超声波测距，单位cm
    int vol;        // 电压，单位0.1v 0-1000
    int km;         // 当次上电里程数，单位km 0-10000
    int G4;         // 4G网络状态 0=无，1=有
    int led;        // 警示灯 0-关，1=开
    int bell;       // 喇叭 0=关，1=开
} HMI_Send_Data_t;

// 接收自屏幕的数据结构 (屏幕 -> 单片机)
typedef struct {
    uint8_t is_online;      // 屏幕在线状态 0=离线，1=在线
    uint32_t last_tick;     // 上次接收数据的时间戳，用于超时判断

    uint8_t stop_flag;      // 急停标志 0=强制停止，1=正常
    int8_t speed_cmd;       // 速度指令 -100~100 (负值后退)
    int8_t steer_cmd;       // 转向指令 -100~100 (负值左转)

    uint8_t func_cmd;       // 功能键：bit0=急停，bit1=灯，bit2=喇叭
    uint8_t mode_cmd;       // 模式切换：0=手动，1=自动，2=避障

    uint8_t horn_req;       // 喇叭请求 0=关，1=开
    uint8_t light_req;      // 警示灯请求 0=关，1=开
} HMI_Recv_Data_t;

extern HMI_Send_Data_t HMI_SendData;
extern HMI_Recv_Data_t HMI_RecvData;

// ==========================================
// 接口函数
// ==========================================

// 初始化
void HMI_Init(void);

// 接收协议解析 (放在串口中断或接收任务中)
void HMI_Parse_Byte(uint8_t byte);
void HMI_Parse_Buffer(uint8_t *buf, uint16_t len);
void HMI_Process_Frame(uint8_t cmd, uint8_t *data, uint8_t len);

// 心跳超时检查 (放在1Hz左右的定时器或任务中)
void HMI_Check_Timeout(void);

// 发送数据给屏幕的底层接口（淘晶驰格式）
void HMI_Send_Var(UART_HandleTypeDef *huart, const char *varName, int value);
void HMI_Send_Str(UART_HandleTypeDef *huart, const char *compName, const char *str);

// 定期同步所有参数给屏幕 (建议1Hz ~ 20Hz调用)
void HMI_Sync_To_Screen(UART_HandleTypeDef *huart);

#endif // _HMI_UART_H
