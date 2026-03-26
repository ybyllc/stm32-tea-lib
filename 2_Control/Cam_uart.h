#ifndef _CAM_UART_H
#define _CAM_UART_H

#include "common.h"

/*
 * 摄像头通信协议（固定12字节）：
 * AA 55 21 CLASS XH XL YH YL FLAG CHECK 0D 0A
 *
 * CHECK 校验和计算方式：
 * CHECK = (CMD + CLASS + XH + XL + YH + YL + FLAG) & 0xFF
 */
#define CAM_FRAME_HEAD1          0xAA
#define CAM_FRAME_HEAD2          0x55
#define CAM_FRAME_TAIL1          0x0D
#define CAM_FRAME_TAIL2          0x0A
#define CAM_CMD_OBJECT_COORD     0x21
#define CAM_FRAME_LEN            12U

/* FLAG */
#define CAM_FLAG_NONE            0U  /* 未识别 */
#define CAM_FLAG_TRACKING        1U  /* 已识别 */
#define CAM_FLAG_PREDICTING      2U  /* 丢失后预测 */

/* CLASS 识别物品 */
#define CAM_CLASS_NONE           0U
#define CAM_CLASS_BOTTLE         1U  // 水瓶
#define CAM_CLASS_GRASS          2U  // 草
#define CAM_CLASS_OTHER          3U  // 其他

/* 摄像头解析后的共享数据区 */
typedef struct {
    uint8_t  is_online;       /* 在线状态：1在线，0离线 */
    uint8_t  has_new;         /* 新数据标志：1有新帧未取走，0无 */

    uint8_t  cmd;             /* 命令字，当前固定 0x21 */
    uint8_t  class_id;        /* 目标类别ID */
    int16_t  x;               /* X偏差，范围约 -1000 ~ 1000 */
    int16_t  y;               /* Y偏差，范围约 -1000 ~ 1000 */
    uint8_t  flag;            /* 状态位：0/1/2 */

    uint32_t last_tick;       /* 最近一次收到有效帧的系统tick */
    uint32_t frame_ok;        /* 有效帧计数 */
    uint32_t frame_err;       /* 无效帧计数（总） */
    uint32_t checksum_err;    /* 校验错误计数 */
    uint32_t format_err;      /* 帧格式/状态机错误计数 */
} Cam_Recv_Data_t;

extern Cam_Recv_Data_t Cam_RecvData;

/* ==================== 基础接口 ==================== */

/* 初始化模块（上电后调用一次） */
void Cam_Uart_Init(void);

/* 软复位模块状态（等价于重新初始化） */
void Cam_Uart_Reset(void);

/* 逐字节喂数据（推荐在串口接收中断回调中调用） */
void Cam_Uart_Parse_Byte(uint8_t byte);

/* 批量喂数据（DMA/环形缓冲场景可用） */
void Cam_Uart_Parse_Buffer(const uint8_t *buf, uint16_t len);

/*
 * 在线超时检查（建议周期调用）
 * timeout_ms：超时时间，超过则将 is_online 置0
 */
void Cam_Uart_Check_Timeout(uint32_t timeout_ms);

/* ==================== 数据读取接口 ==================== */

/* 获取内部数据只读指针（不清除 has_new） */
const Cam_Recv_Data_t *Cam_Uart_Get(void);

/*
 * 读取一份数据快照并返回是否有“新帧”
 * 返回1：本次取到新帧（并清除 has_new）
 * 返回0：没有新帧（out 仍会得到当前快照）
 */
uint8_t Cam_Uart_Fetch(Cam_Recv_Data_t *out);

#endif /* _CAM_UART_H */
