#include "Cam_uart.h"
#include <string.h>

/*
 * Cam_uart.c
 * 作用：
 * 1) 解析摄像头固定协议帧（AA 55 21 ... 0D 0A）
 * 2) 提供在线状态、新数据标志和错误统计
 * 3) 对上层暴露“中断喂字节 + 周期取快照”的最小接口
 */

/* 协议接收状态机 */
typedef enum {
    CAM_RX_WAIT_HEAD1 = 0, /* 等待帧头1: 0xAA */
    CAM_RX_WAIT_HEAD2,      /* 等待帧头2: 0x55 */
    CAM_RX_WAIT_CMD,        /* 等待命令字: 0x21 */
    CAM_RX_WAIT_CLASS,      /* 等待类别 */
    CAM_RX_WAIT_XH,         /* 等待X高字节 */
    CAM_RX_WAIT_XL,         /* 等待X低字节 */
    CAM_RX_WAIT_YH,         /* 等待Y高字节 */
    CAM_RX_WAIT_YL,         /* 等待Y低字节 */
    CAM_RX_WAIT_FLAG,       /* 等待状态FLAG */
    CAM_RX_WAIT_CHECK,      /* 等待CHECK */
    CAM_RX_WAIT_TAIL1,      /* 等待帧尾1: 0x0D */
    CAM_RX_WAIT_TAIL2       /* 等待帧尾2: 0x0A */
} Cam_RxState_t;

/* 状态机内部临时寄存变量（仅解析过程使用） */
static Cam_RxState_t s_rx_state = CAM_RX_WAIT_HEAD1;
static uint8_t s_cmd = 0U;
static uint8_t s_class = 0U;
static uint8_t s_xh = 0U;
static uint8_t s_xl = 0U;
static uint8_t s_yh = 0U;
static uint8_t s_yl = 0U;
static uint8_t s_flag = 0U;
static uint8_t s_checksum_sum = 0U;
static uint8_t s_checksum_recv = 0U;

/* 对外共享的接收结果 */
Cam_Recv_Data_t Cam_RecvData;

/* 重置状态机，不清空统计与上次有效数据 */
static void Cam_Uart_Reset_RxState(void)
{
    s_rx_state = CAM_RX_WAIT_HEAD1;
    s_cmd = 0U;
    s_class = 0U;
    s_xh = 0U;
    s_xl = 0U;
    s_yh = 0U;
    s_yl = 0U;
    s_flag = 0U;
    s_checksum_sum = 0U;
    s_checksum_recv = 0U;
}

/* 只接受协议定义的3种 FLAG，提前拦截异常值 */
static uint8_t Cam_Uart_IsFlagValid(uint8_t flag)
{
    return ((flag == CAM_FLAG_NONE) ||
            (flag == CAM_FLAG_TRACKING) ||
            (flag == CAM_FLAG_PREDICTING)) ? 1U : 0U;
}

/* 一帧完整合法后，将临时解析值写入共享数据区 */
static void Cam_Uart_CommitFrame(void)
{
    /* 摄像头发送的是16位补码，直接转 int16_t 即可还原负数 */
    int16_t x = (int16_t)(((uint16_t)s_xh << 8) | s_xl);
    int16_t y = (int16_t)(((uint16_t)s_yh << 8) | s_yl);

    Cam_RecvData.is_online = 1U;
    Cam_RecvData.has_new = 1U;

    Cam_RecvData.cmd = s_cmd;
    Cam_RecvData.class_id = s_class;
    Cam_RecvData.x = x;
    Cam_RecvData.y = y;
    Cam_RecvData.flag = s_flag;

    Cam_RecvData.last_tick = HAL_GetTick();
    Cam_RecvData.frame_ok++;
}

/* 模块初始化：清数据 + 状态机复位 */
void Cam_Uart_Init(void)
{
    memset(&Cam_RecvData, 0, sizeof(Cam_RecvData));
    Cam_Uart_Reset_RxState();
}

/* 对外复位入口 */
void Cam_Uart_Reset(void)
{
    Cam_Uart_Init();
}

/*
 * 逐字节状态机解析
 * 设计原则：
 * - 发现异常立即丢帧并回到 WAIT_HEAD1
 * - 中断里不做打印，避免阻塞控制链路
 */
void Cam_Uart_Parse_Byte(uint8_t byte)
{
    switch (s_rx_state) {
        case CAM_RX_WAIT_HEAD1:
            if (byte == CAM_FRAME_HEAD1) {
                s_rx_state = CAM_RX_WAIT_HEAD2;
            }
            break;

        case CAM_RX_WAIT_HEAD2:
            if (byte == CAM_FRAME_HEAD2) {
                s_rx_state = CAM_RX_WAIT_CMD;
                s_checksum_sum = 0U; /* 从 CMD 开始累加 */
            } else if (byte == CAM_FRAME_HEAD1) {
                /* 连续收到AA时，允许重同步 */
                s_rx_state = CAM_RX_WAIT_HEAD2;
            } else {
                Cam_Uart_Reset_RxState();
            }
            break;

        case CAM_RX_WAIT_CMD:
            if (byte != CAM_CMD_OBJECT_COORD) {
                /* 当前模块只支持0x21命令，其他命令直接判错丢弃 */
                Cam_RecvData.frame_err++;
                Cam_RecvData.format_err++;
                Cam_Uart_Reset_RxState();
                break;
            }
            s_cmd = byte;
            s_checksum_sum = (uint8_t)(s_checksum_sum + byte);
            s_rx_state = CAM_RX_WAIT_CLASS;
            break;

        case CAM_RX_WAIT_CLASS:
            s_class = byte;
            s_checksum_sum = (uint8_t)(s_checksum_sum + byte);
            s_rx_state = CAM_RX_WAIT_XH;
            break;

        case CAM_RX_WAIT_XH:
            s_xh = byte;
            s_checksum_sum = (uint8_t)(s_checksum_sum + byte);
            s_rx_state = CAM_RX_WAIT_XL;
            break;

        case CAM_RX_WAIT_XL:
            s_xl = byte;
            s_checksum_sum = (uint8_t)(s_checksum_sum + byte);
            s_rx_state = CAM_RX_WAIT_YH;
            break;

        case CAM_RX_WAIT_YH:
            s_yh = byte;
            s_checksum_sum = (uint8_t)(s_checksum_sum + byte);
            s_rx_state = CAM_RX_WAIT_YL;
            break;

        case CAM_RX_WAIT_YL:
            s_yl = byte;
            s_checksum_sum = (uint8_t)(s_checksum_sum + byte);
            s_rx_state = CAM_RX_WAIT_FLAG;
            break;

        case CAM_RX_WAIT_FLAG:
            s_flag = byte;
            s_checksum_sum = (uint8_t)(s_checksum_sum + byte);
            if (Cam_Uart_IsFlagValid(s_flag) == 0U) {
                Cam_RecvData.frame_err++;
                Cam_RecvData.format_err++;
                Cam_Uart_Reset_RxState();
            } else {
                s_rx_state = CAM_RX_WAIT_CHECK;
            }
            break;

        case CAM_RX_WAIT_CHECK:
            s_checksum_recv = byte;
            if (s_checksum_recv != s_checksum_sum) {
                /* CHECK 不一致，丢整帧 */
                Cam_RecvData.frame_err++;
                Cam_RecvData.checksum_err++;
                Cam_Uart_Reset_RxState();
            } else {
                s_rx_state = CAM_RX_WAIT_TAIL1;
            }
            break;

        case CAM_RX_WAIT_TAIL1:
            if (byte == CAM_FRAME_TAIL1) {
                s_rx_state = CAM_RX_WAIT_TAIL2;
            } else {
                Cam_RecvData.frame_err++;
                Cam_RecvData.format_err++;
                Cam_Uart_Reset_RxState();
            }
            break;

        case CAM_RX_WAIT_TAIL2:
            if (byte == CAM_FRAME_TAIL2) {
                Cam_Uart_CommitFrame();
            } else {
                /* 尾字节错误同样按格式错统计 */
                Cam_RecvData.frame_err++;
                Cam_RecvData.format_err++;
            }
            Cam_Uart_Reset_RxState();
            break;

        default:
            Cam_Uart_Reset_RxState();
            break;
    }
}

/* 批量解析接口，适配DMA/环形缓冲读取结果 */
void Cam_Uart_Parse_Buffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == NULL) {
        return;
    }

    for (i = 0U; i < len; i++) {
        Cam_Uart_Parse_Byte(buf[i]);
    }
}

/* 周期性在线检测：超过 timeout_ms 未收到有效帧则离线 */
void Cam_Uart_Check_Timeout(uint32_t timeout_ms)
{
    if ((Cam_RecvData.is_online != 0U) &&
        ((HAL_GetTick() - Cam_RecvData.last_tick) > timeout_ms)) {
        Cam_RecvData.is_online = 0U;
    }
}

/* 返回内部数据只读指针（不改变 has_new） */
const Cam_Recv_Data_t *Cam_Uart_Get(void)
{
    return &Cam_RecvData;
}

/*
 * 获取一份快照并查询“是否有新帧”
 * 返回1：本次确实拿到新帧（并清除 has_new）
 * 返回0：没有新帧（out 仍会得到最近一次快照）
 */
uint8_t Cam_Uart_Fetch(Cam_Recv_Data_t *out)
{
    if (out == NULL) {
        return 0U;
    }

    *out = Cam_RecvData;
    if (Cam_RecvData.has_new != 0U) {
        Cam_RecvData.has_new = 0U;
        return 1U;
    }

    return 0U;
}
