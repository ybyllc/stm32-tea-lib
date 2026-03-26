#ifndef _DEBUG_UART_H
#define _DEBUG_UART_H

#include "common.h"

/*
 * ============================================================
 * debug_uart 串口调参说明
 * ============================================================
 *
 * 一、基础规则
 * 1) 字符编码：ASCII
 * 2) 一条指令一行，行尾 '\n'（可带 '\r'）
 * 3) 每条指令必须以 '$' 开头
 * 4) 设备返回：
 *    - $ACK 开头：命令执行成功
 *    - $ERR 开头：命令执行失败
 *    - $ST/$SPD/$CIR/$MOT：状态上报帧
 *
 * 二、状态上报帧（设备 -> AI）
 * 1) $ST：系统+陀螺仪状态
 *    示例：
 *    $ST,t=12345,mode=0,page=8,gyro=1,cam=1,yaw=12.30,gx=0.10,gy=0.20,gz=1.23,ax=0.001,ay=0.002,az=0.998
 *
 * 2) $SPD：速度环核心状态
 *    示例：
 *    $SPD,kp=0.1600,ki=0.0200,kd=0.0000,tar=520.0,ml=500.0,mr=510.0,pl=120,pr=118,en=1
 *
 * 3) $CIR：画圆角度环核心状态
 *    示例：
 *    $CIR,kp=2.8000,ki=0.0300,kd=0.0800,ref=30.00,yaw=28.50,err=1.50,pl=210,pr=230
 *
 * 4) $MOT：四路电机调试值
 *    示例：
 *    $MOT,fl=0,fr=0,bl=120,br=118
 *
 * 三、控制指令（AI -> 设备）
 * 1) 单次拉取状态：
 *    $GET,ONCE
 *
 * 2) 流式开关：
 *    $CFG,STREAM,1      // 开启周期上报
 *    $CFG,STREAM,0      // 关闭周期上报
 *
 * 3) 设置上报周期（ms，内部会限幅到 20~2000）：
 *    $CFG,PERIOD,100
 *
 * 4) 调速度环 PID（SPD）：
 *    $SET,SPD,KP,0.18
 *    $SET,SPD,KI,0.03
 *    $SET,SPD,KD,0.01
 *    $SET,SPD,TAR,600
 *    $SET,SPD,EN,1
 *    $SET,SPD,ALL,0.18,0.03,0.01,600,1
 *
 * 5) 调画圆角度环 PID（CIR）：
 *    $SET,CIR,KP,3.0
 *    $SET,CIR,KI,0.04
 *    $SET,CIR,KD,0.10
 *    $SET,CIR,ALL,3.0,0.04,0.10
 *
 * 6) 查询帮助：
 *    $HELP
 *
 * 四、建议 AI 调参流程
 * 1) 开启流：$CFG,STREAM,1
 * 2) 设周期：$CFG,PERIOD,100
 * 3) 先只调 KP，再调 KI，最后再加 KD
 * 4) 每次改参后先发 $GET,ONCE，确认回读值与误差趋势
 * 5) 若震荡，优先降低 KP 或 KI，再考虑增加 KD
 *
 * 五、切换调试串口
 * - 在 Core/Src/usart.c 文件开头宏 `DEBUG_UART_BIND_USART` 中切换：
 *   1 表示绑定 USART1，2 表示绑定 USART2。
 * - 切换后需确保该串口实际接到你的上位机/AI 侧。
 *
 * ============================================================
 *
 */

void Debug_Uart_Init(UART_HandleTypeDef *huart_tx);
void Debug_Uart_Parse_Byte(uint8_t byte);
void Debug_Uart_Task(void);

#endif /* _DEBUG_UART_H */
