#ifndef _COMMON_H	//
#define _COMMON_H  //避免重复编译，只是自己的取名，只能包含字母和下划线
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "main.h"

static u32 fac_us=0;							//us延时倍乘数
void Delay_us(u32 nus);
#define delay_ms(n) HAL_Delay(n)
#define delay_us(n) Delay_us(n)

// 串口发送
uint8_t Printf_XL(UART_HandleTypeDef *huart,const char *format, ...);// 超长发送
uint8_t printf_DMA(UART_HandleTypeDef *huart,const char *format,...);// DMA发送

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#define BUFFER_SIZE 128

// 串口接收状态
extern u16 USART1_RX_STA;         
extern u16 USART2_RX_STA;         
extern u16 USART3_RX_STA;       
extern u16 USART4_RX_STA;         
extern u16 USART5_RX_STA;   

extern u8  USART1_RX_BUF[BUFFER_SIZE]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8  USART2_RX_BUF[BUFFER_SIZE]; 
extern u8  USART3_RX_BUF[BUFFER_SIZE]; 
extern u8  USART4_RX_BUF[BUFFER_SIZE]; 
extern u8  USART5_RX_BUF[BUFFER_SIZE]; 


// 变量类型
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef signed int  s32;
typedef signed short s16;
typedef signed char  s8;

#endif