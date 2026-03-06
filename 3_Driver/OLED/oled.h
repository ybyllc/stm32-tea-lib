//=========================================电源接线================================================//
//      5V  接DC 5V电源
//     GND  接地
//======================================OLED屏数据线接线==========================================//
//本模块数据线类型为IIC
//     SCL  接PB13    // IIC时钟信号
//     SDA  接PB14    // IIC数据信号
//======================================OLED屏数据线接线==========================================//
//本模块数据线类型为IIC，不需要接片选信号线
//=========================================图片显示说明=========================================//
//本模块本身不带图片显示功能
//============================================================================================//

#ifndef __OLED_H
#define __OLED_H
#include "common.h"
#include "stdlib.h"
#include "oled_ui_mode.h"

// 屏幕尺寸
#define X_WIDTH 128
#define Y_WIDTH 64

//-----------------OLED IIC端口定义----------------

#define OLED_SCLK_PIN   GPIO_PIN_15 // SCL 时钟引脚 PB15
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) // CLK
#define OLED_SCLK_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)

#define OLED_SDIN_PIN   GPIO_PIN_14 // SDA 数据引脚 PB14
#define OLED_SDIN_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET) // DIN
#define OLED_SDIN_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

#define OLED_CMD 0  // 写命令
#define OLED_DATA 1 // 写数据

#define IIC_SLAVE_ADDR 0x78 // IIC slave device address // 二进制 1111000

// OLED函数声明
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(unsigned dat);
void OLED_DrawPoint(u8 x, u8 y, u8 t);
void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 Char_Size, u8 mode);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size);
void OLED_ShowString(u8 x, u8 y, const char *p, u8 Char_Size);
void OLED_ShowString_Reverse(u8 x, u8 y, const char *p, u8 Char_Size);// 反色显示字符串
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x, u8 y, u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);
void OLED_Refresh(void); // 刷新缓存到OLED屏幕(每次显示都要调用)

// OLED图形绘制函数
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 color);
void OLED_DrawRectangle(u8 x1, u8 y1, u8 x2, u8 y2, u8 color);
void OLED_DrawFillRectangle(u8 x1, u8 y1, u8 x2, u8 y2, u8 color);


// OLED参数定义
#define OLED_MODE 0
#define SIZE 8
#define XLevelL 0x00
#define XLevelH 0x10
#define Max_Column 128
#define Max_Row 64
#define Brightness 0xFF

#endif
