/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  塔克创新团队
  * @内  容  PS2无线手柄函数文件
  *
  ******************************************************************************
  * @说  明
  *
  *   PS2数据定义
  *   BYTE   DATA   解释
  *   01     idle
  *   02     0x73   手柄工作模式
  *   03     0x5A   Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7
  *   04     data   SLCT  JOYR  JOYL  STRT   UP   RGIHT  DOWN   L
  *   05     data   L2     R2     L1    R1   Y     B     A      X
  *   06     data   右摇杆X  0x00 = 左    0xff = 右
  *   07     data   右摇杆Y  0x00 = 上    0xff = 下
  *   08     data   左摇杆X  0x00 = 左    0xff = 右
  *   09     data   左摇杆Y  0x00 = 上    0xff = 下
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PS2_H
#define __AX_PS2_H

/* Includes ------------------------------------------------------------------*/	 
//#include "stm32f10x.h"
#include "main.h"
#include "common.h"

// PS2引脚定义
#define AX_PS2_PORT        GPIOC
#define AX_PS2_DI_PIN      GPIO_PIN_0    // 输入 DAT
#define AX_PS2_CMD_PIN     GPIO_PIN_1    // 输出 CMD
#define AX_PS2_CS_PIN      GPIO_PIN_2    // 输出 CS
#define AX_PS2_CLK_PIN     GPIO_PIN_3    // 输出 CLK

#define AX_Delayus(n) Delay_us(n)

// 使用HAL库函数操作GPIO
#define AX_DI_READ() HAL_GPIO_ReadPin(AX_PS2_PORT, AX_PS2_DI_PIN)           //输入 DAT
#define AX_CMD_H() HAL_GPIO_WritePin(AX_PS2_PORT, AX_PS2_CMD_PIN, GPIO_PIN_SET)        //命令位高
#define AX_CMD_L() HAL_GPIO_WritePin(AX_PS2_PORT, AX_PS2_CMD_PIN, GPIO_PIN_RESET)        //命令位低
#define AX_CS_H() HAL_GPIO_WritePin(AX_PS2_PORT, AX_PS2_CS_PIN, GPIO_PIN_SET)       //CS拉高
#define AX_CS_L() HAL_GPIO_WritePin(AX_PS2_PORT, AX_PS2_CS_PIN, GPIO_PIN_RESET)       //CS拉低
#define AX_CLK_H() HAL_GPIO_WritePin(AX_PS2_PORT, AX_PS2_CLK_PIN, GPIO_PIN_SET)      //时钟拉高
#define AX_CLK_L() HAL_GPIO_WritePin(AX_PS2_PORT, AX_PS2_CLK_PIN, GPIO_PIN_RESET)      //时钟拉低


//PS2手柄数据结构体	 
typedef struct			 				
{
  uint8_t mode;		    /* 手柄的工作模式 */
  uint8_t btn1;         /* B0:SLCT B1:JR  B0:JL B3:STRT B4:UP B5:R B6:DOWN  B7:L   */
  uint8_t btn2;         /* B0:L2   B1:R2  B2:L1 B3:R1   B4:Y  B5:B B6:A     B7:X */
  uint8_t RJoy_LR;      /* 右摇杆X  0x00 = 左    0xff = 右   */
  uint8_t RJoy_UD;      /* 右摇杆Y  0x00 = 上    0xff = 下   */
  uint8_t LJoy_LR;      /* 左摇杆X  0x00 = 左    0xff = 右   */
  uint8_t LJoy_UD;      /* 左摇杆Y  0x00 = 上    0xff = 下   */
}JOYSTICK_TypeDef;

/*** PS2游戏手柄驱动函数 **********/
void AX_PS2_Init(void);  //PS2初始化
void AX_PS2_ScanKey(JOYSTICK_TypeDef* JoystickStruct);//PS2读取按键和摇杆数据
void AX_PS2_ScanKey_Deadzone(JOYSTICK_TypeDef *JoystickStruct);
void AX_PS2_ScanKeyVibration(JOYSTICK_TypeDef *JoystickStruct, uint8_t motor1, uint8_t motor2);
void AX_PS2_SetInit(void);  //PS2设置初始化（开启模拟模式和震动）
void AX_PS2_Vibration(uint8_t motor1, uint8_t motor2); //PS2震动控制
void AX_PS2_DebugScan(void); //PS2详细调试扫描

#define STICK_DEADZONE 16  // 摇杆死区

#endif 

/******************* (C) 版权 2023 XTARK **************************************/
