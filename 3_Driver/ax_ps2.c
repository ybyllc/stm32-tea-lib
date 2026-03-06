/**                                                    
		   ____                    _____ _______ _____       XTARK@星瞳科技
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   开源控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@星瞳科技  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 官方微信： 星瞳科技，关注公众号，获取最新产品资讯
  *      
  ******************************************************************************
  * @作  者  星瞳科技
  * @文  件  PS2游戏手柄驱动文件
  *
  ******************************************************************************
  * @说  明
  *
  *   PS2数据格式
  *   BYTE   DATA   说明
  *   01     idle
  *   02     0x73   手柄数字模式
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

#include "ax_ps2.h"
#include <stdio.h>
//#include "ax_delay.h"
//#include "ax_sys.h"

const  uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};   //数据命令                          
static uint8_t PS2_data[9] = {0};  //接收到的数据


/**
  * @函  数  PS2初始化
  * @参  数  无
  * @返回值  无
  */
void AX_PS2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 启用GPIOC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // AX_PS2_DI_PIN (DI/DAT) - 输入
    GPIO_InitStruct.Pin = AX_PS2_DI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AX_PS2_PORT, &GPIO_InitStruct);
    
    // AX_PS2_CMD_PIN (DO/CMD), AX_PS2_CS_PIN (CS), AX_PS2_CLK_PIN (CLK) - 输出
    GPIO_InitStruct.Pin = AX_PS2_CMD_PIN | AX_PS2_CS_PIN | AX_PS2_CLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AX_PS2_PORT, &GPIO_InitStruct);
    
    // 初始化默认状态
    AX_CS_H();
    AX_CLK_H();
    AX_CMD_L();
}


/**
  * @函  数  PS2数据读写函数（带调试）
  * @参  数  cmd:要写入的命令
  * @返回值  读取到的数据
  */
static uint8_t PS2_ReadWriteData_Debug(uint8_t cmd, uint8_t *bit_data)
{
	volatile uint8_t res = 0;
	volatile uint8_t ref;
	uint8_t bit_idx = 0;
	
	// CLK初始状态为高
	AX_CLK_H();
	AX_Delayus(2);
	
	//写入命令，同时读取一个字节数据
	for(ref = 0x01; ref > 0x00; ref <<= 1)
	{
		// 设置 CMD 数据
		if(ref&cmd)
			AX_CMD_H();
		else
			AX_CMD_L();
		
		// CLK保持高电平一段时间
		AX_Delayus(2);
		
		// CLK下降沿 - 数据有效
		AX_CLK_L();
		AX_Delayus(8);
		
		// 在 CLK 低电平时读取数据（下降沿后）
		uint8_t bit = AX_DI_READ();
		if(bit)
			res |= ref;
		
		//保存位数据用于调试
		if (bit_idx < 8) {
			bit_data[bit_idx] = bit;
		}
		bit_idx++;
		
		// CLK保持低电平一段时间
		AX_Delayus(6);
		
		// CLK回到高电平
		AX_CLK_H();
	}

	//返回读取到的数据
    return res;	
}

/**
  * @函  数  PS2数据读写函数
  * @参  数  cmd:要写入的命令
  * @返回值  读取到的数据
  */
static uint8_t PS2_ReadWriteData(uint8_t cmd)
{
	uint8_t dummy[8];
	return PS2_ReadWriteData_Debug(cmd, dummy);
}

/**
  * @函  数  PS2读取按键和摇杆数据
  * @参  数  *JoystickStruct 手柄数据结构体
  * @返回值  无
  */
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)
{
	uint8_t i;
	
	//启用手柄
	AX_CS_L();
	AX_Delayus(20);  // CS拉低后等待一段时间，让手柄准备就绪
	
	//读取PS2数据
	for(i=0; i<9; i++)
	{
		PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
	}
	
	//关闭启用
	AX_CS_H();
	AX_Delayus(20);  // CS拉高后等待

	//赋值给结构体
	JoystickStruct->mode = PS2_data[1];
	JoystickStruct->btn1 = ~PS2_data[3];
	JoystickStruct->btn2 = ~PS2_data[4];
	JoystickStruct->RJoy_LR = PS2_data[5];
	JoystickStruct->RJoy_UD = PS2_data[6];
	JoystickStruct->LJoy_LR = PS2_data[7];
	JoystickStruct->LJoy_UD = PS2_data[8];
}

/**
  * @函  数  去除死区
  * @参  数  *JoystickStruct 手柄数据结构体
  * @返回值  无
  */
void AX_PS2_ScanKey_Deadzone(JOYSTICK_TypeDef *JoystickStruct)
{
    AX_PS2_ScanKey(JoystickStruct);

    // 右摇杆X去死区处理 (0-255范围，128为中位)
    if (JoystickStruct->RJoy_LR > 128 + STICK_DEADZONE) {
        JoystickStruct->RJoy_LR = 128 + (JoystickStruct->RJoy_LR - (128 + STICK_DEADZONE)) * 127 / (127 - STICK_DEADZONE);
    } else if (JoystickStruct->RJoy_LR < 128 - STICK_DEADZONE) {
        JoystickStruct->RJoy_LR = (JoystickStruct->RJoy_LR * 128) / (128 - STICK_DEADZONE);
    } else {
        JoystickStruct->RJoy_LR = 128;
    }
    
    // 右摇杆Y去死区处理
    if (JoystickStruct->RJoy_UD > 128 + STICK_DEADZONE) {
        JoystickStruct->RJoy_UD = 128 + (JoystickStruct->RJoy_UD - (128 + STICK_DEADZONE)) * 127 / (127 - STICK_DEADZONE);
    } else if (JoystickStruct->RJoy_UD < 128 - STICK_DEADZONE) {
        JoystickStruct->RJoy_UD = (JoystickStruct->RJoy_UD * 128) / (128 - STICK_DEADZONE);
    } else {
        JoystickStruct->RJoy_UD = 128;
    }
    
    // 左摇杆X去死区处理
    if (JoystickStruct->LJoy_LR > 128 + STICK_DEADZONE) {
        JoystickStruct->LJoy_LR = 128 + (JoystickStruct->LJoy_LR - (128 + STICK_DEADZONE)) * 127 / (127 - STICK_DEADZONE);
    } else if (JoystickStruct->LJoy_LR < 128 - STICK_DEADZONE) {
        JoystickStruct->LJoy_LR = (JoystickStruct->LJoy_LR * 128) / (128 - STICK_DEADZONE);
    } else {
        JoystickStruct->LJoy_LR = 128;
    }
    
    // 左摇杆Y去死区处理
    if (JoystickStruct->LJoy_UD > 128 + STICK_DEADZONE) {
        JoystickStruct->LJoy_UD = 128 + (JoystickStruct->LJoy_UD - (128 + STICK_DEADZONE)) * 127 / (127 - STICK_DEADZONE);
    } else if (JoystickStruct->LJoy_UD < 128 - STICK_DEADZONE) {
        JoystickStruct->LJoy_UD = (JoystickStruct->LJoy_UD * 128) / (128 - STICK_DEADZONE);
    } else {
        JoystickStruct->LJoy_UD = 128;
    }
}

/**
  * @函  数  PS2读取按键和摇杆数据（带震动）
  * @参  数  *JoystickStruct 手柄数据结构体
  *         motor1: 大电机震动强度 (0-255)
  *         motor2: 小电机震动开关 (0或1)
  * @返回值  无
  * @注    在读取数据的同时发送震动命令
  */
void AX_PS2_ScanKeyVibration(JOYSTICK_TypeDef *JoystickStruct, uint8_t motor1, uint8_t motor2)
{
	uint8_t i;
	
	//启用手柄
	AX_CS_L();
	AX_Delayus(20);  // CS拉低后等待一段时间，让手柄准备就绪
	
	//发送命令并读取数据（前3个字节固定，第4、5字节为震动参数）
	PS2_data[0] = PS2_ReadWriteData(0x01);  // 启动命令
	PS2_data[1] = PS2_ReadWriteData(0x42);  // 请求数据
	PS2_data[2] = PS2_ReadWriteData(0x00);  // 空字节
	PS2_data[3] = PS2_ReadWriteData(motor1); // 大电机震动强度
	PS2_data[4] = PS2_ReadWriteData(motor2); // 小电机震动开关
	PS2_data[5] = PS2_ReadWriteData(0x00);  // 空字节
	PS2_data[6] = PS2_ReadWriteData(0x00);  // 空字节
	PS2_data[7] = PS2_ReadWriteData(0x00);  // 空字节
	PS2_data[8] = PS2_ReadWriteData(0x00);  // 空字节
	
	//关闭启用
	AX_CS_H();
	AX_Delayus(20);  // CS拉高后等待

	//赋值给结构体
	JoystickStruct->mode = PS2_data[1];
	JoystickStruct->btn1 = ~PS2_data[3];
	JoystickStruct->btn2 = ~PS2_data[4];
	JoystickStruct->RJoy_LR = PS2_data[5];
	JoystickStruct->RJoy_UD = PS2_data[6];
	JoystickStruct->LJoy_LR = PS2_data[7];
	JoystickStruct->LJoy_UD = PS2_data[8];
}

/**
  * @函  数  PS2详细调试扫描
  * @参  数  无
  * @返回值  无
  * @注    输出详细的位级调试信息到串口
  */
void AX_PS2_DebugScan(void)
{
	uint8_t i, j;
	uint8_t bit_data[8];
	uint8_t recv_data[9];
	
	printf("\r\n[PS2 Debug] Starting detailed scan...\r\n");
	
	//检查CS前的引脚状态
	printf("[PS2 Debug] Before CS_L: DAT=%d CMD=%d CS=%d CLK=%d\r\n",
		   HAL_GPIO_ReadPin(AX_PS2_PORT, AX_PS2_DI_PIN),
		   HAL_GPIO_ReadPin(AX_PS2_PORT, AX_PS2_CMD_PIN),
		   HAL_GPIO_ReadPin(AX_PS2_PORT, AX_PS2_CS_PIN),
		   HAL_GPIO_ReadPin(AX_PS2_PORT, AX_PS2_CLK_PIN));
	
	//启用手柄
	AX_CS_L();
	AX_Delayus(10);
	
	printf("[PS2 Debug] After CS_L: DAT=%d\r\n", HAL_GPIO_ReadPin(AX_PS2_PORT, AX_PS2_DI_PIN));
	
	//读取PS2数据（带调试）
	for(i=0; i<9; i++)
	{
		recv_data[i] = PS2_ReadWriteData_Debug(PS2_cmnd[i], bit_data);
		
		//输出每个字节的位数据
		printf("[PS2 Debug] Byte %d: Cmd=0x%02X Recv=0x%02X Bits=", i, PS2_cmnd[i], recv_data[i]);
		for(j=0; j<8; j++) {
			printf("%d", bit_data[j]);
		}
		printf("\r\n");
	}
	
	//关闭启用
	AX_CS_H();
	
	printf("[PS2 Debug] After CS_H: DAT=%d\r\n", HAL_GPIO_ReadPin(AX_PS2_PORT, AX_PS2_DI_PIN));
	
	//分析结果
	printf("[PS2 Debug] Analysis:\r\n");
	printf("  Header: 0x%02X (should be 0xFF)\r\n", recv_data[0]);
	printf("  Mode: 0x%02X (0x41=Digital, 0x73=Analog)\r\n", recv_data[1]);
	printf("  Data[2]: 0x%02X\r\n", recv_data[2]);
	printf("  Btn1: 0x%02X\r\n", recv_data[3]);
	printf("  Btn2: 0x%02X\r\n", recv_data[4]);
	
	if (recv_data[0] != 0xFF) {
		printf("[PS2 Debug] ERROR: Header byte should be 0xFF!\r\n");
	}
	if (recv_data[1] != 0x41 && recv_data[1] != 0x73) {
		printf("[PS2 Debug] ERROR: Invalid mode byte!\r\n");
	}
}

/**
  * @函  数  PS2发送短查询
  * @参  数  无
  * @返回值  无
  */
static void AX_PS2_ShortPoll(void)
{
  AX_CS_L();
  AX_Delayus(10);
  PS2_ReadWriteData(0x01);
  PS2_ReadWriteData(0x42);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0x00);
  PS2_ReadWriteData(0x00);
  AX_CS_H();
  AX_Delayus(10);
}

/**
  * @函  数  进入配置模式
  * @参  数  无
  * @返回值  无
  */
static void AX_PS2_EnterConfing(void)
{
  AX_CS_L();
  AX_Delayus(10);
  PS2_ReadWriteData(0x01);
  PS2_ReadWriteData(0x43);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0x01);
  PS2_ReadWriteData(0x00);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0X00);
  AX_CS_H();
  AX_Delayus(10);
}

/**
  * @函  数  开启模拟模式
  * @参  数  无
  * @返回值  无
  */
static void AX_PS2_TurnOnAnalogMode(void)
{
  AX_CS_L();
  PS2_ReadWriteData(0x01);
  PS2_ReadWriteData(0x44);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0x01); //analog=0x01;digital=0x00 设为模拟模式
  PS2_ReadWriteData(0xEE); //Ox03 锁定设置，退出后必须重新进入配置才可以更改模式        //0xEE 不锁定设置，退出后无需重新进入配置就可以更改模式
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0X00);
  AX_CS_H();
  AX_Delayus(10);
}

/**
  * @函  数  开启震动
  * @参  数  无
  * @返回值  无
  */
static void AX_PS2_VibrationMode(void)
{
  AX_CS_L();
  AX_Delayus(10);
  PS2_ReadWriteData(0x01);
  PS2_ReadWriteData(0x4D);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0x00);
  PS2_ReadWriteData(0X01);
  AX_CS_H();
  AX_Delayus(10);
}

/**
  * @函  数  退出配置模式
  * @参  数  无
  * @返回值  无
  */
static void AX_PS2_ExitConfing(void)
{
  AX_CS_L();
  AX_Delayus(10);
  PS2_ReadWriteData(0x01);
  PS2_ReadWriteData(0x43);
  PS2_ReadWriteData(0X00);
  PS2_ReadWriteData(0x00);
  PS2_ReadWriteData(0x5A);
  PS2_ReadWriteData(0x5A);
  PS2_ReadWriteData(0x5A);
  PS2_ReadWriteData(0x5A);
  PS2_ReadWriteData(0x5A);
  AX_CS_H();
  AX_Delayus(10);
}

/**
  * @函  数  PS2设置初始化（开启模拟模式和震动）
  * @参  数  无
  * @返回值  无
  */
void AX_PS2_SetInit(void)
{
  AX_PS2_ShortPoll();
  AX_PS2_ShortPoll();
  AX_PS2_ShortPoll();
  AX_PS2_EnterConfing(); // 进入配置模式
  AX_PS2_TurnOnAnalogMode(); // 开启模拟模式
  AX_PS2_VibrationMode(); // 开启震动模式
  AX_PS2_ExitConfing(); // 退出配置模式
}

/**
  * @函  数  PS2震动控制
  * @参  数  motor1: 大电机震动强度 (0-255)
  *         motor2: 小电机震动开关 (0或1)
  * @返回值  无
  */
void AX_PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
   AX_CS_L();
   AX_Delayus(10);
   PS2_ReadWriteData(0x01); // 发送启动命令
   PS2_ReadWriteData(0x42); // 发送请求数据
   PS2_ReadWriteData(0X00);
   PS2_ReadWriteData(motor1);
   PS2_ReadWriteData(motor2);
   PS2_ReadWriteData(0X00);
   PS2_ReadWriteData(0X00);
   PS2_ReadWriteData(0X00);
   PS2_ReadWriteData(0X00);
   AX_CS_H();
   AX_Delayus(10);
}

/******************* (C) 版权 2023 XTARK **************************************/
