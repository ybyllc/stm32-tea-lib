/****************************************************************************************************
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
**************************************************************************************************/

#include "oled.h"
#include "oled_ui_mode.h"
#include "stdlib.h"
#include "oledfont.h"
#include "common.h"

#if OLED_UI_MODE == OLED_UI_MODE_TRADITIONAL

// OLED显存
// 存储格式如下.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127
// OLED显存缓冲区 (128列x64行/8位 = 1024字节)
unsigned char OLED_GRAM[128][8];

/**********************************************
//初始化IIC通信
**********************************************/
/**********************************************
//IIC Start
**********************************************/
static void OLED_IIC_Start()
{

	OLED_SCLK_Set();
	OLED_SDIN_Set();
	OLED_SDIN_Clr();
	OLED_SCLK_Clr();
}
/**********************************************
//IIC Stop
**********************************************/
static void OLED_IIC_Stop()
{
	OLED_SCLK_Set();
	//	OLED_SCLK_Clr();
	OLED_SDIN_Clr();
	OLED_SDIN_Set();
}

static void OLED_IIC_Wait_Ack()
{

	// 从机不需要ack应答
	OLED_SCLK_Set();
	OLED_SCLK_Clr();
}
/**********************************************
// IIC Write byte
**********************************************/
static void OLED_Write_IIC_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	unsigned char m, da;
	da = IIC_Byte;
	OLED_SCLK_Clr();
	for (i = 0; i < 8; i++)
	{
		m = da;
		//	OLED_SCLK_Clr();
		m = m & 0x80;
		if (m == 0x80)
		{
			OLED_SDIN_Set();
		}
		else
			OLED_SDIN_Clr();
		da = da << 1;
		OLED_SCLK_Set();
		OLED_SCLK_Clr();
	}
}
/**********************************************
// IIC Write Command
**********************************************/
static void OLED_Write_IIC_Command(unsigned char IIC_Command)
{
	OLED_IIC_Start();
	OLED_Write_IIC_Byte(IIC_SLAVE_ADDR); // Slave address,SA0=0
	OLED_IIC_Wait_Ack();
	OLED_Write_IIC_Byte(0x00); // write command
	OLED_IIC_Wait_Ack();
	OLED_Write_IIC_Byte(IIC_Command);
	OLED_IIC_Wait_Ack();
	OLED_IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/
static void OLED_Write_IIC_Data(unsigned char IIC_Data)
{
	OLED_IIC_Start();
	OLED_Write_IIC_Byte(IIC_SLAVE_ADDR); // D/C#=0; R/W#=0
	OLED_IIC_Wait_Ack();
	OLED_Write_IIC_Byte(0x40); // write data
	OLED_IIC_Wait_Ack();
	OLED_Write_IIC_Byte(IIC_Data);
	OLED_IIC_Wait_Ack();
	OLED_IIC_Stop();
}
static void OLED_WR_Byte(unsigned dat, unsigned cmd)
{
	if (cmd)
	{

		OLED_Write_IIC_Data(dat);
	}
	else
	{
		OLED_Write_IIC_Command(dat);
	}
}

/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
	unsigned char m, n;
	for (m = 0; m < 8; m++)
	{
		for (n = 0; n < 128; n++)
		{
			OLED_GRAM[n][m] = fill_Data; // 填充显存
		}
	}
	OLED_Refresh(); // 刷新显示
}

/***********************Delay****************************************/
void Delay_50ms(unsigned int Del_50ms)
{
	unsigned int m;
	for (; Del_50ms > 0; Del_50ms--)
		for (m = 6245; m > 0; m--)
			;
}

void Delay_1ms(unsigned int Del_1ms)
{
	unsigned char j;
	while (Del_1ms--)
	{
		for (j = 0; j < 123; j++)
			;
	}
}

// 坐标设置

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xb0 + y, OLED_CMD);
	OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
	OLED_WR_Byte((x & 0x0f), OLED_CMD);
}
// 开启OLED显示
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
	OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
	OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}
// 关闭OLED显示
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
	OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
	OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}
// 清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(unsigned dat)
{
	u8 i, n;
	for (i = 0; i < 8; i++)
	{
		for (n = 0; n < 128; n++)
		{
			OLED_GRAM[n][i] = dat; // 填充显存
		}
	}
	//OLED_Refresh(); // 刷新显示
}

// 在指定位置画点
void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
	u8 pos, bx, temp = 0;
	if (x > 127 || y > 63)
		return;	 // 超出范围
	pos = y / 8; // 将y坐标转换为page，page 0对应顶部，page 7对应底部
	bx = y % 8;
	temp = 1 << bx; // bit0对应点0，bit7对应点7
	if (t)
		OLED_GRAM[x][pos] |= temp; // 设置点
	else
		OLED_GRAM[x][pos] &= ~temp; // 清除点
}

// 在指定区域内填充指定颜色
void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot)
{
	u8 x, y;
	u8 pos, bx, temp;
	if (x1 > x2)
		return;
	if (y1 > y2)
		return;
	for (x = x1; x <= x2; x++)
	{
		for (y = y1; y <= y2; y++)
		{
			pos = y / 8; // 将y坐标转换为page
			bx = y % 8;
			temp = 1 << bx; // bit0对应点0，bit7对应点7
			if (dot)
				OLED_GRAM[x][pos] |= temp; // 设置点
			else
				OLED_GRAM[x][pos] &= ~temp; // 清除点
		}
	}
}
void OLED_On(void)
{
	u8 i, n;
	for (i = 0; i < 8; i++)
	{
		OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址，0~7页
		OLED_WR_Byte(0x00, OLED_CMD);	  // 设置显示位置-列低地址
		OLED_WR_Byte(0x10, OLED_CMD);	  // 设置显示位置-列高地址
		for (n = 0; n < 128; n++)
			OLED_WR_Byte(1, OLED_DATA);
	} // 全屏显示
}
// 在指定位置显示一个字符,包括部分字符
// x:0~127
// y:0~7 (页地址，从上到下)
// mode:0,反白显示;1,正常显示
// size:选择字体 16/12
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 Char_Size, u8 mode)
{
	unsigned char c = 0, i = 0;
	c = chr - ' '; // 得到偏移后的值
	if (x > Max_Column - 1)
	{
		x = 0;
		y = y + 2;
	}
	if (Char_Size == 16)
	{
		for (i = 0; i < 8; i++)
		{
			if(mode == 0) // 反白显示
				OLED_GRAM[x + i][y] = ~F8X16[c * 16 + i];
			else // 正常显示
				OLED_GRAM[x + i][y] = F8X16[c * 16 + i];
		}
		for (i = 0; i < 8; i++)
		{
			if(mode == 0) // 反白显示
				OLED_GRAM[x + i][y + 1] = ~F8X16[c * 16 + i + 8];
			else // 正常显示
				OLED_GRAM[x + i][y + 1] = F8X16[c * 16 + i + 8];
		}
	}
	else if (Char_Size == 12)
	{
		for (i = 0; i < 6; i++)
		{
			if(mode == 0) // 反白显示
				OLED_GRAM[x + i][y] = ~F6x8[c][i];
			else // 正常显示
				OLED_GRAM[x + i][y] = F6x8[c][i];
		}
	}
}
// m^n函数
u32 oled_pow(u8 m, u8 n)
{
	u32 result = 1;
	while (n--)
		result *= m;
	return result;
}
// 显示2个数字
// x,y :起点坐标
// len :数字的位数
// size:字体大小
// mode:模式	0,填充模式;1,叠加模式
// num:数值(0~4294967295);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size2)
{
	u8 t, temp;
	u8 enshow = 0;
	for (t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2, 1); // 正常显示
				continue;
			}
			else
				enshow = 1;
		}
		OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2, 1); // 正常显示
	}
}
// 显示一个字符串放大
void OLED_ShowString(u8 x, u8 y, const char *p, u8 Char_Size)
{
	unsigned char j = 0;
	u8 max_x = 128;  // OLED宽度为128像素
	u8 char_width = (Char_Size == 16) ? 8 : 6;  // 16号字体宽度8像素，12号字体宽度6像素
	
	while (p[j] != '\0')
	{
		// 检查是否超出屏幕边界，超出则停止显示
		if (x + char_width > max_x)
		{
			break;  // 超出屏幕右边界，停止显示
		}
		
		OLED_ShowChar(x, y, p[j], Char_Size, 1); // 默认白底黑字显示
		x += char_width;
		j++;
	}
}

// 反色显示一个字符串放大
void OLED_ShowString_Reverse(u8 x, u8 y, const char *p, u8 Char_Size)
{
	unsigned char j = 0;
	u8 max_x = 128;  // OLED宽度为128像素
	u8 char_width = (Char_Size == 16) ? 8 : 6;  // 16号字体宽度8像素，12号字体宽度6像素
	
	while (p[j] != '\0')
	{
		// 检查是否超出屏幕边界，超出则停止显示
		if (x + char_width > max_x)
		{
			break;  // 超出屏幕右边界，停止显示
		}
		
		OLED_ShowChar(x, y, p[j], Char_Size, 0); // 反色显示
		x += char_width;
		j++;
	}
}
// 显示汉字
void OLED_ShowCHinese(u8 x, u8 y, u8 no)
{
	u8 t, adder = 0;
	for (t = 0; t < 16; t++)
	{
		OLED_GRAM[x + t][y] = Hzk[2 * no][t];
		adder += 1;
	}
	for (t = 0; t < 16; t++)
	{
		OLED_GRAM[x + t][y + 1] = Hzk[2 * no + 1][t];
		adder += 1;
	}
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
	unsigned int j = 0;
	unsigned char x, y;

	for (y = y0; y < y1; y++)
	{
		for (x = x0; x < x1; x++)
		{
			OLED_GRAM[x][y] = BMP[j++];
		}
	}
}

// 刷新缓存到OLED屏幕(每次显示都要调用)
void OLED_Refresh(void)
{
    unsigned char i,n;
    for(i=0;i<8;i++)
    {
        OLED_WR_Byte(0xB0+i,OLED_CMD);    // 设置页地址
        OLED_WR_Byte(0x00,OLED_CMD);      // 设置列低地址
        OLED_WR_Byte(0x10,OLED_CMD);      // 设置列高地址
        for(n=0;n<128;n++)
        {
            OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); // 写入显存数据
        }
    }
}

// 初始化SSD1306
void OLED_Init(void)
{

	// 初始化IIC接口
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOB_CLK_ENABLE(); // 使能B端口时钟
	GPIO_InitStructure.Pin = OLED_SCLK_PIN | OLED_SDIN_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure); // 初始化GPIOB14,15
	HAL_GPIO_WritePin(GPIOB, OLED_SCLK_PIN | OLED_SDIN_PIN, GPIO_PIN_SET);

	HAL_Delay(200);

	OLED_WR_Byte(0xAE, OLED_CMD); // 关闭显示
	OLED_WR_Byte(0x00, OLED_CMD); // 设置列低地址
	OLED_WR_Byte(0x10, OLED_CMD); // 设置列高地址
	OLED_WR_Byte(0x40, OLED_CMD); // 设置起始行地址
	OLED_WR_Byte(0xB0, OLED_CMD); // 设置页地址
	OLED_WR_Byte(0x81, OLED_CMD); // 设置对比度
	OLED_WR_Byte(0xFF, OLED_CMD); // 最大对比度
	OLED_WR_Byte(0xA1, OLED_CMD); // 设置段重映射 (0xA0正常, 0xA1反置)
	OLED_WR_Byte(0xA6, OLED_CMD); // 设置显示模式 (0xA6正常, 0xA7反白)
	OLED_WR_Byte(0xA8, OLED_CMD); // 设置显示行数
	OLED_WR_Byte(0x3F, OLED_CMD); // 64行模式
	OLED_WR_Byte(0xC8, OLED_CMD); // 设置COM扫描方向 (0xC0正常, 0xC8反置)
	OLED_WR_Byte(0xD3, OLED_CMD); // 设置显示偏移
	OLED_WR_Byte(0x00, OLED_CMD); // 无偏移

	OLED_WR_Byte(0xD5, OLED_CMD); // 设置时钟分频
	OLED_WR_Byte(0x80, OLED_CMD); // 默认值

	OLED_WR_Byte(0xD8, OLED_CMD); // 设置区域颜色模式
	OLED_WR_Byte(0x05, OLED_CMD); // 关闭区域颜色模式

	OLED_WR_Byte(0xD9, OLED_CMD); // 设置预充电周期
	OLED_WR_Byte(0xF1, OLED_CMD); // 预充电周期

	OLED_WR_Byte(0xDA, OLED_CMD); // 设置COM引脚配置
	OLED_WR_Byte(0x12, OLED_CMD); // 设置COM引脚配置

	OLED_WR_Byte(0xDB, OLED_CMD); // 设置VCOMH电压
	OLED_WR_Byte(0x30, OLED_CMD); // VCOMH电压等级

	OLED_WR_Byte(0x8D, OLED_CMD); // 设置电荷泵
	OLED_WR_Byte(0x14, OLED_CMD); // 开启电荷泵

	OLED_WR_Byte(0xAF, OLED_CMD); // 开启显示
}

/**
  * @brief  在OLED上画一条线
  * @param  x1: 起点x坐标 (0-127)
  * @param  y1: 起点y坐标 (0-63)
  * @param  x2: 终点x坐标 (0-127)
  * @param  y2: 终点y坐标 (0-63)
  * @param  color: 颜色 (0-黑色, 1-白色)
  * @retval 无
  */
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 color)
{
	int dx, dy, sx, sy, err, e2;
	
	// 确保坐标在有效范围内
	if (x1 > 127) x1 = 127;
	if (x2 > 127) x2 = 127;
	if (y1 > 63) y1 = 63;
	if (y2 > 63) y2 = 63;
	
	// Bresenham直线算法
	dx = (x2 > x1) ? (x2 - x1) : (x1 - x2);
	dy = (y2 > y1) ? (y2 - y1) : (y1 - y2);
	sx = (x1 < x2) ? 1 : -1;
	sy = (y1 < y2) ? 1 : -1;
	err = dx - dy;
	
	while (1)
	{
		OLED_DrawPoint(x1, y1, color);
		
		if (x1 == x2 && y1 == y2)
			break;
		
		e2 = 2 * err;
		if (e2 > -dy)
		{
			err -= dy;
			x1 += sx;
		}
		if (e2 < dx)
		{
			err += dx;
			y1 += sy;
		}
	}
}

/**
  * @brief  在OLED上画一个矩形（空心）
  * @param  x1: 左上角x坐标 (0-127)
  * @param  y1: 左上角y坐标 (0-63)
  * @param  x2: 右下角x坐标 (0-127)
  * @param  y2: 右下角y坐标 (0-63)
  * @param  color: 颜色 (0-黑色, 1-白色)
  * @retval 无
  */
void OLED_DrawRectangle(u8 x1, u8 y1, u8 x2, u8 y2, u8 color)
{
	// 确保坐标在有效范围内
	if (x1 > 127) x1 = 127;
	if (x2 > 127) x2 = 127;
	if (y1 > 63) y1 = 63;
	if (y2 > 63) y2 = 63;
	
	// 确保x1 < x2, y1 < y2
	if (x1 > x2) { u8 temp = x1; x1 = x2; x2 = temp; }
	if (y1 > y2) { u8 temp = y1; y1 = y2; y2 = temp; }
	
	// 画四条边
	OLED_DrawLine(x1, y1, x2, y1, color); // 上边
	OLED_DrawLine(x1, y2, x2, y2, color); // 下边
	OLED_DrawLine(x1, y1, x1, y2, color); // 左边
	OLED_DrawLine(x2, y1, x2, y2, color); // 右边
}

/**
  * @brief  在OLED上画一个填充矩形（实心）
  * @param  x1: 左上角x坐标 (0-127)
  * @param  y1: 左上角y坐标 (0-63)
  * @param  x2: 右下角x坐标 (0-127)
  * @param  y2: 右下角y坐标 (0-63)
  * @param  color: 颜色 (0-黑色, 1-白色)
  * @retval 无
  */
void OLED_DrawFillRectangle(u8 x1, u8 y1, u8 x2, u8 y2, u8 color)
{
	u8 x, y;
	
	// 确保坐标在有效范围内
	if (x1 > 127) x1 = 127;
	if (x2 > 127) x2 = 127;
	if (y1 > 63) y1 = 63;
	if (y2 > 63) y2 = 63;
	
	// 确保x1 < x2, y1 < y2
	if (x1 > x2) { u8 temp = x1; x1 = x2; x2 = temp; }
	if (y1 > y2) { u8 temp = y1; y1 = y2; y2 = temp; }
	
	// 填充矩形区域
	for (y = y1; y <= y2; y++)
	{
		for (x = x1; x <= x2; x++)
		{
			OLED_DrawPoint(x, y, color);
		}
	}
}

#endif
