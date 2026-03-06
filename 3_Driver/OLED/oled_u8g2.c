/*
 * oled_u8g2.c
 * 作用：
 * 1. 用 U8G2 适配旧版 oled.h 接口，尽量不改上层菜单和业务代码。
 * 2. 所有绘制先写入 U8G2 缓冲区，调用 OLED_Refresh() 后才真正刷新到屏幕。
 *
 * 坐标约定：
 * 1. 点、线、框等图形接口使用像素坐标，Y 范围 0~63。
 * 2. 文本接口沿用旧工程页坐标，y 表示页号 0~7，每页 8 像素。
 * 3. 位图接口的 y0/y1 同样按页坐标处理，不是像素坐标。
 *
 * 兼容性约定：
 * 1. 旧驱动里部分底层接口在 U8G2 模式下已无实际意义。
 * 2. 这些接口保留为空实现，只为兼容旧代码，不建议新代码继续依赖。
 */
#include "oled.h"
#include "oled_ui_mode.h"
#include "u8g2/u8g2.h"
#include <stdio.h>

#if OLED_UI_MODE == OLED_UI_MODE_U8G2

u8g2_t u8g2;

/* 防止重复初始化，避免显示状态被重复配置打断。 */
static u8 s_oled_inited = 0;

/*
 * U8G2 底层适配
 * 负责把 U8G2 的延时、GPIO、软件 I2C 时序请求转接到当前 STM32 工程。
 * 上层不直接调用，仅供 U8G2 初始化时注册。
 */
static uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;

    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        return 1;
    case U8X8_MSG_DELAY_MILLI:
        HAL_Delay(arg_int);
        return 1;
    case U8X8_MSG_DELAY_10MICRO:
        while (arg_int--)
        {
            delay_us(10);
        }
        return 1;
    case U8X8_MSG_DELAY_100NANO:
    case U8X8_MSG_DELAY_NANO:
    case U8X8_MSG_DELAY_I2C:
        return 1;
    case U8X8_MSG_GPIO_I2C_CLOCK:
        if (arg_int)
        {
            OLED_SCLK_Set();
        }
        else
        {
            OLED_SCLK_Clr();
        }
        return 1;
    case U8X8_MSG_GPIO_I2C_DATA:
        if (arg_int)
        {
            OLED_SDIN_Set();
        }
        else
        {
            OLED_SDIN_Clr();
        }
        return 1;
    default:
        return 0;
    }
}

/*
 * 初始化 OLED 和 U8G2 运行环境。
 * 只初始化一次：配置屏幕型号、软件 I2C 回调、清空缓冲区并首次上屏。
 */
void OLED_Init(void)
{
    if (s_oled_inited)
    {
        /* 避免重复初始化。 */
        return;
    }

    /* 使用 SSD1306 128x64 全缓冲软件 I2C 配置，适合菜单类整屏绘制。 */
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_stm32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    s_oled_inited = 1;
}

static const uint8_t *oled_pick_font(u8 char_size)
{
    /*
     * 根据旧接口的字号参数选择 U8G2 字体。
     * 这里是兼容映射，不追求字号完全等比，只保证界面基本可用。
     */
    switch (char_size)
    {
    case 16:
        return u8g2_font_9x15_tr;
    case 24:
        return u8g2_font_10x20_tr;
    case 12:
    default:
        return u8g2_font_6x10_tr;
    }
}

static u8g2_uint_t oled_page_to_pixel_y(u8 page_y)
{
    /*
     * 把旧 OLED 接口里的页坐标转换成 U8G2 需要的像素 Y 坐标。
     * 旧接口 1 页 = 8 像素，因此直接乘 8。
     */
    return (u8g2_uint_t)(page_y * 8U);
}


/* 打开 OLED 显示输出；不会重绘缓冲区，只是退出省电模式。 */
void OLED_Display_On(void)
{
    if (s_oled_inited)
    {
        u8g2_SetPowerSave(&u8g2, 0);
    }
}

/* 关闭 OLED 显示输出；显存内容仍保留在 U8G2 缓冲区中。 */
void OLED_Display_Off(void)
{
    if (s_oled_inited)
    {
        u8g2_SetPowerSave(&u8g2, 1);
    }
}



/*
 * 清空当前显示缓冲区。
 * 注意：这里只清 RAM 缓冲，不会立刻更新到屏幕，仍需调用 OLED_Refresh()。
 */
void OLED_Clear(unsigned dat)
{
    /* dat 是旧接口遗留参数，当前统一清空整块显示缓冲区。 */
    (void)dat;
    u8g2_ClearBuffer(&u8g2);
}

/*
 * 在指定像素位置画点或擦点。
 * 参数：x/y 为像素坐标，t=1 表示点亮，t=0 表示清除。
 */
void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
    /* t=1 画点，t=0 擦点；擦点后恢复默认前景色。 */
    if (t)
    {
        u8g2_DrawPixel(&u8g2, x, y);
    }
    else
    {
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawPixel(&u8g2, x, y);
        u8g2_SetDrawColor(&u8g2, 1);
    }
}

/*
 * 填充一个矩形区域。
 * 参数坐标均为像素坐标，支持传入左上/右下颠倒的坐标。
 * dot=1 表示填充亮点，dot=0 表示整块清空。
 */
void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot)
{
    u8 tmp;

    /* 允许传入逆序坐标，这里统一归一化。 */
    if (x1 > x2)
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;
    }
    if (y1 > y2)
    {
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    /* dot=1 表示填充亮点，dot=0 表示清空区域。 */
    u8g2_SetDrawColor(&u8g2, dot ? 1 : 0);
    u8g2_DrawBox(&u8g2, x1, y1, (u8g2_uint_t)(x2 - x1 + 1U), (u8g2_uint_t)(y2 - y1 + 1U));
    u8g2_SetDrawColor(&u8g2, 1);
}

/*
 * 在指定位置显示单个 ASCII 字符。
 * x 为像素坐标，y 为页坐标，Char_Size 为旧接口字号参数。
 * mode 为兼容保留参数，当前实现未使用。
 */
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 Char_Size, u8 mode)
{
    char str[2] = {(char)chr, '\0'};

    /* y 是页号；mode 在当前兼容层未使用，仅保留旧接口形态。 */
    (void)mode;
    u8g2_SetFontPosTop(&u8g2);
    u8g2_DrawStr(&u8g2, x, oled_page_to_pixel_y(y), str);
}

/*
 * 在指定位置显示无符号数字。
 * len 控制最小显示宽度，不足时左侧补 0。
 */
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size)
{
    char buf[20];

    /* 保持旧行为：按 len 左补 0，再复用字符串显示。 */
    (void)snprintf(buf, sizeof(buf), "%0*u", (int)len, (unsigned)num);
    OLED_ShowString(x, y, buf, size);
}

/*
 * 在指定位置显示字符串。
 * x 为像素坐标，y 为页坐标；字符串按当前映射字体顶部对齐绘制。
 */
void OLED_ShowString(u8 x, u8 y, const char *p, u8 Char_Size)
{
    /* 文本统一按顶部对齐，便于页坐标换算后稳定布局。 */
    u8g2_SetFont(&u8g2, oled_pick_font(Char_Size));
    u8g2_SetFontPosTop(&u8g2);
    u8g2_DrawStr(&u8g2, x, oled_page_to_pixel_y(y), p);
}

/*
 * 在指定位置以反显方式显示字符串。
 * 实现方式是先画实心底框，再用反色绘制文字，常用于菜单选中态。
 */
void OLED_ShowString_Reverse(u8 x, u8 y, const char *p, u8 Char_Size)
{
    u8g2_uint_t y_px;
    u8g2_uint_t width;
    u8g2_int_t ascent;
    u8g2_int_t descent;
    u8g2_uint_t height;

    /* 先画底框，再反色绘字，最后恢复默认颜色。 */
    u8g2_SetFont(&u8g2, oled_pick_font(Char_Size));
    u8g2_SetFontPosTop(&u8g2);

    y_px = oled_page_to_pixel_y(y);
    width = u8g2_GetStrWidth(&u8g2, p);
    ascent = u8g2_GetAscent(&u8g2);
    descent = u8g2_GetDescent(&u8g2);
    height = (u8g2_uint_t)(ascent - descent);
    if (height == 0)
    {
        /* 保底高度，避免异常字体信息导致反显背景不可见。 */
        height = 8;
    }

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, x, y_px, width, height);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, x, y_px, p);
    u8g2_SetDrawColor(&u8g2, 1);
}

/*
 * 旧版设置显存写入位置接口。
 * U8G2 采用图形 API 绘制，不再使用该写指针模型，这里为空实现。
 */
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    /* U8G2 不暴露旧式写指针位置接口，这里仅做兼容保留。 */
    (void)x;
    (void)y;
}

/*
 * 旧版中文字库索引显示接口。
 * 当前 U8G2 兼容层未接入对应中文字库，因此保留为空实现。
 */
void OLED_ShowCHinese(u8 x, u8 y, u8 no)
{
    /* 旧中文字库索引接口未迁移到 U8G2 字体体系，当前为空实现。 */
    (void)x;
    (void)y;
    (void)no;
}

/*
 * 在指定区域绘制单色位图。
 * x0/x1 为像素列范围，y0/y1 为页范围，BMP 数据按 XBMP 格式解释。
 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    u8g2_uint_t width;
    u8g2_uint_t height;

    /* x 按像素，y 按页；高度需乘 8 转成像素。 */
    if (x1 <= x0 || y1 <= y0)
    {
        return;
    }

    width = (u8g2_uint_t)(x1 - x0);
    height = (u8g2_uint_t)((y1 - y0) * 8U);
    u8g2_DrawXBMP(&u8g2, x0, oled_page_to_pixel_y(y0), width, height, BMP);
}

/*
 * 把当前 U8G2 缓冲区内容刷新到 OLED 屏幕。
 * 所有绘图、文本、清屏操作完成后，通常都需要调用一次。
 */
void OLED_Refresh(void)
{
    /* U8G2 是缓冲绘制模型，只有这里才真正发到 OLED。 */
    u8g2_SendBuffer(&u8g2);
}


/*
 * 绘制一条直线。
 * 所有坐标均为像素坐标，color=1 画线，color=0 擦线。
 */
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2, u8 color)
{
    /* 图形接口全部使用像素坐标。 */
    u8g2_SetDrawColor(&u8g2, color ? 1 : 0);
    u8g2_DrawLine(&u8g2, x1, y1, x2, y2);
    u8g2_SetDrawColor(&u8g2, 1);
}

/*
 * 绘制空心矩形边框。
 * 所有坐标均为像素坐标，支持传入逆序坐标。
 */
void OLED_DrawRectangle(u8 x1, u8 y1, u8 x2, u8 y2, u8 color)
{
    u8 tmp;

    /* 先统一坐标顺序，降低上层调用约束。 */
    if (x1 > x2)
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;
    }
    if (y1 > y2)
    {
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }

    /* 画空心矩形边框，不填充内部区域。 */
    u8g2_SetDrawColor(&u8g2, color ? 1 : 0);
    u8g2_DrawFrame(&u8g2, x1, y1, (u8g2_uint_t)(x2 - x1 + 1U), (u8g2_uint_t)(y2 - y1 + 1U));
    u8g2_SetDrawColor(&u8g2, 1);
}

/*
 * 绘制实心矩形。
 * 本质上是对 OLED_Fill() 的语义化封装，便于上层按功能调用。
 */
void OLED_DrawFillRectangle(u8 x1, u8 y1, u8 x2, u8 y2, u8 color)
{
    /* 复用填充接口，保持颜色与坐标语义一致。 */
    OLED_Fill(x1, y1, x2, y2, color);
}

#endif /* ENABLE_OLED_U8G2_COMPAT */
