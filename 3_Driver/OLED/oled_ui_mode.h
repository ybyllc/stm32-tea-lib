#ifndef OLED_UI_MODE_H
#define OLED_UI_MODE_H

/*
 * OLED 显示模式配置
 * 0: 传统 OLED_GRAM + OLED_* 接口
 * 1: 用U8G2通用方式驱动OLED
 * 1: LVGL 模式
 */
#define OLED_UI_MODE_TRADITIONAL 0 //传统驱动
#define OLED_UI_MODE_U8G2 		 1
#define OLED_UI_MODE_LVGL        2

#ifndef OLED_UI_MODE
#define OLED_UI_MODE OLED_UI_MODE_TRADITIONAL//OLED_UI_MODE_U8G2
#endif

#endif /* OLED_UI_MODE_H */
