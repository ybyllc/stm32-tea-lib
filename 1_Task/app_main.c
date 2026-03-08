#include "app_main.h"
#include "main.h"          // 包含 HAL 句柄 & 外设句柄

#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "motor-m.h"         // 电机驱动
// #include "pid.h"           // 速度环
// #include "led.h"           // 指示灯
#include "wit_gyro_sdk.h"    // 串口陀螺仪
#include "icm42670.h"         // ICM42670 陀螺仪
#include "gyro_collision.h"   // 陀螺仪碰撞检测
#include "TOF_VL53L0X.h"
#include "oled.h"
#include "ax_ps2.h"
#include "encoder_knob.h"
#include "menu.h"

#include <stdio.h>

JOYSTICK_TypeDef my_joystick;  //手柄键值结构体

// 陀螺仪类型枚举
typedef enum {
    GYRO_TYPE_NONE,        // 无陀螺仪
    GYRO_TYPE_WIT,         // 原来的陀螺仪
    GYRO_TYPE_ICM42670     // ICM42670 陀螺仪
} GyroType;

// 当前使用的陀螺仪类型
GyroType current_gyro_type = GYRO_TYPE_NONE;


int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}


int fgetc(FILE *f)	//接收(阻塞）
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

//static u32 fac_us=0;							//us延时倍乘数
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};
}

/* 全局变量 ------------------------------------------------*/
static uint32_t tick_1ms = 0;   // 1 ms 软计数
float servo_target_angle = 90.0f; // 舵机目标角度
float servo_current_angle = 90.0f; // 舵机当前角度

/* 私有函数声明 --------------------------------------------*/
static void SystemTick_Handler_1ms(void);   // 1 ms 时基

/* 外部变量声明 --------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* 函数体 --------------------------------------------------*/

// 舵机角度控制函数 - 仅设置目标角度
// angle: 0-180度
void Servo_Control(void)
{
    // 1) 目标角限幅
    if (servo_target_angle < 0.0f)   servo_target_angle = 0.0f;
    if (servo_target_angle > 180.0f) servo_target_angle = 180.0f;

    // ===== 参数（你先用这组）=====
    const float dt = 0.001f;              // 控制周期(秒)。若你是100Hz就0.01；200Hz就0.005
    const float deadband = 0.4f;         // 死区(度)：抑制IMU抖动引起来回动
    const float omega_max = 360.0f;       // 最大角速度(度/秒)：越大越快
    const float Kp = 10.0f;               // 角速度增益(1/秒)：配合 omega_max 使用
    const float tau = 0.1f;             // 目标低通时间常数(秒)
    const float alpha = dt / (tau + dt); // 低通系数

    // 2) 目标角低通（强烈建议：让目标更连续）
    static float target_f = 90.0f;       // 初始化到中位（也可初始化为 servo_target_angle）
    //target_f += alpha * (servo_target_angle - target_f);
	target_f = servo_target_angle;
	
    // 3) 误差
    float e = target_f - servo_current_angle;

    // 4) 死区：小误差不动
    if (fabsf(e) < deadband) e = 0.0f;

    // 5) 角速度命令（度/秒）
    float omega = Kp * e;

    // 6) 限速（决定“顺”和“快”的关键）
    if (omega >  omega_max) omega =  omega_max;
    if (omega < -omega_max) omega = -omega_max;

    // 7) 积分得到舵机角度命令（连续变化，不再“0.15°一段段跳”）
    servo_current_angle += omega * dt;

    // 8) 舵机角限幅
    if (servo_current_angle > 180.0f) servo_current_angle = 180.0f;
    if (servo_current_angle < 0.0f)   servo_current_angle = 0.0f;

    // 9) PWM 输出
    uint32_t compare = (uint32_t)(500.0f + servo_current_angle * (2000.0f / 180.0f));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
}


void Servo_Control1()
{
	// 舵机角度限制
    if(servo_target_angle < 0.0f) servo_target_angle = 0.0f;
    if(servo_target_angle > 180.0f) servo_target_angle = 180.0f;
    
    // 舵机控制逻辑
	float deadzone = 30.0f; // 2度死区
	float max_step = 0.3f; // 每次最多移动0.5度
	float angle_diff = servo_target_angle - servo_current_angle;
	
	if(fabs(angle_diff) < deadzone)
	// 检查是否在死区内
	{
		if(angle_diff>20) servo_current_angle += 0.2;
		else if(angle_diff>2) servo_current_angle += 0.15;
		else if(angle_diff>0.4) servo_current_angle += 0.04;
		else if(angle_diff<-20) servo_current_angle -= 0.25;
		else if(angle_diff<-2) servo_current_angle -= 0.15;
		else if(angle_diff<-0.4) servo_current_angle -= 0.04;
	}
	else
	{
		// 限制每次移动的角度
		if(angle_diff > max_step)
		{
			servo_current_angle += max_step;
		}
		else if(angle_diff < -max_step)
		{
			servo_current_angle -= max_step;
		}
		else
		{
			// 差值在max_step范围内，直接移动到目标角度
			servo_current_angle = servo_target_angle;
		}
		
		// 限制角度范围在0-180度
		if(servo_current_angle > 180.0f)
		{
			servo_current_angle = 180.0f;
		}
		else if(servo_current_angle < 0.0f)
		{
			servo_current_angle = 0.0f;
		}
		

	}
	// 计算PWM占空比并设置
	/* 0.5 ms -> 500 counts, 2.5 ms -> 2500 counts */
	uint32_t compare = (uint32_t)(500.0f + servo_current_angle * (2000.0f / 180.0f));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare);
		
    // 仅设置目标角度，实际控制在TIM1_100us_Callback中执行
    
}
/**
 * @brief  代替原来 main() 里的所有初始化
 * @note   CubeMX 外设句柄已经初始化完毕，此函数只干业务层的事
 */
void App_Init(void)
{
    /* 1. 业务模块初始化 */
    // LED_Init();
    // MOTOR_Init();          // 包含 PID 参数
	Motor_Init();

    
	

//	PCout(6);// TOF SHUT引脚上拉启动
    // PID_Init(&g_pidSpeed, 0.1f, 0.5f, 0.0f);

    /* 3. 启动周期定时器（例如 TIM6 1 kHz） */
    //HAL_TIM_Base_Start_IT(&htim6);

    //LED_On(LED_GREEN);
	
//	if(VL53L0X_Init())
//    {
//        printf("VL53L0X not found!\r\n");
//    }
//	printf("VL53L0X ready\r\n");
	
	OLED_Init();		//初始化OLED  
	OLED_Clear(0);
    
    // 显示系统初始化提示信息
    OLED_ShowString(0, 2, "Starting.", 16);
    OLED_Refresh();
	
	// 旋转编码器初始化
	Ec11_knob_Init();
	OLED_ShowString(0, 2, "Starting..", 16);
    OLED_Refresh();

//	// 初始化为低电平
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
//    // 启动舵机PWM输出 (PA6 - TIM3_CH1)
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//    // 初始角度为90度
//    //Servo_SetAngle(90.0f);
//    servo_current_angle = 90.0f; // 确保初始角度一致
//    // 启动TIM1中断，用于1ms时间
    HAL_TIM_Base_Start_IT(&htim1);
//	delay_ms(2000);

    OLED_ShowString(0, 2, "Starting...", 16);
    OLED_Refresh();
//    // TOF 诊断测试
//    printf("\r\n=== 启动 TOF 诊断测试 ===\r\n");
//    TOF_DiagnosticTest();
//    TOF_ReadStatusInfo();
//    printf("=== TOF 诊断测试完成 ===\r\n\r\n");

    // PS2手柄初始化
    //AX_PS2_Init();
   // delay_ms(100);
    //PS2_SetInit();
    
    // 初始化完成，清除屏幕并进入菜单系统
    OLED_Clear(0);
    Menu_Init();
}

/**
 * @brief  超级循环，代替 while(1){}
 */
void App_Loop(void)
{
	while(1)
	{
		// 运行菜单系统
        Menu_Run();
	}
}

void App_Loop2(void)
{
	u16 mm;
	
	u8 t;
	t=' ';
	OLED_Clear(0);
	OLED_ShowString(6,3,"0.96' OLED TEST",16);
	OLED_Clear(0);
	
	char str[20];
	char str1[20];
	
	// 编码器测试 - 定时器1ms扫描版
	Ec11_knob_Clear_Count();
	
	while(1)
	{
		OLED_Clear(0);
		
		// 获取当前计数（由定时器自动扫描更新）
		int32_t current_count = Ec11_knob_Get_Count();
		
		// 显示标题
		OLED_ShowString(0,0,"EC11 Test:",16);
		
		// 显示计数值
		sprintf(str,"Cnt:%4ld", current_count);
		OLED_ShowString(0,2,str,16);
		
		// 按键短按清零
		uint8_t key_evt = Ec11_knob_Key_GetEvent();
		if(key_evt == 1 || key_evt == 2)
		 	Ec11_knob_Clear_Count();
		
		// 显示按键状态
		sprintf(str1,"K:%s",Ec11_knob_Key_GetState() ? "R" : "P");
		OLED_ShowString(100,2,str1,12);
		
		OLED_ShowString(0,4,"Turn me!",12);
		
		OLED_Refresh();
		delay_ms(5);  // 显示刷新50ms，扫描由定时器完成
	}
	while(1)
	{
		OLED_Clear(0);
//		for(mm = 0;mm <= 64;mm++)
//		{
//			OLED_DrawPoint(mm*2,2,1);
//			delay_ms(33);
//			//OLED_Show_Display();
//		}

		sprintf(str,"x:%d,y:%d,z:%d",mm,mm,mm);
		
		Gryo_Update();
    	// printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", fGyro[0], fGyro[1], fGyro[2], fAcc[0], fAcc[1], fAcc[2]);


		//改为竖排
		sprintf(str,"servo:");
		OLED_ShowString(0,0,str,16);
		sprintf(str1,"%.1f",fYaw);
//		OLED_ShowString_Reverse(strlen(str)*8,0,str1,16);
		sprintf(str,"x:%.1f",fAngle[0]);
		OLED_ShowString(0,2,str,16);
		sprintf(str,"y:%.1f",fAngle[1]);
		OLED_ShowString(0,4,str,16);
		sprintf(str,"z:%.1f",fAngle[2]);	
		OLED_ShowString(0,6,str,16);

		//OLED_ShowString(80,0,"acc:",16);	
		sprintf(str," x:%.1f",fAcc[0]*10);
		OLED_ShowString(63,2,str,16);
		sprintf(str," y:%.1f",fAcc[1]*10);
		OLED_ShowString(63,4,str,16);
		sprintf(str," z:%.1f",fAcc[2]*10);
		OLED_ShowString(63,6,str,16);
		
		// 舵机控制 - 每次最多移动0.5度，2度死区
		float target_angle = servo_current_angle + fAngle[1];

		
		servo_target_angle = target_angle;
		
		sprintf(str1,"%.1f",target_angle);
		OLED_ShowString_Reverse(strlen(str)*8,0,str1,16);
		
		// PS2手柄数据处理
//		PS2_ReadData();
//		u8 key = PS2_DataKey();
//		if(key > 0)
//		{
//			printf("PS2 Key pressed: %d\r\n\r\n", key);
//		}

//		AX_PS2_ScanKey(&my_joystick);
//		
//		//打印手柄键值
//		printf("MODE:%2x BTN1:%2x BTN2:%2x RJOY_LR:%2x RJOY_UD:%2x LJOY_LR:%2x LJOY_UD:%2x\r\n",
//		my_joystick.mode, my_joystick.btn1, my_joystick.btn2, 
//		my_joystick.RJoy_LR, my_joystick.RJoy_UD, my_joystick.LJoy_LR, my_joystick.LJoy_UD);	

		
		OLED_Refresh();
		//OLED_Clear(0);
		delay_ms(10);
	}
	while(1) 
	{		
		OLED_Clear(0);
        OLED_ShowString(0,0," hello hsc",16);
		OLED_ShowString(6,3,"0.96' OLED TEST",16);
		OLED_ShowString(0,6,"ASCII:",16);  
		OLED_ShowString(63,6,"CODE:",16);  
		OLED_ShowChar(48,6,t,16,1);//显示ASCII字符	   
		t++;
		if(t>'~')t=' ';
		OLED_ShowNum(103,6,(u32)t,3,16);//显示ASCII字符的码值 	
		OLED_Refresh();
		
		
		delay_ms(50000);
		delay_ms(50000);
		delay_ms(50000);
		delay_ms(50000);
		OLED_Clear(0);
	}	  
	
    while (1)
    {
        delay_ms(50);
        //Motor_Set(-630,630,630,-630);//前进
		printf("hello");
		
	    mm = VL53L0X_ReadDistance();
        if(mm == 0xFFFF)
            printf("Out of range\r\n");
        else
            printf("Distance = %d mm\r\n", mm);
        //delay_ms(500);
		
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		//char buf[10];
		// scanf("%s",buf);	//得有新行
		// printf("%s",buf);
		
        /* 1 ms 软定时器 */
        // if (tick_1ms)
        // {
        //     tick_1ms = 0;
        //     //MOTOR_SpeedLoop();      // 10 ms 或 1 ms 都可
        //     

        // }

        /* 非关键任务放这里 */
        //LED_Breath();
    }
}

/* TIM1中断回调函数 - 100us */
void TIM1_100us_Callback(void)
{
	static uint16_t cnt = 0;
	cnt++;
	if(cnt >= 10)
	{
		cnt = 0;
		//gryo_test();
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		//Servo_Control();// 舵机控制
	}
    
    // 编码器扫描 - 每1ms执行一次
    Ec11_knob_TIM_Callback();
}

void gryo_test()
{
	int16_t acc[3];
    int16_t gyro[3];
    
	// 获取陀螺仪数据
    Gryo_Update();
    // printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", fGyro[0], fGyro[1], fGyro[2], fAcc[0], fAcc[1], fAcc[2]);

    acc[0] = (int16_t)(fAcc[0] * 1000);
    acc[1] = (int16_t)(fAcc[1] * 1000);
    acc[2] = (int16_t)(fAcc[2] * 1000);

    gyro[0] = (int16_t)(fGyro[0] * 10);
    gyro[1] = (int16_t)(fGyro[1] * 10);
    gyro[2] = (int16_t)(fGyro[2] * 10);

    CDK_Update(acc, gyro); // 陀螺仪碰撞检测算法
    printf("%.3d,%.3d,%.3d,%.3d,%.3d,%.3d\r\n", gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);

    
    if (acc[1] > 800 || acc[1] < -800)
    {
        if(gyro[2] > 1800 || gyro[2] < -1800)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
        }
    }
}


/**
 * @brief  1 ms 中断回调（在 stm32f4xx_it.c 里调用）
 */
void App_1ms_IRQHandler(void)
{
    tick_1ms = 1;
}