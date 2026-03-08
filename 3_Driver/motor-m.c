/**
  * @file	电机文件-麦克纳姆轮版
  * @brief  功能：集成电机需要用到PWM例程的函数
  *         适合电机为1脚PWM输入，1脚控制正反转的驱动
  * 
  *         车轮方向A↑↗ ↖↑ B（内八）
  *                 D↑↖ ↗↑ C
  *  ------------------------------------------------------------------*/
#include "motor-m.h"

//电机最大占空比	1000
//====================用户配置部分===================

//适配设备的初始化函数
void Motor_Init(void) 
{
	// 打开PWM
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//PWM启动函数
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	Motor_Set(0,0,0,0);//速度为0
}

/**
  * @brief   设置每个电机速度（PWM占空比）
  * @param 参数  num：电机编号（1-4），PWM：PWM值
  */   
void Motor_Set(short pwm1,short pwm2,short pwm3,short pwm4)  
{

		LF1_PWM(pwm1>0? pwm1:0);
		LF2_PWM(pwm1>0? 0:-pwm1);

		RF1_PWM(pwm2>0? pwm2:0);
		RF2_PWM(pwm2>0? 0:-pwm2);
		
		LB1_PWM(pwm3>0? pwm3:0);
		LB2_PWM(pwm3>0? 0:-pwm3);
		
		RB1_PWM(pwm4>0? pwm4:0);
		RB2_PWM(pwm4>0? 0:-pwm4);
	
	//刹车模式
	if(pwm1==0&&pwm2==0&&pwm3==0&&pwm4==0)
	{
		LF1_PWM(1000);
		LF2_PWM(1000);
		LB1_PWM(1000);
		LB2_PWM(1000);
		RF1_PWM(1000);
		RF2_PWM(1000);
		RB1_PWM(1000);
		RB2_PWM(1000);
	}
			
}



//=====================备用函数===================

/* 麦轮四向移动的驱动程序*/
//麦轮位置解算
//前后量，左右量，旋转量(逆时针正方向)
//void Motor_move(short FB,short LR,short turn)
//{
//    u16 move_sum;
//    float k;
//    //PWM量总和
//    move_sum=abs(LR)+abs(FB)+abs(turn);
//    if (move_sum > 3000)
//    {
//      k = 3000.0/move_sum;//当速度和过大时（转成小数）
//      LR *= k;
//      FB *= k;
//      turn *= k;
//    }
//    
//    Motor_Set(1, FB+LR*-1 +turn*-1);//左前驱动
//    Motor_Set(2, FB+LR*1+turn );//右前
//    Motor_Set(3, FB+LR*-1 +turn );//右后
//    Motor_Set(4, FB+LR*1+turn*-1);//左后
//}


//麦克纳姆轮的位置解算
/**
  * @brief  计算x和y变化的位置 (单位是mm
  *
  * @param 参数 每个编码器的值
  *
  * @note 说明  坐标轴↓y →x，y轴是反向的
  */
//前后
//float Motor_y(short Motor_enc1,short Motor_enc2,short Motor_enc3,short Motor_enc4)
//{
//    float y;
//    float Motor_1,Motor_2,Motor_3,Motor_4;//轮子的距离
//    Motor_1=lunzi*(float)(Motor_enc1)/enc_sum;//轮子一圈的长度*一圈的占比
//    Motor_2=lunzi*(float)(Motor_enc2)/enc_sum;
//    Motor_3=lunzi*(float)(Motor_enc3)/enc_sum;
//    Motor_4=lunzi*(float)(Motor_enc4)/enc_sum;
//    //Y轴的距离计算
//    y=(Motor_1+Motor_2+Motor_3+Motor_4)/1.414f;
//    return -y;//y轴是负方向的
//}

////左右
//float Motor_x(short Motor_enc1,short Motor_enc2,short Motor_enc3,short Motor_enc4)
//{
//    float x;
//    float Motor_1,Motor_2,Motor_3,Motor_4;//轮子的距离
//    Motor_1=lunzi*(float)(Motor_enc1)/enc_sum;//轮子一圈的长度*一圈的占比
//    Motor_2=lunzi*(float)(Motor_enc2)/enc_sum;
//    Motor_3=lunzi*(float)(Motor_enc3)/enc_sum;
//    Motor_4=lunzi*(float)(Motor_enc4)/enc_sum;
//    //X轴的距离计算
//    x=(Motor_1-Motor_2+Motor_3-Motor_4)/1.414f; //这里后缀f是把数值转化为float型计算
//    return x;   
//}

