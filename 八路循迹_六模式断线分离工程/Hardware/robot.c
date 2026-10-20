/*******************************************************************************
 * 文 件 名 : robot.c
 * 功    能 : 机器人底层运动控制（把"前进/后退/左转/右转/原地旋转/刹车"翻译成
 *            四路 PWM 占空比），PWM 由 TIM4 的 4 个通道产生：
 *              TIM4_CH1 → PB6  左电机输入 1（正转）
 *              TIM4_CH2 → PB7  左电机输入 2（反转）
 *              TIM4_CH3 → PB8  右电机输入 1（正转）
 *              TIM4_CH4 → PB9  右电机输入 2（反转）
 *            每个电机的两路输入配合，就可以实现正转/反转/停转。
 *
 * 速度参数：0 ~ 100（对应 0% ~ 100% 占空比，因为 PWM_Init 里 ARR=100）。
 * 时间参数：单位 ms，函数内部 Delay_ms() 之后返回；传 0 表示不延时（一直保持）。
 * 方向修正：本车两个电机的两根线接反了，已在下面 robot_speed 里用
 *          MOTOR_DIR_REVERSE = 1 做整体反向（不需要拆线）。
 * 注    意：各运动函数结尾"清零 PWM"的那行都被注释掉了，所以函数返回后电机
 *          会保持在最后一次设定的速度上，需要靠 makerobo_brake() 停车。
 ******************************************************************************/
#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Delay.h"
// 机器人初始化
void robot_Init(void)
{
	PWM_Init();   // 只需初始化 TIM4 的四路 PWM 输出
}

//四路PWM控制速度调节
//left1_speed/left2_speed：左电机正/反转占空比；right1_speed/right2_speed：右电机正/反转占空比
void robot_speed(uint8_t left1_speed,uint8_t left2_speed,uint8_t right1_speed,uint8_t right2_speed)
{	
/* ---------------------------------------------------------------------------
 * 电机方向总开关 MOTOR_DIR_REVERSE
 *   0 = 出厂方向（正转线接驱动板 CH1 / CH3）
 *   1 = 整体反向：两轮的实际转向与程序相反时用（现象：前进变后退、
 *       左旋/右旋也对调，循迹和避障方向一起反，俗称"电机线接反了"）
 * 本车实测两个电机都接反，所以设为 1，相当于把每个电机的两路 PWM 输入交叉输出。
 * 以后如果重新焊接/插接过电机线，方向又正常了，把这里改回 0 即可，不用动别的代码。
 * --------------------------------------------------------------------------- */
#define MOTOR_DIR_REVERSE   1

#if (MOTOR_DIR_REVERSE == 1)
	    TIM_SetCompare1(TIM4,left2_speed);    // CH1 → PB6  交叉输出 = 反向
	    TIM_SetCompare2(TIM4,left1_speed);    // CH2 → PB7  交叉输出 = 反向
	    TIM_SetCompare3(TIM4,right2_speed);   // CH3 → PB8  交叉输出 = 反向
	    TIM_SetCompare4(TIM4,right1_speed);   // CH4 → PB9  交叉输出 = 反向
#else
	    TIM_SetCompare1(TIM4,left1_speed);    // CH1 → PB6 左电机正转
	    TIM_SetCompare2(TIM4,left2_speed);    // CH2 → PB7 左电机反转
	    TIM_SetCompare3(TIM4,right1_speed);   // CH3 → PB8 右电机正转
	    TIM_SetCompare4(TIM4,right2_speed);   // CH4 → PB9 右电机反转
#endif
}

// 基本的运动函数
// 机器人前进
void makerobo_run(int8_t speed,uint16_t time)  //前进函数
{
      if(speed > 100)          // 限幅：速度不能超过 100%
			{
				speed = 100;
			}
			if(speed < 0)          // 限幅：速度不能为负
			{
				speed = 0;
			}
	    robot_speed(speed,0,speed,0);   // 左、右电机同时正转 → 前进
			Delay_ms(time);                 // 时间为毫秒
			//robot_speed(0,0,0,0);           // 机器人停止（保留速度，此处不清零）
 
}

void makerobo_brake(uint16_t time) //刹车函数
{
		robot_speed(0,0,0,0);     // 电机停止：四路占空比全部为 0
		Delay_ms(time);          // 时间为毫秒（这里的 time 是延时，不是刹车力度）
}

void makerobo_Left(int8_t speed,uint16_t time) //左转函数
{
	    if(speed > 100)
			{
				speed = 100;
			}
			if(speed < 0)
			{
				speed = 0;
			}
		robot_speed(0,0,speed,0);   // 左轮停、右轮正转 → 以左轮为支点向左偏转
		Delay_ms(time);                 //时间为毫秒  
	  //robot_speed(0,0,0,0);           // 机器人停止

}

void makerobo_Spin_Left(int8_t speed,uint16_t time) //左旋转函数
{
		  if(speed > 100)
			{
				speed = 100;
			}
			if(speed < 0)
			{
				speed = 0;
			}  
		robot_speed(0,speed,speed,0);   // 左轮反转、右轮正转 → 原地左旋
		Delay_ms(time);                    //时间为毫秒 
    //robot_speed(0,0,0,0);           // 机器人停止			
}

void makerobo_Right(int8_t speed,uint16_t time) //右转函数
{
	    if(speed > 100)
			{
				speed = 100;
			}
			if(speed < 0)
			{
				speed = 0;
			}
		robot_speed(speed,0,0,0);   // 左轮正转、右轮停 → 以右轮为支点向右偏转
		Delay_ms(time);                 //时间为毫秒  
	  //robot_speed(0,0,0,0);           // 机器人停止

}

void makerobo_Spin_Right(int8_t speed,uint16_t time) //右旋转函数
{
		  if(speed > 100)
			{
				speed = 100;
			}
			if(speed < 0)
			{
				speed = 0;
			}  
		robot_speed(speed,0,0,speed);   // 左轮正转、右轮反转 → 原地右旋
		Delay_ms(time);                    //时间为毫秒 
    //robot_speed(0,0,0,0);           // 机器人停止			
}

void makerobo_back(int8_t speed,uint16_t time)  //后退函数
{
      if(speed > 100)
			{
				speed = 100;
			}
			if(speed < 0)
			{
				speed = 0;
			}
	    robot_speed(0,speed,0,speed);   // 左、右电机同时反转 → 后退
			Delay_ms(time);                 // 时间为毫秒
			//robot_speed(0,0,0,0);           // 机器人停止
 
}

