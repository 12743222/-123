/*******************************************************************************
 * 文 件 名 : PWM.c
 * 功    能 : 用 TIM4 的 4 个通道产生 4 路 PWM，驱动电机（每路一个方向）。
 *              TIM4_CH1 → PB6
 *              TIM4_CH2 → PB7
 *              TIM4_CH3 → PB8
 *              TIM4_CH4 → PB9
 *
 * 频率与占空比计算：
 *    TIM4 挂在 APB1 上，APB1 分频系数为 2（APB1=36MHz），
 *    但定时器时钟会再乘 2，所以 TIM4 的计数时钟 = 72MHz。
 *    PWM 频率 = 72MHz / (预分频 36 × 重装值 100) = 20kHz
 *    占空比   = CCR / 100，所以形参直接传 0~100 就是百分比。
 ******************************************************************************/
#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	    GPIO_InitTypeDef GPIO_InitStructure;
      TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
      TIM_OCInitTypeDef TIM_OCInitStructure;

      //使能定时器TIM4时钟，注意TIM4时钟为APB1
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
      //使能PWM输出GPIO时钟
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
                                                                           
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//定时器TIM4的PWM输出通道1,TIM4_CH1
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//定时器TIM4的PWM输出通道2,TIM4_CH2
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//定时器TIM4的PWM输出通道3,TIM4_CH3
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//定时器TIM4的PWM输出通道4,TIM4_CH4
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

      TIM_TimeBaseStructure.TIM_Period = 100 - 1;//arr;//自动重装值：决定了 PWM 周期，占空比按 0~100 计算
      TIM_TimeBaseStructure.TIM_Prescaler =36 - 1;//psc; //时钟预分频数：72MHz/36 = 2MHz 计数频率
      TIM_TimeBaseStructure.TIM_ClockDivision = 0;
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
      TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //初始化TIM4
     
      //初始化TIM4_CH1的PWM模式
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;      //PWM模式1
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //使能输出
      TIM_OCInitStructure.TIM_Pulse = 0; //初始比较值 0 → 占空比 0%
      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //有效电平为高
      TIM_OC1Init(TIM4, &TIM_OCInitStructure);//初始化TIM4_CH1

      //初始化TIM4_CH2的PWM模式
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      TIM_OCInitStructure.TIM_Pulse = 0;
      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
       //TIM4_CH2初始化,OC2
      TIM_OC2Init(TIM4, &TIM_OCInitStructure);

       //初始化TIM4_CH3的PWM模式
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      TIM_OCInitStructure.TIM_Pulse = 0;
      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
      TIM_OC3Init(TIM4, &TIM_OCInitStructure);

      //初始化TIM4_CH4的PWM模式
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      TIM_OCInitStructure.TIM_Pulse = 0;
      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
      TIM_OC4Init(TIM4, &TIM_OCInitStructure);

      //使能4个通道的预装载寄存器
      TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//OC1
      TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);//OC2
      TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);//OC3
      TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);//OC4
      TIM_ARRPreloadConfig(TIM4, ENABLE); //使能重装寄存器

      TIM_Cmd(TIM4, ENABLE);//使能定时器TIM4,准备工作 
}

