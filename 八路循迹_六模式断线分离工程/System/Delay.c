/*******************************************************************************
 * 文 件 名 : Delay.c
 * 功    能 : 基于 SysTick 内核定时器的软件延时（微秒 / 毫秒 / 秒）。
 * 说    明 : 每 1us 需要 72 个计数（系统时钟 72MHz），SysTick 的 LOAD 是
 *            24 位，最大 16777215，所以 Delay_us 的最大延时约 233015us。
 *            SysTick 默认时钟源为 HCLK/8，这里在 CTRL 中选了 HCLK（第 2 位）。
 ******************************************************************************/
#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)                 // 循环调用微秒延时实现毫秒延时
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)                  // 循环调用毫秒延时实现秒延时
	{
		Delay_ms(1000);
	}
} 

