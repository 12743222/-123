/*******************************************************************************
 * 文 件 名 : Key.c
 * 功    能 : 板载按键（PA15）初始化与读取。
 *            按键一端接 PA15、另一端接地，所以按下读到低电平 0。
 * 说    明 : 本固件没调用 Key_GetNum()（它会阻塞等松手），而是由 main 主循环
 *            直接读 PA15 电平，配合 Calibration 状态机做短按/长按识别。
 *            Key_GetNum() 保留备用。
 ******************************************************************************/
#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // 按键接在 PA15
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           // 上拉输入：松手为高电平
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
 * 函 数 名 : Key_GetNum
 * 功    能 : 读取按键，返回 1 表示"按下了一次"。
 *            内部做了 20ms 消抖，并 while 等待松手（阻塞式），
 *            因此按住不放时不会连续返回，主循环里可以放心地用它切换模式。
 * 输    入 : 无
 * 输    出 : 1=按下过一次，0=没有按下
 ******************************************************************************/
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0)   // 按下（低电平）
	{
		Delay_ms(20);                                     // 延时 20ms 消抖
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0);  // 等待松手
		Delay_ms(20);                                     // 松手后再消抖
		KeyNum = 1;
	}
	return KeyNum;
}

