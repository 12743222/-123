/*******************************************************************************
 * 文 件 名 : LEDSEG.c
 * 功    能 : 一位数码管显示当前状态：1~4=当前模式，8=等待，0=完成，9=中止。
 *            （本固件专用于标定，不再显示循迹/避障/遥控的旧含义）
 *            段选接 PA0~PA7（PA0=a 段 …… PA7=dp 段），推挽输出。
 *            段码表是按"共阳极数码管"写的：某位为 0 时对应段点亮。
 ******************************************************************************/
#include "stm32f10x.h"                  // Device header

uint8_t LedShowData[]={0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0x88,0x83,0xC6,0xA1,0x86,0x8E};//0,1,2,3,4,5,6,7,8,9 //定义数码管显示数据
uint16_t GPIO_PIN_x[] = {GPIO_Pin_0,GPIO_Pin_1,GPIO_Pin_2,GPIO_Pin_3,GPIO_Pin_4,GPIO_Pin_5,GPIO_Pin_6,GPIO_Pin_7};  // 段 a~dp 对应的引脚
void LEDSEG_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // 段选接在 PA0~PA7
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        // 推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
											GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);   // 全部置高 = 所有段熄灭（共阳极）
}

/*******************************************************************************
 * 函 数 名 : Digital_Display
 * 功    能 : 让数码管显示 0~F 中的某个数字。
 * 输    入 : val —— 要显示的数字（0~15，数组有16个元素，越界则熄灭）
 * 输    出 : 无
 * 说    明 : 逐位把段码写进 PA0~PA7；段码的 bit0 对应 PA0(a)，bit7 对应 PA7(dp)。
 ******************************************************************************/
void Digital_Display(uint8_t val)
{
	uint8_t i;
	uint16_t display_val = val < 16 ? LedShowData[val] : 0xFF;    // 取出该数字的段码
  for(i = 0;i < 8;i++)
	{
	  GPIO_WriteBit(GPIOA,GPIO_PIN_x[i],(BitAction)((display_val>>i) & 0x01));   // 一位一位地输出
	}
}

