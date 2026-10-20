/*******************************************************************************
 * 文 件 名 : Irobstacle.c
 * 功    能 : 红外避障传感器（两路红外对管）初始化与读取。
 *            左探头 PA11，右探头 PA12，均为上拉输入。
 *            返回值：0 = 该侧检测到障碍物（红外被反射回来），1 = 无障碍。
 *            （模块上的电位器可调节检测距离）
 * 说    明 : 只在模式 2（避障）中使用，由 main.c 的 Robot_Avoidance() 读取。
 ******************************************************************************/
#include "stm32f10x.h"                  // Device header

void Irobstacle_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);   // 避障探头接在 GPIOA
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          // 上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; // PA11=左探头，PA12=右探头
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


uint8_t Left_Irobstacle_Get(void)      // 读左侧避障探头
{
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
}

uint8_t Right_Irobstacle_Get(void)     // 读右侧避障探头
{
	return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
}

