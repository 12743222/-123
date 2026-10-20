/*******************************************************************************
 * 文 件 名 : Incontrol.c
 * 功    能 : 红外遥控接收头（VS1838B 一类）的初始化与"高电平计时"。
 *
 * 硬件连接：PA8 ← 红外接收头 OUT
 *          接收头输出是反相的：空闲时输出高电平，收到 38kHz 载波时输出低电平。
 *
 * NEC 协议（本程序按这个协议解码，帧格式低位先发）：
 *   引导码：9ms 载波(低) + 4.5ms 空闲(高)
 *   数据 0：0.56ms 载波(低) + 0.56ms 空闲(高)
 *   数据 1：0.56ms 载波(低) + 1.69ms 空闲(高)
 *   一帧   ：8 位地址 + 8 位地址反码 + 8 位命令 + 8 位命令反码 = 32 位
 *   所以只要测量"高电平持续时间"，就能区分引导码 / 数据 0 / 数据 1，
 *   具体阈值判定和移位接收在 main.c 的 Irscan() 里完成。
 *
 * 本文件的 EXTI9_5_IRQHandler 是空的：解码已经改到 main.c 的 Irscan()
 * 里用轮询方式做，这里保留外部中断初始化只是沿用原例程。
 ******************************************************************************/
#include "stm32f10x.h"                  // Device header
#include "Delay.h"

//端口定义
#define IRED_PORT 			GPIOA  
#define IRED_PIN 			  GPIO_Pin_8
#define IRED_PORT_RCC		RCC_APB2Periph_GPIOA

uint32_t IR_Receivecode;	  //定义一个32位数据变量，保存接收码
uint8_t  IR_Receiveflag;   //定义一个8位数据的变量，用于指示接收标志（1=收到完整一帧）

/*******************************************************************************
* 函 数 名         : IRremote_Init
* 函数功能		     : 红外端口初始化函数	  时钟端口及外部中断初始化 
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IRremote_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 开启GPIO时钟及管脚复用时钟 */
	RCC_APB2PeriphClockCmd(IRED_PORT_RCC|RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitStructure.GPIO_Pin=IRED_PIN;                        //红外接收：PA8
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;                  //上拉输入（接收头空闲输出高电平）
	GPIO_Init(IRED_PORT,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8); //选择GPIO管脚用作外部中断线路
	EXTI_ClearITPendingBit(EXTI_Line8);                        //先清一次挂起位，避免上电就进中断
	
	/* 设置外部中断的模式 */ 
	EXTI_InitStructure.EXTI_Line=EXTI_Line8;                   //PA8 对应 EXTI8
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;          //中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;      //下降沿触发：载波到来（高→低）时触发
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure); 

	/* 设置NVIC参数 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   //打开全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级为0（最高）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	 //响应优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   //使能
	NVIC_Init(&NVIC_InitStructure);

}

/*******************************************************************************
* 函 数 名         : IRremote_Counttime
* 函数功能		     : 高电平持续时间，将记录的时间保存在t中返回，其中一次大约20us 
* 输    入         : 无
* 输    出         : t（单位约 20us；上限 250 → 约 5ms，超时直接返回 250）
* 说    明         : 在 Irscan() 中被调用，用来区分引导码(200~250)、
*                    数据 1(60~90)、数据 0(10~50)
*******************************************************************************/
uint8_t IRremote_Counttime()
{
	u8 t=0;
	while(GPIO_ReadInputDataBit(IRED_PORT,IRED_PIN)==1)//高电平
	{
		t++;
		Delay_us(20);                                    //每个计数约 20us
		if(t>=250) return t;//超时溢出：说明是空闲高电平（不是有效信号）
	}
	return t;
}

void EXTI9_5_IRQHandler(void)	  //红外遥控外部中断
{
//	uint8_t Tim=0,Ok=0,Data,Num=0;

//	while(1)
//	{
//	  if(GPIO_ReadInputDataBit(IRED_PORT,IRED_PIN)==1)
//		{
//			 Tim=IRremote_Counttime();//获得此次高电平时间

//			 if(Tim>=250) break;//不是有用的信号

//			 if(Tim>=200 && Tim<250)
//			 {
//			 	Ok=1;//收到起始信号
//			 }
//			 else if(Tim>=60 && Tim<90)
//			 {
//			 	Data=1;//收到数据 1
//			 }
//			 else if(Tim>=10 && Tim<50)
//			 {
//			 	Data=0;//收到数据 0
//			 }

//			 if(Ok==1)
//			 {
//			 	IR_Receivecode<<=1;
//				IR_Receivecode+=Data;

//				if(Num>=32)
//				{
//					IR_Receiveflag=1;
//				  break;
//				}
//			 }
//			 Num++;
//		}
//	}
//	EXTI_ClearITPendingBit(EXTI_Line8);	
	/* 上面这段解码代码已整体搬到 main.c 的 Irscan() 里（改为在主循环轮询执行），
	 * 所以本中断服务函数留空。如果想让中断方式解码生效，可把上面代码取消注释，
	 * 并把 main.c 模式 3 里的 Irscan() 换成空循环。 */
}

