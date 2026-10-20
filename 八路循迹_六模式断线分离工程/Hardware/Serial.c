/*******************************************************************************
 * 文 件 名 : Serial.c
 * 功    能 : 串口 USART1 初始化与收发。主要用于打印红外遥控接收到的编码，
 *            方便调试和扩展按键（波特率 9600，8 位数据，1 位停止位，无校验）。
 *            接线：PA9=TX → USB 转串口模块的 RX；PA10=RX ← 模块的 TX。
 * 说    明 : 重定向了 fputc()，所以可以直接用 printf() 往串口发数据。
 ******************************************************************************/
#include "stm32f10x.h"                  // Device header
#include <stdio.h>

#include <stdarg.h>

uint8_t Serial_RxData;    // 收到的 1 个字节
uint8_t Serial_RxFlag;    // 接收完成标志：1=收到新数据

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);   // 使能 USART1 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    // 使能 GPIOA 时钟

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // PA9=TX，复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          // PA10=RX，上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStruture;
	USART_InitStruture.USART_BaudRate = 9600;                              // 波特率 9600
	USART_InitStruture.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件流控
	USART_InitStruture.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;         // 收发都开
	USART_InitStruture.USART_Parity = USART_Parity_No;                     // 无校验
	USART_InitStruture.USART_StopBits = USART_StopBits_1;                  // 1 位停止位
	USART_InitStruture.USART_WordLength = USART_WordLength_8b;             // 8 位数据
	USART_Init(USART1,&USART_InitStruture);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);            // 开接收中断（RXNE）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       // 串口 1 中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);                               // 使能 USART1
}

void Serial_SendByte(uint8_t Byte)      // 发一个字节
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);   // 等待发送完成
	
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)   // 发一个字节数组
{
	uint16_t i;
	for(i = 0;i < Length; i++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)    // 发一个字符串（遇到 '\0' 结束）
{
	uint8_t i;
	for (i = 0;String[i] != '\0';i++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X,uint32_t Y)   // 求 X 的 Y 次方，给下面的数字发送用
{
	uint32_t Result = 1;
	while(Y --)
	{
		Result *=X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number,uint8_t Length)   // 按指定位数发送数字（十进制）
{
	uint8_t i;
	for(i = 0; i < Length; i++)
	{
		Serial_SendByte(Number / Serial_Pow(10,Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch,FILE *f)	   // 重定向 C 库的 fputc，使 printf 输出到串口
{
	Serial_SendByte(ch);
	return ch;
}


void Serial_Printf(char *format, ...)    // 自己的格式化打印（不依赖 C 库重定向）
{
	char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String);
	
}

uint8_t Serial_GetRxFlag(void)     // 查询是否收到数据：读走标志（读完自动清零）
{
	if(Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)     // 取走收到的那个字节
{
	return Serial_RxData;
}

void USART1_IRQHandler(void)       // 串口 1 接收中断
{
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE) == SET)   // 收到一个字节
	{
		Serial_RxData = USART_ReceiveData(USART1);
		Serial_RxFlag = 1;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}

