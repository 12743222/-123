/*******************************************************************************
 * 文 件 名 : Serial.h
 * 功    能 : 串口（USART1）接口声明
 ******************************************************************************/
#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

void Serial_Init(void);                                            // 串口初始化：9600 8N1，开接收中断
void Serial_SendByte(uint8_t Byte);                                // 发送一个字节
void Serial_SendArray(uint8_t *Array, uint16_t Length);            // 发送一个数组
void Serial_SendString(char *String);                              // 发送字符串
void Serial_SendNumber(uint32_t Number,uint8_t Length);            // 发送指定长度的十进制数字
void Serial_Printf(char *format, ...);                             // 格式化打印（类似 printf）

uint8_t Serial_GetRxFlag(void);                                    // 查询是否收到数据（读后自动清零）
uint8_t Serial_GetRxData(void);                                    // 取走收到的字节

#endif

