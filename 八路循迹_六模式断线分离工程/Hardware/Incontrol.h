/*******************************************************************************
 * 文 件 名 : Incontrol.h
 * 功    能 : 红外遥控接收模块接口声明
 ******************************************************************************/
#ifndef __INCONTROL_H
#define __INCONTROL_H

void IRremote_Init(void);      // 红外遥控器初始化：PA8 上拉输入 + EXTI8 下降沿中断
uint8_t IRremote_Counttime();  // 测量 PA8 高电平持续时间（单位约 20us，上限 250）

//定义全局变量
extern uint32_t IR_Receivecode;   // 解码得到的 32 位接收码（地址+反码+命令+反码）
extern uint8_t  IR_Receiveflag;   // 接收完成标志：1=收到完整一帧，处理完要清零

#endif

