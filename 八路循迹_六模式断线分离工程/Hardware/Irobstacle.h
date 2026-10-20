/*******************************************************************************
 * 文 件 名 : Irobstacle.h
 * 功    能 : 红外避障模块接口声明
 ******************************************************************************/
#ifndef __IROBSTACLE_H
#define __IROBSTACLE_H

void Irobstacle_Init(void);          // 避障探头初始化（PA11/PA12 上拉输入）
uint8_t Left_Irobstacle_Get(void);   // 读左侧探头：0=有障碍，1=无障碍
uint8_t Right_Irobstacle_Get(void);  // 读右侧探头：0=有障碍，1=无障碍

#endif

