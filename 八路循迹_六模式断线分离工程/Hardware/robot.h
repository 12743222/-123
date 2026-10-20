/*******************************************************************************
 * 文 件 名 : robot.h
 * 功    能 : 机器人运动控制接口声明
 * 说    明 : speed 取 0~100（PWM 占空比百分数）；time 为动作持续时间，单位 ms，
 *            传 0 表示不延时（电机保持该速度直到下次调用）。
 *            本固件实际只用 robot_Init() 和 robot_speed()，
 *            其余 makerobo_* 基本动作是模板保留，供以后扩展用。
 ******************************************************************************/
#ifndef __ROBOT_H
#define __ROBOT_H

void robot_Init(void);                                  // 机器人初始化：初始化 TIM4 四路 PWM
void robot_speed(uint8_t left1_speed,uint8_t left2_speed,uint8_t right1_speed,uint8_t right2_speed); // 直接设置四路 PWM 占空比
// 基本运动
void makerobo_run(uint8_t speed,uint16_t time); // 机器人前进
void makerobo_brake(uint16_t time);             // 机器人停止（刹车），参数为延时 ms
void makerobo_Left(int8_t speed,uint16_t time); // 左转函数：左轮停、右轮转
void makerobo_Spin_Left(int8_t speed,uint16_t time);  // 左旋转函数：原地左旋
void makerobo_Right(int8_t speed,uint16_t time);      //右转函数：左轮转、右轮停
void makerobo_Spin_Right(int8_t speed,uint16_t time); //右旋转函数：原地右旋
void makerobo_back(int8_t speed,uint16_t time);      //后退函数
#endif

