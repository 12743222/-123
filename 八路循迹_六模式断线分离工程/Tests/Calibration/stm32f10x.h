#ifndef TEST_STM32_H
#define TEST_STM32_H
#include <stdint.h>
typedef struct { int GPIO_Mode; uint16_t GPIO_Pin; int GPIO_Speed; } GPIO_InitTypeDef;
#define GPIOA ((void *)3)
#define GPIO_Mode_Out_PP 2
#define RCC_APB2Periph_GPIOA 2
typedef enum {Bit_RESET=0,Bit_SET=1} BitAction;
#define GPIO_Pin_0 (1u << 0)
#define GPIO_Pin_1 (1u << 1)
#define GPIO_Pin_2 (1u << 2)
#define GPIO_Pin_3 (1u << 3)
#define GPIO_Pin_4 (1u << 4)
#define GPIO_Pin_5 (1u << 5)
#define GPIO_Pin_6 (1u << 6)
#define GPIO_Pin_7 (1u << 7)
#define GPIOB ((void *)1)
#define TIM4 ((void *)2)
#define GPIO_Pin_10 (1u << 10)
#define GPIO_Pin_11 (1u << 11)
#define GPIO_Pin_12 (1u << 12)
#define GPIO_Pin_13 (1u << 13)
#define GPIO_Pin_14 (1u << 14)
#define GPIO_Pin_15 (1u << 15)
#define GPIO_Mode_IPU 1
#define GPIO_Speed_50MHz 1
#define RCC_APB2Periph_GPIOB 1
#define RCC_APB2Periph_AFIO 4
#define GPIO_Remap_SWJ_JTAGDisable 0x00300200u
#define ENABLE 1
void RCC_APB2PeriphClockCmd(int clock, int enabled);
void GPIO_Init(void *port, GPIO_InitTypeDef *config);
uint8_t GPIO_ReadInputDataBit(void *port, uint16_t pin);
uint16_t GPIO_ReadInputData(void *port);
void TIM_SetCompare1(void *timer, uint16_t value);
void TIM_SetCompare2(void *timer, uint16_t value);
void TIM_SetCompare3(void *timer, uint16_t value);
void TIM_SetCompare4(void *timer, uint16_t value);
void GPIO_SetBits(void *port, uint16_t pins);
void GPIO_WriteBit(void *port,uint16_t pin,BitAction value);
void GPIO_PinRemapConfig(uint32_t remap,int enabled);
#endif
