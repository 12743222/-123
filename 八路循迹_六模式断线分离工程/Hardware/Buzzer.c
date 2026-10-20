#include "stm32f10x.h"
#include "Buzzer.h"
#include "Calibration.h"

#define BEEP_TICKS (500u / CAL_TICK_MS)
#define GAP_TICKS   (20u / CAL_TICK_MS)
static uint8_t remaining, pending, sounding;

void Buzzer_Stop(void)
{
    remaining=pending=sounding=0;
    GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET);
}

void Buzzer_Init(void)
{
    GPIO_InitTypeDef cfg;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    Buzzer_Stop(); /* Preload low before enabling the active-high buzzer output. */
    cfg.GPIO_Pin=GPIO_Pin_0;
    cfg.GPIO_Mode=GPIO_Mode_Out_PP;
    cfg.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&cfg);
}

static void start_beep(void)
{
    sounding=1;
    remaining=BEEP_TICKS;
    GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);
}

void Buzzer_Beep(void)
{
    if(!remaining) start_beep();
    else if(pending<255) ++pending;
}

void Buzzer_Tick(void)
{
    if(!remaining || --remaining) return;
    if(sounding)
    {
        sounding=0;
        GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET);
        remaining=GAP_TICKS;
    }
    else if(pending)
    {
        --pending;
        start_beep();
    }
}
