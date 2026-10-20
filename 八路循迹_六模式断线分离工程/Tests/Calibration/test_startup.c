#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include "stm32f10x.h"
#include "Calibration.h"
#include "Irtracking.h"
#define main firmware_main
#include "../../User/main.c"
#undef main
static jmp_buf finished;
static unsigned init_count, displayed, ticks, first_delay, checks, failures;
static int motors_stopped;
static unsigned buzzer_level=2, buzzer_output;
static void check(int ok,const char *name) {
    ++checks; if(!ok) ++failures;
    printf("%s: %s\n",ok?"PASS":"FAIL",name);
}
void RCC_APB2PeriphClockCmd(int clock,int enabled) { (void)clock;(void)enabled; }
void GPIO_PinRemapConfig(uint32_t remap,int enabled) { (void)remap;(void)enabled; }
void GPIO_WriteBit(void *port,uint16_t pin,BitAction value) {
    if(port!=GPIOB||pin!=GPIO_Pin_0) exit(2);
    buzzer_level=(unsigned)value;
}
void GPIO_Init(void *port,GPIO_InitTypeDef *cfg) {
    if(port!=GPIOB||cfg->GPIO_Pin!=GPIO_Pin_0||cfg->GPIO_Mode!=GPIO_Mode_Out_PP) exit(2);
    check(buzzer_level==0,"preload the buzzer off level before enabling its output");
    buzzer_output=1;
}
uint8_t GPIO_ReadInputDataBit(void *port,uint16_t pin) { (void)port;(void)pin;return 1; }
void Key_Init(void) { ++init_count; }
void LEDSEG_Init(void) { ++init_count; }
void robot_Init(void) { ++init_count;motors_stopped=1; }
void Irtracking_Init(void) { ++init_count; }
void Calibration_Init(void) { ++init_count;motors_stopped=1; }
void Digital_Display(uint8_t value) { displayed=value; }
void Calibration_Tick(uint8_t pressed) {
    if(!ticks) check(!pressed&&displayed==8,"menu polling starts on the first control tick without a warmup wait");
    ++ticks;
}
uint8_t Calibration_GetDisplay(void) { return 1; }
void Delay_ms(uint32_t ms) {
    if(!first_delay) {
        check(init_count==5&&motors_stopped,"all hardware initializes with motors stopped before the menu");
        check(ms==CAL_TICK_MS,"power-on goes straight into the five-millisecond control loop");
        check(buzzer_output&&buzzer_level==0,"PB0 is held at the buzzer off level during startup");
        first_delay=ms;
    } else {
        check(ms==CAL_TICK_MS,"normal five-millisecond control loop follows startup");
        check(displayed==1&&motors_stopped,"startup returns to mode one without automatic motion");
        check(buzzer_output&&buzzer_level==0,"menu control does not reactivate the buzzer");
        longjmp(finished,1);
    }
}
int main(void) {
    if(!setjmp(finished)) firmware_main();
    printf("%u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
