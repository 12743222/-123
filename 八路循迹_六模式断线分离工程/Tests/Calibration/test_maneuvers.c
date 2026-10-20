#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "route_inputs.h"
#include "Corner.h"
#include "Roundabout.h"

static uint16_t pwm[4];
static unsigned mask=6;
static int reads,checks,failures;
void PWM_Init(void) {}
void Delay_ms(uint32_t ms) { (void)ms; }
void TIM_SetCompare1(void *p,uint16_t v) { (void)p;pwm[0]=v; }
void TIM_SetCompare2(void *p,uint16_t v) { (void)p;pwm[1]=v; }
void TIM_SetCompare3(void *p,uint16_t v) { (void)p;pwm[2]=v; }
void TIM_SetCompare4(void *p,uint16_t v) { (void)p;pwm[3]=v; }
void RCC_APB2PeriphClockCmd(int c,int e) { (void)c;(void)e; }
void GPIO_WriteBit(void *port,uint16_t pin,BitAction value) {
    if(port!=GPIOB||pin!=GPIO_Pin_0) exit(2);
    (void)value;
}
void GPIO_Init(void *p,GPIO_InitTypeDef *c) { (void)p;(void)c; }
uint16_t GPIO_ReadInputData(void *p) {
    if(p!=GPIOB) exit(2);
    ++reads;
    return route_input_pins(mask);
}
uint8_t GPIO_ReadInputDataBit(void *p,uint16_t bit)
{ return (GPIO_ReadInputData(p)&bit)?1u:0u; }
static int stopped(void)
{ return !pwm[0]&&!pwm[1]&&!pwm[2]&&!pwm[3]; }
static void check(int ok,const char *name)
{ ++checks;if(!ok)++failures;printf("%s: %s\n",ok?"PASS":"FAIL",name); }
static void corner_samples(unsigned value,unsigned count)
{
    mask=value;
    while(count--)
    {
        int before=reads;
        Corner_Tick();
        if(reads-before>1 || pwm[0] || pwm[2] || pwm[1]>100 || pwm[3]>100)
        { puts("FAIL: corner motor or sensor snapshot");exit(2); }
    }
}
static void ring_samples(unsigned value,unsigned count)
{
    mask=value;
    while(count--)
    {
        int before=reads;
        Roundabout_Tick();
        if(reads-before>1 || pwm[0] || pwm[2] || pwm[1]>100 || pwm[3]>100)
        { puts("FAIL: ring motor or sensor snapshot");exit(2); }
    }
}

int main(void)
{
    int j;
    Corner_Start(0);
    corner_samples(6,30);corner_samples(3,CORNER_CONFIRM_TICKS-1);
    check(Corner_GetDisplay()==10,"left corner needs confirmed branch samples");
    corner_samples(3,1);
    check(Corner_GetDisplay()==11&&pwm[1]==0&&pwm[3]>0,
          "confirmed left corner pivots immediately");
    corner_samples(0,200);
    check(Corner_GetDisplay()==11&&pwm[1]==0&&pwm[3]==68,
          "left corner keeps turning through one second of white");
    corner_samples(6,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==12&&pwm[1]>0&&pwm[3]>0,
          "outgoing line restores tracking after left corner");
    corner_samples(6,CORNER_AFTER_TICKS+CORNER_EXIT_TICKS);
    check(Corner_GetDisplay()==0&&stopped(),
          "standalone corner observation finishes and stops");

    Corner_Start(1);
    corner_samples(6,30);corner_samples(12,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==11&&pwm[1]>0&&pwm[3]==0,
          "confirmed right corner pivots immediately");
    corner_samples(0,200);
    check(Corner_GetDisplay()==11&&pwm[1]==68&&pwm[3]==0,
          "right corner keeps turning through one second of white");
    corner_samples(6,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==12&&!stopped(),
          "outgoing line restores tracking after right corner");

    Corner_Start(0);
    corner_samples(6,30);corner_samples(3,CORNER_CONFIRM_TICKS);
    corner_samples(0,CORNER_FLIP_WHITE_TICKS+CORNER_TURN_MAX_TICKS);
    check(Corner_GetDisplay()==9&&stopped(),
          "corner retains the bounded search timeout");
    Corner_Start(1);corner_samples(12,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==9&&stopped(),
          "starting on a branch without centered approach is rejected");

    Roundabout_Start(0);
    ring_samples(6,30);ring_samples(7,ROUND_CONFIRM_TICKS);
    check(Roundabout_GetDisplay()==11&&!stopped(),
          "left ring recognizes its entrance");
    ring_samples(6,ROUND_ENTRY_TICKS);
    check(Roundabout_GetDisplay()==12&&pwm[1]<pwm[3],
          "left ring enters counterclockwise loop");
    ring_samples(6,ROUND_LAP_MIN_TICKS);
    /* Walk the polygon vertices so the return junction is accepted only after a lap. */
    for(j=0;j<(int)ROUND_LAP_SIDES;++j)
    {
        ring_samples(6,ROUND_VERTEX_ARM_TICKS+2);
        ring_samples(1,1);
    }
    ring_samples(7,ROUND_CONFIRM_TICKS);
    check(Roundabout_GetDisplay()==13&&!stopped(),
          "left ring recognizes return junction");
    ring_samples(0,105);
    check(Roundabout_GetDisplay()==13&&pwm[1]==36&&pwm[3]==72,
          "left ring exit white arcs toward the left instead of spinning");
    ring_samples(6,ROUND_FOUND_LINE_TICKS+ROUND_LEFT_EXIT_MIN_TICKS);
    check(Roundabout_GetDisplay()==14&&!stopped(),
          "left ring resumes outgoing tracking after stable line");

    printf("%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
