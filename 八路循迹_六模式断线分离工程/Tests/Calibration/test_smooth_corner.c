#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "Irtracking.h"
#include "Corner.h"

static const uint16_t channel_pins[8]={GPIO_Pin_14,GPIO_Pin_12,GPIO_Pin_13,GPIO_Pin_15,
                                     GPIO_Pin_5,GPIO_Pin_1,GPIO_Pin_10,GPIO_Pin_11};
static uint16_t pins=0xffffu,pwm[4];
static unsigned checks,failures,reads,result;
void PWM_Init(void) {}
void Delay_ms(uint32_t ms) { (void)ms; }
void TIM_SetCompare1(void *p,uint16_t v) { (void)p;pwm[0]=v; }
void TIM_SetCompare2(void *p,uint16_t v) { (void)p;pwm[1]=v; }
void TIM_SetCompare3(void *p,uint16_t v) { (void)p;pwm[2]=v; }
void TIM_SetCompare4(void *p,uint16_t v) { (void)p;pwm[3]=v; }
void RCC_APB2PeriphClockCmd(int c,int e) { (void)c;(void)e; }
void GPIO_Init(void *p,GPIO_InitTypeDef *c) { (void)p;(void)c; }
void GPIO_WriteBit(void *p,uint16_t pin,BitAction v) { (void)p;(void)pin;(void)v; }
uint16_t GPIO_ReadInputData(void *p) { if(p!=GPIOB)exit(2);++reads;return pins; }
uint8_t GPIO_ReadInputDataBit(void *p,uint16_t pin) { return (GPIO_ReadInputData(p)&pin)?1u:0u; }
static void check(int ok,const char *name) {
    ++checks;if(!ok)++failures;printf("%s: %s\n",ok?"PASS":"FAIL",name);
}
static void line(unsigned mask) {
    unsigned i;pins=0xffffu;
    for(i=0;i<8;++i)if(mask&(1u<<i))pins^=channel_pins[i];
}
static void track(unsigned mask,unsigned n) {
    line(mask);while(n--)Irtracking_Update();
}
static void samples(unsigned mask,unsigned n) {
    line(mask);
    while(n--) {
        unsigned before=reads;result=Corner_Tick();
        if(reads-before>1||pwm[0]||pwm[2]||pwm[1]>100||pwm[3]>100)exit(2);
    }
}
static void begin(unsigned right) {
    Irtracking_Reset();Corner_StartCourse((uint8_t)right);
    samples(0x18,30);samples(right?0x90u:0x09u,3);
}
int main(void) {
    unsigned i,right,old_l,old_r;
    int bounded=1;
    static const struct {unsigned mask,left,right;} cases[]={
        {0x01,60,84},{0x02,62,82},{0x04,66,78},{0x08,70,74},
        {0x10,74,70},{0x20,78,66},{0x40,82,62},{0x80,84,60},
        {0x18,72,72},{0x0f,64,80},{0xf0,80,64},{0x3f,68,76},{0xfc,76,68}
    };
    for(i=0;i<sizeof(cases)/sizeof(cases[0]);++i) {
        Irtracking_Reset();track(cases[i].mask,50);
        check(pwm[1]==cases[i].left&&pwm[3]==cases[i].right,
              "correction follows line position smoothly while straight speed stays 72");
    }
    Irtracking_Reset();track(0x18,50);
    for(i=0;i<160;++i) {
        old_l=pwm[1];old_r=pwm[3];track((i/4)%2?0x08u:0x10u,1);
        if(abs((int)pwm[1]-(int)old_l)>3||abs((int)pwm[3]-(int)old_r)>3||
           pwm[1]<70||pwm[1]>74||pwm[3]<70||pwm[3]>74)bounded=0;
    }
    check(bounded,"alternating center probes cannot create a large steering swing");
    Irtracking_Reset();Irtracking_Resume(48,48);
    for(i=0;i<40;++i)Irtracking_UpdateStateAtSpeed(0x80,48);
    check(pwm[1]==56&&pwm[3]==40,"temporary exit-speed limit scales and bounds its steering correction");
    track(0x18,40);
    check(pwm[1]==72&&pwm[3]==72,"a temporary exit-speed limit cannot remain enabled in ordinary tracking");
    Irtracking_UpdateStateAtSpeed(0x18,0);
    check(Irtracking_HasFault()&&!pwm[1]&&!pwm[3],"zero exit speed is rejected with a latched stop");
    Irtracking_Reset();Irtracking_UpdateStateAtSpeed(0x18,73);
    check(Irtracking_HasFault()&&!pwm[1]&&!pwm[3],"invalid exit speed cannot exceed the normal speed");
    for(right=0;right<2;++right) {
        begin(right);samples(0,16);
        check(right?(pwm[1]<=CORNER_PIVOT_PWM&&pwm[3]==0):(pwm[1]==0&&pwm[3]<=CORNER_PIVOT_PWM),
              "corner search is capped at 60 instead of 78");
        samples(right?0x80u:0x01u,6);
        check(right?(pwm[1]<=CORNER_ALIGN_PWM&&pwm[3]==0):(pwm[1]==0&&pwm[3]<=CORNER_ALIGN_PWM),
              "first outgoing-line sighting slows the committed turn to 42");
        samples(right?0x20u:0x04u,4);
        check(result!=0&&Corner_GetDisplay()==11,
              "an inner-edge probe alone cannot finish a corner before the center arrives");
        samples(0x18,1);
        check(pwm[1]==CORNER_EXIT_PWM&&pwm[3]==CORNER_EXIT_PWM&&result!=0,
              "first central sighting removes one-wheel yaw immediately at 48/48");
        samples(0x18,2);
        check(result!=0&&pwm[1]==CORNER_EXIT_PWM&&pwm[3]==CORNER_EXIT_PWM,
              "confirmed line enters low-speed recovery instead of advancing the route");
        samples(0x18,CORNER_EXIT_TICKS-20);
        check(result!=0&&pwm[1]>=CORNER_EXIT_PWM&&pwm[1]<IRTRACKING_STRAIGHT_SPEED&&pwm[1]==pwm[3],
              "outgoing recovery blends the speed up over the exit window");
        samples(0x18,CORNER_EXIT_TICKS);
        check(result==0,
              "stable recovery hands both moving wheels to the next route step");
        track(0x18,10);
        check(pwm[1]==72&&pwm[3]==72,"normal straight speed resumes gradually after corner recovery");

        begin(right);samples(0,16);samples(0x18,3);
        check(pwm[1]==CORNER_EXIT_PWM&&pwm[3]==CORNER_EXIT_PWM,
              "an early valid outgoing line is honored before the old 200ms forced turn ends");
        samples(0,1);
        check(pwm[1]>0&&pwm[3]>0,"one white noise sample does not restart a pivot");
        samples(0,2);
        check(right?(pwm[1]<=CORNER_EXIT_PWM&&pwm[3]==0):(pwm[1]==0&&pwm[3]<=CORNER_EXIT_PWM),
              "confirmed recovery loss returns only to slow same-direction alignment");
        samples(0x18,3);samples(0x18,CORNER_EXIT_TICKS+5);
        check(result==0,"a recovered center line can still finish the same corner");

        begin(right);samples(0,16);
        for(i=0;i<120;++i) { samples(0x18,3);samples(0,3); }
        check(result==9&&!pwm[1]&&!pwm[3],
              "repeated recovery attempts cannot extend the total corner timeout");

        begin(right);samples(0,1);samples(0x18,60);
        check(result!=0&&Corner_GetDisplay()==11,
              "one white glitch cannot falsely prove that the original incoming line has cleared");

        begin(right);samples(0x18,80);
        check(result!=0&&Corner_GetDisplay()==11,
              "the original incoming line cannot complete a turn before it has cleared");
        samples(0,CORNER_FLIP_WHITE_TICKS+CORNER_TURN_MAX_TICKS);
        check(result==9&&!pwm[1]&&!pwm[3],"corner search retains its total timeout and stop");
        samples(0x18,100);
        check(result==9&&!pwm[1]&&!pwm[3],"late line sightings cannot restart a timed-out corner");
    }
    printf("%u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
