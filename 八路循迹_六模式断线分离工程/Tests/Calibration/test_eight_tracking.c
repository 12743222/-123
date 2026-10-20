#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "Irtracking.h"
#include "robot.h"
#include "Corner.h"
#include "Crossing.h"
#include "Roundabout.h"
#include "Course.h"
#include "Buzzer.h"

/* Physical wiring agreed with the user, X1 at the car's left. */
static const uint16_t channel_pins[8] = {
    GPIO_Pin_14, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_15,
    GPIO_Pin_5, GPIO_Pin_1, GPIO_Pin_10, GPIO_Pin_11
};
static uint16_t pins = 0xffffu, pwm[4], init_pins;
static int checks, failures, reads;
static unsigned buzzer_level, buzzer_edges;
void PWM_Init(void) {}
void Delay_ms(uint32_t ms) { (void)ms; }
void TIM_SetCompare1(void *p,uint16_t v) { (void)p; pwm[0]=v; }
void TIM_SetCompare2(void *p,uint16_t v) { (void)p; pwm[1]=v; }
void TIM_SetCompare3(void *p,uint16_t v) { (void)p; pwm[2]=v; }
void TIM_SetCompare4(void *p,uint16_t v) { (void)p; pwm[3]=v; }
void RCC_APB2PeriphClockCmd(int c,int e) { (void)c; (void)e; }
void GPIO_WriteBit(void *p,uint16_t pin,BitAction level) {
    if(p!=GPIOB||pin!=GPIO_Pin_0) exit(2);
    if(!buzzer_level&&level==Bit_SET) ++buzzer_edges;
    buzzer_level=(unsigned)level;
}
void GPIO_Init(void *p,GPIO_InitTypeDef *c) {
    if (p!=GPIOB || c->GPIO_Mode!=GPIO_Mode_IPU) exit(2);
    init_pins |= c->GPIO_Pin;
}
uint16_t GPIO_ReadInputData(void *p) {
    if(p!=GPIOB) exit(2);
    ++reads; return pins;
}
uint8_t GPIO_ReadInputDataBit(void *p,uint16_t b) {
    (void)p; return (pins&b)?1u:0u;
}
static void check(int ok,const char *name) {
    ++checks; if(!ok) ++failures;
    printf("%s: %s\n",ok?"PASS":"FAIL",name);
}
static int stopped(void) { return !pwm[0]&&!pwm[1]&&!pwm[2]&&!pwm[3]; }
static void line(unsigned mask) {
    unsigned i;
    pins=0xffffu;
    for(i=0;i<8;++i) if(mask&(1u<<i)) pins ^= channel_pins[i];
}
static void tick(void) {
    Irtracking_Update();
    if(pwm[0]||pwm[2]||pwm[1]>100||pwm[3]>100) exit(2);
}
static void ticks(unsigned n) { while(n--) tick(); }
static void corner_samples(unsigned mask,unsigned n) {
    line(mask); while(n--) Corner_Tick();
}
static void cross_samples(unsigned mask,unsigned n) {
    line(mask); while(n--) { Buzzer_Tick(); Crossing_Tick(); }
}
static void ring_samples(unsigned mask,unsigned n) {
    line(mask); while(n--) Roundabout_Tick();
}
static void course_samples(unsigned mask,unsigned n) {
    line(mask); while(n--) Course_Tick();
}
int main(void) {
    unsigned i;
    int before,old_l,old_r,smooth=1,all_masks=1;
    static const struct { unsigned mask,left,right; } cases[] = {
        {0x01,60,84}, {0x02,62,82}, {0x04,66,78}, {0x08,70,74},
        {0x10,74,70}, {0x20,78,66}, {0x40,82,62}, {0x80,84,60},
        {0x18,72,72}, {0x3c,72,72}, {0x0f,64,80}, {0xf0,80,64},
        {0x3f,68,76}, {0xfc,76,68}
    };
    Irtracking_Init();
    check(init_pins==0xfc22u,"initialize eight sensor inputs on PB5/PB1/PB10-15, leaving buzzer PB0 free");
    for(i=0;i<8;++i) {
        line(1u<<i); before=reads;
        check(Irtracking_ReadState()==(1u<<i) && reads==before+1,
              "X1 through X8 map left to right using exactly one GPIO snapshot");
    }
    for(i=0;i<256;++i) {
        line(i); before=reads;
        if(Irtracking_ReadState()!=i || reads!=before+1) all_masks=0;
        pins ^= 0x03ddu;
        if(Irtracking_ReadState()!=i) all_masks=0;
    }
    check(all_masks,"all 256 input combinations ignore unrelated GPIO levels");
    all_masks=1;
    for(i=0;i<256;++i) {
        unsigned expected=((i&0x03u)?1u:0u)|((i&0x0cu)?2u:0u)|
                          ((i&0x30u)?4u:0u)|((i&0xc0u)?8u:0u);
        if(Irtracking_GetRegions((uint8_t)i)!=expected) all_masks=0;
    }
    check(all_masks,"all eight probes contribute to their correct route regions");
    for(i=0;i<sizeof(cases)/sizeof(cases[0]);++i) {
        Irtracking_Reset(); line(cases[i].mask); ticks(40);
        check(pwm[1]==cases[i].left && pwm[3]==cases[i].right,
              "eight-channel weighted tracking steers toward the observed line");
    }
    Irtracking_Reset(); line(0x18); ticks(40); line(1); ticks(2);
    check(pwm[1]==72&&pwm[3]==72,"two noisy samples do not change the confirmed center");
    tick(); check(pwm[1]==69&&pwm[3]==75,"third sample begins bounded steering");
    line(0x80);
    for(i=0;i<40;++i) {
        old_l=pwm[1]; old_r=pwm[3]; tick();
        if(abs((int)pwm[1]-old_l)>3 || abs((int)pwm[3]-old_r)>3) smooth=0;
    }
    check(smooth&&pwm[1]==84&&pwm[3]==60,"direction changes retain the PWM ramp");
    line(0x0f); ticks(40);
    check(!stopped()&&!Irtracking_HasFault(),"four left sensors black is steering, not all-black stop");
    line(0xff); tick(); check(stopped(),"all eight black stop immediately");
    line(0x18); ticks(2); check(stopped(),"restart still requires confirmed line samples");
    tick(); check(pwm[1]==3&&pwm[3]==3,"confirmed center restarts gradually");
    Irtracking_Reset(); line(0x18); ticks(40); line(0); ticks(3);
    check(stopped(),"line loss without a known side coasts before searching");
    line(0x18); ticks(40); check(!stopped(),"line can recover before the search turn starts");
    Irtracking_Reset(); line(1); ticks(40); line(0); ticks(101);
    check(!pwm[1]&&pwm[3]==72&&!Irtracking_HasFault(),"persistent ordinary loss turns to search instead of stopping");
    line(0x18); ticks(1000); check(!stopped()&&pwm[1]==72&&pwm[3]==72,"returning line resumes tracking after a long search");
    Irtracking_Reset(); line(1); ticks(40); line(0xc3); ticks(40);
    check(pwm[1]==60&&pwm[3]==84,"symmetric outer branches preserve left history");
    Irtracking_Reset(); line(0x80); ticks(40); line(0xc3); ticks(40);
    check(pwm[1]==84&&pwm[3]==60,"symmetric outer branches preserve right history");
    all_masks=1;
    for(i=1;i<255;++i) {
        Irtracking_Reset(); line(i); ticks(40);
        if(stopped()||Irtracking_HasFault()) all_masks=0;
    }
    check(all_masks,"every partial eight-channel pattern remains usable for tracking");
    Irtracking_UpdateState(256);
    check(stopped()&&Irtracking_HasFault(),"states outside eight bits still latch a fault");
    Irtracking_Reset(); Irtracking_UpdateRingState(256);
    check(stopped()&&Irtracking_HasFault(),"ring tracking rejects states outside eight bits");
    Irtracking_Reset(); line(1); ticks(40); Irtracking_Forward();
    check(pwm[1]==63&&pwm[3]==81,"crossing straight command ramps from actual PWM");
    for(i=0;i<10;++i) Irtracking_Forward();
    check(pwm[1]==72&&pwm[3]==72,"forced straight does not spend a loss deadline");
    Irtracking_Resume(101,60);
    check(stopped()&&Irtracking_HasFault(),"invalid PWM is rejected");
    Irtracking_Reset(); line(1); ticks(40); before=reads;
    for(i=0;i<70000;++i) Irtracking_UpdateRingState(0);
    check(reads==before&&pwm[1]==IRTRACKING_RING_SEARCH_INNER_PWM&&pwm[3]==72&&!Irtracking_HasFault(),
          "ring search can continue without additional GPIO reads or loss timeout");
    for(i=0;i<IRTRACKING_SEARCH_DELAY_TICKS;++i) Irtracking_UpdateState(0);
    check(pwm[1]==60&&pwm[3]==84&&!Irtracking_HasFault(),
          "ordinary tracking keeps two-wheel correction during the coast phase");
    for(i=0;i<IRTRACKING_SEARCH_DELAY_TICKS+60;++i) Irtracking_UpdateState(0);
    check(!pwm[1]&&pwm[3]==72&&!Irtracking_HasFault(),
          "ordinary tracking switches to the last side instead of stopping");

    Course_StartMode(2); course_samples(0x18,30); course_samples(0x60,40);
    check(Course_GetMode()==2&&Corner_GetDisplay()==10&&pwm[1]==80&&pwm[3]==64,
          "step one corrects a narrow line under X6/X7 instead of committing a right corner");
    course_samples(0x18,40);
    check(Course_GetMode()==2&&Corner_GetDisplay()==10&&pwm[1]==72&&pwm[3]==72,
          "returning to the center after a right offset resumes straight driving");
    Corner_Start(0); corner_samples(0x18,30); corner_samples(0x06,40);
    check(Corner_GetDisplay()==10&&pwm[1]==64&&pwm[3]==80,
          "narrow line under X2/X3 remains left correction rather than a committed corner");

    Corner_Start(0); corner_samples(0x18,30); corner_samples(0x03,40);
    check(Corner_GetDisplay()==10,"left outer pair alone cannot trigger a corner");
    corner_samples(0x09,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==11&&!pwm[1]&&pwm[3]>0,
          "X1 and X4 confirm a left corner in the agreed mounting direction");
    corner_samples(0,CORNER_TURN_MIN_TICKS); corner_samples(0x18,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==12,"center X4/X5 reacquires the left corner exit");
    Corner_Start(1); corner_samples(0x18,30); corner_samples(0xc0,40);
    check(Corner_GetDisplay()==10,"right outer pair alone cannot trigger a corner");
    corner_samples(0x90,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==11&&pwm[1]>0&&!pwm[3],
          "X5 and X8 confirm a right corner");
    corner_samples(0,CORNER_TURN_MIN_TICKS); corner_samples(0x18,CORNER_CONFIRM_TICKS);
    check(Corner_GetDisplay()==12,"center X4/X5 reacquires the right corner exit");

    Crossing_Start(); cross_samples(0x18,30); cross_samples(0x55,3);
    check(Crossing_GetDisplay()==5,"one probe per region is not an all-black crossing");
    cross_samples(0xff,2);
    check(buzzer_edges==0&&buzzer_level==0,"two all-black samples do not beep before crossing confirmation");
    cross_samples(0x18,1); cross_samples(0xff,2);
    check(buzzer_edges==0,"interrupted all-black noise cannot trigger a crossing beep");
    cross_samples(0xff,1);
    check(Crossing_GetDisplay()==6&&!stopped(),"eight black probes confirm a crossing");
    check(buzzer_level==1&&buzzer_edges==1,"confirmed crossing starts one buzzer pulse while motors keep driving");
    cross_samples(0x18,20);
    check(Crossing_GetDisplay()==7,"crossing exits on the narrow center line");
    check(buzzer_edges==1,"staying on and leaving the same crossing do not retrigger its beep");
    for(i=0;i<2;++i) {
        Roundabout_Start((uint8_t)i); ring_samples(0x18,30);
        ring_samples(i?0x60u:0x06u,40);
        check(Roundabout_GetDisplay()==10&&pwm[1]>0&&pwm[3]>0,
              "adjacent probes crossing a region boundary do not falsely enter a ring");
        ring_samples(0x18,30);
        ring_samples(i?0x90u:0x09u,ROUND_CONFIRM_TICKS);
        check(Roundabout_GetDisplay()==11&&!stopped(),
              "narrow same-side eight-channel branch enters the selected ring");
    }
    printf("%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
