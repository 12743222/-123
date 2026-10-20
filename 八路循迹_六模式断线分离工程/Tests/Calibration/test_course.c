#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "Calibration.h"
#include "Course.h"
#include "Corner.h"
#include "Crossing.h"
#include "Roundabout.h"
#include "Gap.h"
#include "Irtracking.h"
#include "Buzzer.h"
static uint16_t pwm[4];
static unsigned mask=0x18,checks,failures,reads,buzzer_level,buzzer_edges;
static const uint16_t pins[8]={GPIO_Pin_14,GPIO_Pin_12,GPIO_Pin_13,GPIO_Pin_15,GPIO_Pin_5,GPIO_Pin_1,GPIO_Pin_10,GPIO_Pin_11};
void PWM_Init(void) {}
void Delay_ms(uint32_t ms) {(void)ms;}
void TIM_SetCompare1(void *p,uint16_t v){(void)p;pwm[0]=v;}
void TIM_SetCompare2(void *p,uint16_t v){(void)p;pwm[1]=v;}
void TIM_SetCompare3(void *p,uint16_t v){(void)p;pwm[2]=v;}
void TIM_SetCompare4(void *p,uint16_t v){(void)p;pwm[3]=v;}
void RCC_APB2PeriphClockCmd(int c,int e){(void)c;(void)e;}
void GPIO_Init(void *p,GPIO_InitTypeDef *c){(void)p;(void)c;}
void GPIO_WriteBit(void *p,uint16_t pin,BitAction v){
    if(p!=GPIOB||pin!=GPIO_Pin_0)exit(2);
    if(!buzzer_level&&v==Bit_SET)++buzzer_edges;
    buzzer_level=(unsigned)v;
}
uint16_t GPIO_ReadInputData(void *p){
    uint16_t value=0xffffu;unsigned i;if(p!=GPIOB)exit(2);++reads;
    for(i=0;i<8;++i)if(mask&(1u<<i))value^=pins[i];return value;
}
uint8_t GPIO_ReadInputDataBit(void *p,uint16_t pin){return (GPIO_ReadInputData(p)&pin)?1u:0u;}
static int stopped(void){return !pwm[0]&&!pwm[1]&&!pwm[2]&&!pwm[3];}
static void check(int ok,const char *s){++checks;if(!ok)++failures;printf("%s: %s\n",ok?"PASS":"FAIL",s);}
static void samples(unsigned m,unsigned n){
    mask=m;while(n--){unsigned before=reads;Buzzer_Tick();Course_Tick();
        if(reads-before>1||pwm[0]||pwm[2]||pwm[1]>100||pwm[3]>100)exit(2);
    }
}
static void begin(unsigned mode){
    Buzzer_Stop();buzzer_edges=0;mask=0x18;Course_StartMode((uint8_t)mode);samples(0x18,30);
    check(Course_GetMode()==mode&&Course_GetDisplay()==mode&&!stopped(),"each independent mode tracks its incoming line");
}
static void latched(void){
    samples(0xff,CORNER_EXIT_TICKS+CORNER_AFTER_TICKS+20);check(Course_Tick()==0&&stopped(),"completion stays stopped even on a new feature");
}
int main(void){
    unsigned right,i,j;
    begin(1);samples(0x0f,10);
    check(Crossing_GetDisplay()==5&&buzzer_edges==0,"one-sided branch is not a crossing");
    samples(0x18,10);samples(0x7e,2);samples(0x18,1);samples(0x7e,2);
    check(Crossing_GetDisplay()==5&&!buzzer_edges,"wide crossing still needs consecutive confirmation");
    samples(0x7e,1);check(Crossing_GetDisplay()==6&&buzzer_edges==1&&buzzer_level,"six-probe wide stripe triggers one crossing beep");
    samples(0x80,30);check(Course_Tick()==1&&!stopped(),"outer-only stripe clearance cannot finish crossing");
    samples(0x18,3);check(Course_Tick()==0&&stopped(),"confirmed central outgoing line stops after one crossing");
    latched();check(!buzzer_level&&buzzer_edges==1,"crossing sound finishes once without restarting mode");

    for(right=0;right<2;++right){
        begin(2);samples(right?0x60u:0x06u,40);
        check(Corner_GetDisplay()==10&&pwm[1]>0&&pwm[3]>0,"narrow off-center line never commits an automatic corner");
        samples(0xff,10);check(Corner_GetDisplay()==10&&pwm[1]>0&&pwm[3]>0,"ambiguous both-side stripe cannot pick an arbitrary corner");
        samples(0x18,10);
        samples(0x0f,1);samples(0xf0,1);samples(0x0f,1);samples(0xf0,1);
        check(Corner_GetDisplay()==10,"alternating left-right evidence cannot accumulate one corner");
        samples(0x18,1);samples(right?0xf0u:0x0fu,3);
        check(Corner_GetDisplay()==11&&(right?(pwm[1]>0&&!pwm[3]):(!pwm[1]&&pwm[3]>0)),"automatic corner locks the actually observed left or right direction");
        samples(0,16);samples(right?0x80u:0x01u,6);
        check(right?(pwm[1]<=48&&!pwm[3]):(!pwm[1]&&pwm[3]<=48),"automatic turn slows at first outgoing edge");
        samples(0x18,1);check(pwm[1]==54&&pwm[3]==54,"central reacquisition immediately removes one-wheel yaw");
        samples(0x18,CORNER_EXIT_TICKS+5);
        check(Corner_GetDisplay()==10&&!stopped(),"mode 2 re-arms after a corner and keeps going");
        for(j=1;j<CORNER_COURSE_CORNERS;++j){
            samples(0x18,3);
            samples(right?0xf0u:0x0fu,3);
            samples(0,16);samples(right?0x80u:0x01u,6);
            samples(0x18,1);
            samples(0x18,CORNER_EXIT_TICKS+5);
        }
        check(Course_Tick()==0&&stopped(),"mode 2 stops after the requested number of corners");
        latched();check(!buzzer_edges,"corner mode does not beep");
    }
    begin(2);samples(0xf8,3);
    check(Corner_GetDisplay()==11&&pwm[1]>0&&!pwm[3],"far-right probe plus centre commits a right corner");
    begin(2);samples(0x1f,3);
    check(Corner_GetDisplay()==11&&!pwm[1]&&pwm[3]>0,"far-left probe plus centre commits a left corner");
    begin(2);samples(0x78,40);
    check(Corner_GetDisplay()==10&&pwm[1]>0&&pwm[3]>0,"inner-right stripe without the far probe keeps tracking");
    begin(2);samples(0x18,10);samples(0x78,2);samples(0,6);
    check(Corner_GetDisplay()==10,"brief white loss keeps the corner approach running");
    begin(2);samples(0x18,10);samples(0x78,2);samples(0,CORNER_WHITE_COMMIT_TICKS);
    check(Corner_GetDisplay()==11&&pwm[1]>0&&!pwm[3],"missed corner falls back to the side the line was last seen on");
    begin(2);samples(0xf0,3);samples(0,CORNER_FLIP_WHITE_TICKS+4);
    check(Corner_GetDisplay()==11&&!pwm[1]&&pwm[3]>0,"a committed turn with no outgoing line flips once to the other side");
    begin(2);samples(0x0f,3);samples(0,CORNER_FLIP_WHITE_TICKS+CORNER_TURN_MAX_TICKS);
    check(Course_Tick()==9&&stopped(),"committed automatic corner retains bounded search");

    begin(3);check(pwm[1]==60&&pwm[3]==60,"gap approach retains the original straight speed");
    samples(0,COURSE_GAP_WHITE_SAMPLES-1);samples(0x18,1);
    check(Gap_GetDisplay()==10,"brief line dropout cannot start a gap");
    samples(0,COURSE_GAP_WHITE_SAMPLES);check(Gap_GetDisplay()==11&&stopped(),"confirmed gap stops before the original calibrated maneuver");
    samples(0,COURSE_GAP_ENTRY_SETTLE_TICKS+100);
    check(pwm[1]==76&&pwm[3]==59,"first gap arc uses the slightly stronger 76/59 PWM");
    samples(0,CAL_GAP_ARC1_TICKS);check(pwm[1]==60&&pwm[3]==60,"gap straight keeps original 60/60 PWM");
    samples(0x18,CAL_GAP_LINE_SAMPLES);check(Gap_GetDisplay()==6&&stopped(),"confirmed far-side line retains settle stop");
    samples(0x18,CAL_GAP_SETTLE_TICKS+25);check(pwm[1]==85&&pwm[3]==50,"gap exit arc keeps original 85/50 PWM");
    samples(0x18,200);check(Course_Tick()==0&&stopped(),"one gap finishes after stable low-speed outgoing recovery");latched();
    begin(3);samples(0x18,10);samples(0,COURSE_GAP_WHITE_SAMPLES+COURSE_GAP_ENTRY_SETTLE_TICKS+CAL_GAP_ARC1_TICKS+CAL_GAP_STRAIGHT_MAX_TICKS+2);
    check(Course_Tick()==9&&stopped(),"gap without a far-side line fails within original deadline");
    begin(3);samples(0x0cu,160);samples(0,COURSE_GAP_WHITE_SAMPLES);
    samples(0,COURSE_GAP_ENTRY_SETTLE_TICKS+40);
    check(pwm[1]==73&&pwm[3]==62,"gap arc carries the learned straight-line differential");
    samples(0,CAL_GAP_ARC1_TICKS);
    check(pwm[1]==57&&pwm[3]==63,"gap straight keeps that differential instead of drifting right");
    begin(4);samples(0,COURSE_GAP_WHITE_SAMPLES);
    check(Gap_GetDisplay()==11&&stopped(),"second break confirms its gap before the arc");
    samples(0,COURSE_GAP_ENTRY_SETTLE_TICKS+20);
    check(pwm[1]==72&&pwm[3]==63,"second break uses a single right arc at 72/63");
    samples(0,CAL_GAP2_ARC_TICKS-20);
    check(pwm[1]==72&&pwm[3]==63,"second break holds the 3 s right arc to the end");
    samples(0x18,COURSE_GAP_RECOVER_TICKS+5);
    samples(0x18,COURSE_GAP_RECOVER_TICKS+5);
    check(Course_Tick()==1&&!stopped(),"second break hands over to the finish run after the 3 s arc");
    samples(0x0f,COURSE_FINISH_CONFIRM_TICKS+2);
    check(Course_Tick()==1&&!stopped(),"four black probes do not stop the car");
    samples(0x1f,COURSE_FINISH_CONFIRM_TICKS+1);
    check(Course_Tick()==0&&stopped(),"five black probes stop the car at once");
    begin(4);samples(0,COURSE_GAP_WHITE_SAMPLES);
    samples(0,COURSE_GAP_ENTRY_SETTLE_TICKS+20);
    samples(0xff,COURSE_FINISH_CONFIRM_TICKS+1);
    check(Course_Tick()==0&&stopped(),"wide black during the second-break arc stops the car at once");
    begin(4);samples(0,COURSE_GAP_WHITE_SAMPLES);
    samples(0,COURSE_GAP_ENTRY_SETTLE_TICKS+CAL_GAP2_ARC_TICKS+5);
    samples(0x18,COURSE_FINISH_RUN_MAX_TICKS+20);
    check(Course_Tick()==9&&stopped(),"finish run without any wide black times out with a fault");

    for(right=0;right<2;++right){
        begin(right?6:5);samples(right?0xf8u:0x1fu,3);
        check(Roundabout_GetDisplay()==11,"selected ring detects its entrance");
        samples(0x18,ROUND_ENTRY_TICKS);samples(0x18,ROUND_LAP_MIN_TICKS);
        for(j=0;j<ROUND_LAP_SIDES;++j){
            samples(0x18,ROUND_VERTEX_ARM_TICKS+2);
            samples(right?0xc0u:0x03u,3);
        }
        samples(right?0xc3u:0x1fu,right?16:3);check(Roundabout_GetDisplay()==13,"departed ring confirms its return junction");
        samples(right?0x80u:0x01u,30);
        check(Roundabout_GetDisplay()==13&&Course_Tick()==1,"outer-only ring line cannot complete the outgoing handover");
        samples(0x18,right?ROUND_RIGHT_EXIT_RUN_TICKS+20:ROUND_LEFT_EXIT_MIN_TICKS+10);
        check(Course_Tick()==0&&stopped(),"one left or right ring stops only after confirmed central outgoing line");latched();
    }
    begin(5);samples(0x1f,3);samples(0,COURSE_MODE_MAX_TICKS);
    check(Course_Tick()==9&&stopped(),"ring white search cannot run forever in independent mode");
    for(i=1;i<=6;++i){begin(i);samples(0x18,COURSE_MODE_APPROACH_TICKS);check(Course_Tick()==9&&stopped(),"missing selected feature has a bounded approach");}
    Course_StartMode(0);check(Course_Tick()==9&&stopped(),"invalid mode zero stops");
    Course_StartMode(7);check(Course_Tick()==9&&stopped(),"invalid mode above six stops");
    printf("%u checks, %u failures\n",checks,failures);return failures?1:0;
}
