#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "route_inputs.h"
#include "Calibration.h"
#include "Course.h"
#include "robot.h"
#include "Buzzer.h"

static uint16_t pwm[4];
static unsigned mask=6;
static int checks,failures,reads;
static unsigned buzzer_level;
void PWM_Init(void) {}
void Delay_ms(uint32_t ms) { (void)ms; }
void TIM_SetCompare1(void *p,uint16_t v) { (void)p;pwm[0]=v; }
void TIM_SetCompare2(void *p,uint16_t v) { (void)p;pwm[1]=v; }
void TIM_SetCompare3(void *p,uint16_t v) { (void)p;pwm[2]=v; }
void TIM_SetCompare4(void *p,uint16_t v) { (void)p;pwm[3]=v; }
void RCC_APB2PeriphClockCmd(int c,int e) { (void)c;(void)e; }
void GPIO_WriteBit(void *port,uint16_t pin,BitAction value) {
    if(port!=GPIOB||pin!=GPIO_Pin_0) exit(2);
    buzzer_level=(unsigned)value;
}
void GPIO_Init(void *p,GPIO_InitTypeDef *c) { (void)p;(void)c; }
uint16_t GPIO_ReadInputData(void *p) {
    if(p!=GPIOB) exit(2);
    ++reads;
    return route_input_pins(mask);
}
uint8_t GPIO_ReadInputDataBit(void *p,uint16_t pin)
{ return (GPIO_ReadInputData(p)&pin)?1u:0u; }
static int stopped(void)
{ return !pwm[0]&&!pwm[1]&&!pwm[2]&&!pwm[3]; }
static void check(int ok,const char *name)
{ ++checks;if(!ok)++failures;printf("%s: %s\n",ok?"PASS":"FAIL",name); }
static void tick(int pressed)
{
    int before=reads;
    Calibration_Tick((uint8_t)pressed);
    if(reads-before>1 || pwm[0] || pwm[2] || pwm[1]>100 || pwm[3]>100)
    { puts("FAIL: multiple snapshots, reverse or invalid PWM");exit(2); }
}
static void ticks(int pressed,unsigned count) { while(count--)tick(pressed); }
static void reset(void) { mask=6;Calibration_Init();ticks(0,4); }
static void short_press(void) { ticks(1,20);ticks(0,4); }
static void arm(void) { ticks(1,200);ticks(0,4); }
static void select_mode(unsigned mode)
{ unsigned i;reset();for(i=1;i<mode;++i)short_press(); }

int main(void)
{
    unsigned mode;
    robot_speed(70,0,70,0);
    reset();check(stopped()&&Calibration_GetDisplay()==1,"reset selects crossing mode and stops");
    ticks(1,3);ticks(0,4);
    check(Calibration_GetDisplay()==1&&stopped(),"button bounce cannot select a mode");
    for(mode=2;mode<=6;++mode) {
        short_press();check(Calibration_GetDisplay()==mode&&stopped(),"short press selects each of the six independent modes");
    }
    short_press();check(Calibration_GetDisplay()==1,"mode six wraps to one");
    for(mode=1;mode<=6;++mode) {
        select_mode(mode);arm();
        check(Calibration_GetDisplay()==8&&stopped(),"long press waits without motion");
        ticks(0,CAL_WAIT_TICKS-1);check(stopped(),"each mode waits the full two seconds");
        tick(0);check(Course_GetMode()==mode&&Calibration_GetDisplay()==mode,"menu dispatches the chosen feature and displays its mode");
        ticks(0,30);check(!stopped(),"selected mode follows the approach line");
        Buzzer_Beep();Buzzer_Beep();tick(1);
        check(Calibration_GetDisplay()==9&&stopped()&&buzzer_level==0,"raw press stops every mode and silences buzzer");
        ticks(1,200);ticks(0,200);
        check(stopped()&&Calibration_GetDisplay()==9&&buzzer_level==0,"holding abort cannot restart motors or queued sound");
    }
    select_mode(1);arm();ticks(0,CAL_WAIT_TICKS+30);
    mask=15;ticks(0,3);mask=6;ticks(0,20);
    check(Calibration_GetDisplay()==0&&stopped(),"crossing completion reaches menu done state without dispatching another feature");
    check(buzzer_level==1,"automatic motor stop does not truncate the 500ms crossing beep");
    ticks(0,80);check(!buzzer_level&&stopped(),"done state continues buzzer ticks to finish its sound");
    short_press();check(Calibration_GetDisplay()==2&&stopped(),"completed run can select the next mode without reset");
    for(mode=2;mode<=6;++mode) {
        select_mode(mode);arm();ticks(0,CAL_WAIT_TICKS+30);
        if(mode==2) { mask=3;ticks(0,3);mask=0;ticks(0,20); }
        else if(mode==3||mode==4) { mask=0;ticks(0,6+40+30); }
        else { mask=mode==5?7u:14u;ticks(0,3);mask=0;ticks(0,40); }
        check(!stopped(),"maneuver is active before its emergency-stop test");
        tick(1);check(stopped()&&Calibration_GetDisplay()==9,"press interrupts committed turn, gap arc and ring white search");
    }
    select_mode(1);arm();ticks(0,10);tick(1);
    check(stopped()&&Calibration_GetDisplay()==9,"press cancels countdown");
    reset();check(Calibration_GetDisplay()==1&&stopped(),"reset returns to mode one");
    printf("%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
