#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "Buzzer.h"

static unsigned level=2, edges, initialized, checks, failures;
static void check(int ok,const char *name) {
    ++checks; if(!ok) ++failures;
    printf("%s: %s\n",ok?"PASS":"FAIL",name);
}
void RCC_APB2PeriphClockCmd(int clock,int enabled) {
    if(clock!=RCC_APB2Periph_GPIOB||!enabled) exit(2);
}
void GPIO_WriteBit(void *port,uint16_t pin,BitAction value) {
    if(port!=GPIOB||pin!=GPIO_Pin_0) exit(2);
    if(level!=1&&value==Bit_SET) ++edges;
    level=(unsigned)value;
}
void GPIO_Init(void *port,GPIO_InitTypeDef *cfg) {
    if(port!=GPIOB||cfg->GPIO_Pin!=GPIO_Pin_0||cfg->GPIO_Mode!=GPIO_Mode_Out_PP) exit(2);
    check(level==0,"PB0 is preloaded low before its output is enabled");
    initialized=1;
}
static void ticks(unsigned count) { while(count--) Buzzer_Tick(); }
int main(void) {
    unsigned i,high_frames=0;
    Buzzer_Init();
    check(initialized&&level==0,"startup keeps the active-high buzzer silent");
    ticks(200); check(level==0&&!edges,"idle control ticks never beep");
    Buzzer_Beep(); check(level==1&&edges==1,"a crossing event starts a beep immediately");
    ticks(99); check(level==1,"beep remains on for the first 495 milliseconds");
    ticks(1); check(level==0,"beep ends at exactly 100 control ticks, nominally 500ms");
    ticks(200); check(level==0&&edges==1,"a completed beep does not restart by itself");
    Buzzer_Stop(); edges=0;
    for(i=0;i<4;++i) Buzzer_Beep();
    for(i=0;i<500;++i) {
        if(level) ++high_frames;
        Buzzer_Tick();
    }
    check(edges==4,"four close crossing events produce four separate beeps");
    check(high_frames==400,"each queued beep retains its full 500ms duration");
    check(level==0,"buzzer returns low after all crossing events finish");
    Buzzer_Beep(); Buzzer_Beep(); Buzzer_Stop();
    check(level==0,"manual stop immediately silences an active beep");
    i=edges; ticks(500);
    check(level==0&&edges==i,"manual stop also cancels queued beeps");
    Buzzer_Beep(); check(level==1,"a later run can beep again after stop");
    Buzzer_Init(); i=edges; ticks(500);
    check(level==0&&edges==i,"reinitialization clears active and queued sounds");
    printf("%u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
