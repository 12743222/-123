#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "route_inputs.h"
#include "Calibration.h"
#include "Roundabout.h"
#include "Irtracking.h"
static uint16_t pwm[4];
static unsigned mask=6;
static int checks,failures,reads;
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
uint8_t GPIO_ReadInputDataBit(void *p,uint16_t b) { return (GPIO_ReadInputData(p)&b)?1:0; }
static int stopped(void) { return !pwm[0]&&!pwm[1]&&!pwm[2]&&!pwm[3]; }
static void check(int ok,const char *label) {
    ++checks;if(!ok)++failures;printf("%s: %s\n",ok?"PASS":"FAIL",label);
}
static void tick(void) {
    int before=reads;Roundabout_Tick();
    if(reads-before>1||pwm[0]||pwm[2]||pwm[1]>100||pwm[3]>100)exit(2);
}
static void samples(unsigned m,unsigned n) { mask=m;while(n--)tick(); }
static void lap_vertices(unsigned right) {
    unsigned k;
    for(k=0;k<ROUND_LAP_SIDES;++k) {
        samples(6,ROUND_VERTEX_ARM_TICKS+2);
        samples(right?8u:1u,1);
    }
    samples(6,ROUND_VERTEX_BOOST_TICKS+ROUND_VERTEX_ARM_TICKS+2);
}
static void begin_loop(unsigned right) {
    mask=6;Roundabout_Start((uint8_t)right);samples(6,30);
    samples(right?14:7,3);samples(6,60);samples(6,300);
    lap_vertices(right);
}
static void begin_exit(unsigned right) {
    begin_loop(right);samples(right?9:7,right?16:6);
}
int main(void) {
    unsigned right,i,k;
    static const unsigned left_entry[]={3,5,7,9,11,13,15};
    static const unsigned right_entry[]={9,10,11,12,13,14,15};
    for(right=0;right<2;++right) {
        const unsigned *entry=right?right_entry:left_entry;
        for(i=0;i<7;++i) {
            Irtracking_Reset();Irtracking_Resume(60,60);mask=entry[i];
            Roundabout_StartCourse((uint8_t)right);samples(entry[i],3);
            check(Roundabout_GetDisplay()==11&&!stopped(),"course can enter a partial junction without extra centered arming");
        }
        mask=6;Roundabout_Start((uint8_t)right);samples(6,30);samples(right?8:1,40);
        check(Roundabout_GetDisplay()==10&&!stopped(),"a lone outer reading still steers without launching the circle");
        samples(right?14:7,2);samples(6,1);samples(right?14:7,2);
        check(Roundabout_GetDisplay()==10,"entry still needs consecutive branch confirmation");
        samples(right?14:7,1);check(Roundabout_GetDisplay()==11,"fresh third branch reading enters the circle");
        samples(0,40);check(Roundabout_GetDisplay()==11&&!stopped(),"white during entry preserves unfinished entry and keeps turning");
        samples(6,3);samples(6,60);check(Roundabout_GetDisplay()==12,"found line continues the remaining entry instead of restarting route");
        begin_loop(right);samples(0,70000);
        check(Roundabout_GetDisplay()==12&&!stopped(),"circle searches continuously beyond the old loss deadline and counter range");
        check((pwm[1]==72&&pwm[3]==36)||(pwm[1]==36&&pwm[3]==72),"circle search keeps the last rotation with one forward wheel");
        samples(6,1);samples(0,1);samples(6,2);
        check(Roundabout_GetDisplay()==12,"isolated black evidence does not advance circle state");
        samples(6,1);samples(6,30);samples(right?9:7,right?16:6);
        check(Roundabout_GetDisplay()==13,"search recovery still reaches the same circle exit");
        if(right)
        {
            samples(6,ROUND_RIGHT_EXIT_RUN_TICKS+5);
            check(Roundabout_GetDisplay()==0&&stopped(),
                  "right exit drives straight out and stops");
        }
        else
        {
            samples(6,ROUND_LEFT_EXIT_MIN_TICKS);
            check(Roundabout_GetDisplay()==14,"visible outgoing line completes circle exit");
            begin_exit(right);samples(0,1000);
            check(Roundabout_GetDisplay()==13&&!stopped(),"exit white keeps searching and cannot falsely finish the circle");
            samples(6,ROUND_FOUND_LINE_TICKS+ROUND_LEFT_EXIT_MIN_TICKS);
            check(Roundabout_GetDisplay()==14,"late outgoing line resumes the existing exit");
        }
        begin_loop(right);samples(right?12:3,600);
        check(Roundabout_GetDisplay()==12&&!stopped(),"same-side polygon edges are not expanded into return junctions");
        mask=6;Roundabout_Start((uint8_t)right);samples(right?5:10,40);
        check(Roundabout_GetDisplay()==10&&!stopped(),"nonselected separated approach reading does not cause a failure stop");
    }
    begin_exit(0);samples(6,ROUND_EXIT_MIN_TICKS);
    check(Roundabout_GetDisplay()==13&&!stopped(),
          "left exit stays owned by ring when line is lost just past the old handover");
    samples(0,105);
    check(Roundabout_GetDisplay()==13&&!stopped(),
          "left exit white searches instead of tripping ordinary loss stop");
    samples(6,ROUND_FOUND_LINE_TICKS+ROUND_LEFT_EXIT_MIN_TICKS);
    check(Roundabout_GetDisplay()==14&&!stopped(),
          "recovered stable left outgoing line can finish the guarded exit");
    begin_exit(0);samples(6,ROUND_EXIT_MIN_TICKS);samples(2,10);
    check(Roundabout_GetDisplay()==13&&pwm[1]<pwm[3],
          "left exit follows an inner sensor offset before handing over");
    begin_exit(0);samples(6,ROUND_EXIT_MIN_TICKS);samples(4,10);
    check(Roundabout_GetDisplay()==13&&pwm[1]>pwm[3],
          "rightward correction before left exit loss establishes the adverse history");
    samples(0,30);
    check(Roundabout_GetDisplay()==13&&pwm[1]==36&&pwm[3]==72,
          "left exit white turns left even when the previous line correction was rightward");
    samples(6,2);samples(0,1);
    check(Roundabout_GetDisplay()==13&&pwm[1]==36&&pwm[3]==72,
          "unconfirmed black sightings keep the leftward search active");
    samples(6,ROUND_FOUND_LINE_TICKS+ROUND_LEFT_EXIT_MIN_TICKS);
    check(Roundabout_GetDisplay()==14&&pwm[1]==72&&pwm[3]==72,
          "confirmed outgoing line resumes centered tracking after left search");
    begin_exit(0);samples(6,ROUND_EXIT_MIN_TICKS);samples(0,30);
    samples(1,ROUND_FOUND_LINE_TICKS+25);
    check(Roundabout_GetDisplay()==13&&pwm[1]==60&&pwm[3]==84,
          "left outer sensor reacquisition steers toward the outgoing line");
    samples(6,ROUND_LEFT_EXIT_MIN_TICKS);
    check(Roundabout_GetDisplay()==14&&!stopped(),
          "outer-first reacquisition later hands over on stable inner line");
    begin_exit(0);samples(6,ROUND_LEFT_EXIT_MIN_TICKS);
    samples(0,105);
    check(Roundabout_GetDisplay()==14&&pwm[1]==36&&pwm[3]==72,
          "standalone left ring keeps turning left when outgoing observation loses the line");
    samples(6,2);samples(0,1);
    check(Roundabout_GetDisplay()==14&&pwm[1]==36&&pwm[3]==72,
          "standalone outgoing search ignores unconfirmed black sightings");
    samples(6,ROUND_FOUND_LINE_TICKS+30);
    check(Roundabout_GetDisplay()==14&&pwm[1]==72&&pwm[3]==72,
          "standalone outgoing search resumes tracking on confirmed black line");
    for(right=0;right<2;++right) {
        Irtracking_Reset();Irtracking_Resume(60,60);mask=6;
        Roundabout_StartCourse((uint8_t)right);samples(6,40);samples(0,3);
        check(pwm[1]>0&&pwm[3]>0&&Roundabout_GetDisplay()==10,
              "ring approach uses ordinary two-wheel correction without a rotation policy");
        samples(0,1000);
        check(!stopped()&&Roundabout_GetDisplay()==10&&
              (right?(pwm[1]==72&&!pwm[3]):(!pwm[1]&&pwm[3]==72)),
              "ring approach keeps searching instead of tripping the ordinary loss stop");
        begin_exit(right);samples(6,20);samples(right?8:1,40);samples(0,40);
        if(!right)
        {
            check(Roundabout_GetDisplay()==13&&!stopped(),
                  "left exit outer-only line cannot hand over before outgoing line is confirmed");
            samples(6,ROUND_FOUND_LINE_TICKS+ROUND_LEFT_EXIT_MIN_TICKS);
            samples(0,40);
            check(Roundabout_GetDisplay()==14&&pwm[1]==36&&pwm[3]==72,
                  "left outgoing observation keeps left search after handover");
            samples(0,1000);
            check(Roundabout_GetDisplay()==14&&!stopped(),
                  "left outgoing search does not spend the ordinary loss timeout");
            samples(6,ROUND_FOUND_LINE_TICKS+30);
            check(Roundabout_GetDisplay()==14&&pwm[1]==72&&pwm[3]==72,
                  "left outgoing observation resumes centered tracking");
        }
        else
        {
            check(Roundabout_GetDisplay()==13&&!stopped(),
                  "right exit stays in the straight run");
            samples(6,ROUND_RIGHT_EXIT_RUN_TICKS+5);
            check(Roundabout_GetDisplay()==0&&stopped(),
                  "right exit stops after the fixed straight run");
        }
    }
    for(right=0;right<2;++right) {
        begin_loop(right);samples(right?2:8,40);samples(0,1000);
        check((pwm[1]==72&&pwm[3]==36)||(pwm[1]==36&&pwm[3]==72),
              "ring white searches to the right instead of an over-corrected side");
        samples(6,3);samples(6,40);
        check(!Irtracking_HasFault()&&pwm[1]==72&&pwm[3]==72,
              "long ring recovery does not spend the ordinary loss deadline");
        mask=6;Roundabout_Start((uint8_t)right);samples(6,30);
        samples(right?14:7,3);samples(6,60);samples(6,10);samples(0,1000);
        samples(6,3);samples(right?14:7,3);
        check(Roundabout_GetDisplay()==12,
              "white search time cannot satisfy the minimum visible ring lap interval");
        samples(6,300);lap_vertices(right);samples(right?9:7,right?16:6);
        check(Roundabout_GetDisplay()==13,"resumed visible lap time still reaches the exit");
        begin_loop(right);samples(0,1000);
        samples(2,1);samples(4,1);samples(6,1);
        for(i=0;i<150;++i)samples(right?(i%2?8u:4u):(i%2?2u:4u),1);
        check(Roundabout_GetDisplay()==12&&!stopped(),
              "changing valid masks after recovery cannot spend ordinary loss timeout in ring loop");
        samples(6,40);
        check(Roundabout_GetDisplay()==12&&!Irtracking_HasFault()&&pwm[1]==72&&pwm[3]==72,
              "stable line after mixed recovered masks resumes centered ring tracking");
    }
    begin_exit(1);samples(6,30);
    check(pwm[1]==72+ROUND_RIGHT_EXIT_LEAN&&pwm[3]==72-ROUND_RIGHT_EXIT_LEAN,
          "right ring exit applies a gentle right correction");
    begin_exit(0);samples(6,ROUND_LEFT_EXIT_MIN_TICKS);
    check(pwm[1]==72&&pwm[3]==72,"left ring exit keeps the straight handover");
    mask=6;Roundabout_Start(0);samples(6,30);
    samples(7,3);samples(6,60);samples(6,300);
    samples(1,3);samples(7,3);
    check(Roundabout_GetDisplay()==12&&!stopped(),
          "a polygon vertex just after entry is not the return junction");
    mask=6;Roundabout_Start(0);samples(6,30);
    samples(7,3);samples(6,60);samples(6,300);
    for(k=0;k<ROUND_LAP_SIDES-1;++k) { samples(6,ROUND_VERTEX_ARM_TICKS+2); samples(1,1); }
    samples(7,3);
    check(Roundabout_GetDisplay()==12&&!stopped(),
          "a junction one straight side short of the lap is ignored");
    samples(6,ROUND_VERTEX_ARM_TICKS+2);samples(1,1);
    samples(7,3);
    check(Roundabout_GetDisplay()==13,
          "the left ring junction is accepted once the full lap of sides is driven");
    /* Right ring: plain tracking; the exit cue is the line on X1/X2. */
    mask=6;Roundabout_Start(1);samples(6,30);samples(14,3);samples(6,60);samples(6,60);
    samples(9,20);
    check(Roundabout_GetDisplay()==12&&!stopped(),
          "right ring exit waits for the minimum lap time");
    samples(6,300);samples(8,3);
    check(Roundabout_GetDisplay()==12&&!stopped(),
          "right ring ignores right-side-only probes");
    samples(3,60);
    check(Roundabout_GetDisplay()==12&&!stopped(),
          "left-only line from an over-correction is not an exit cue");
    samples(9,ROUND_RIGHT_EXIT_CONFIRM_TICKS+8);
    check(Roundabout_GetDisplay()==13,
          "right ring exits when the left line appears while the ring line stays on the right");

    /* Right ring: normal line following with a reduced correction, no counting. */
    mask=6;Roundabout_Start(1);samples(6,30);samples(14,3);samples(6,60);samples(6,160);
    samples(6,5);
    check(Roundabout_GetDisplay()==12&&pwm[1]==72&&pwm[3]==72,
          "right ring cruises straight on the centre line");
    samples(8,6);
    check(Roundabout_GetDisplay()==12&&
          pwm[1]==72+IRTRACKING_RIGHT_RING_MAX_DELTA&&
          pwm[3]==72-IRTRACKING_RIGHT_RING_MAX_DELTA,
          "right ring follows the line with a reduced correction");
    samples(6,10);
    check(pwm[1]==72&&pwm[3]==72,
          "right ring returns to straight once the line is back in the centre");
    samples(1,6);
    check(pwm[1]==72-IRTRACKING_RIGHT_RING_MAX_DELTA&&
          pwm[3]==72+IRTRACKING_RIGHT_RING_MAX_DELTA,
          "right ring steers left with the same reduced correction");
    samples(6,10);
    check(Roundabout_GetDisplay()==12,"right ring tracking never counts a lap");

    /* The exit cue only counts after the quiet single-line window. */
    mask=6;Roundabout_Start(1);samples(6,30);samples(14,3);samples(6,60);
    samples(9,5);
    check(Roundabout_GetDisplay()==12&&!stopped(),
          "an early exit cue burst is ignored before the quiet window");
    samples(6,320);
    samples(9,ROUND_RIGHT_EXIT_CONFIRM_TICKS+6);
    check(Roundabout_GetDisplay()==13,
          "exit cue counts once the quiet window is met");
    for(right=0;right<1;++right) {
        begin_loop(right);
        samples(6,ROUND_VERTEX_ARM_TICKS+5);
        
        
        samples(right?8u:1u,3);
        check(right?(pwm[1]==83&&pwm[3]==69):(pwm[1]==69&&pwm[3]==83),
              "ring vertex assist speeds up the outer wheel only");
        samples(6,ROUND_VERTEX_BOOST_TICKS+5);
        check(pwm[1]==72&&pwm[3]==72,"outer-wheel assist expires after its short window");
        samples(6,ROUND_VERTEX_ARM_TICKS+5);
        samples(right?8u:1u,3);
        check(right?(pwm[1]==83&&pwm[3]==69):(pwm[1]==69&&pwm[3]==83),
              "the next straight re-arms the ring vertex assist");
    }
    Irtracking_Reset();mask=6;for(i=0;i<40;++i)Irtracking_Update();
    mask=0;for(i=0;i<3;++i)Irtracking_Update();
    check(stopped()&&!Irtracking_HasFault(),"ordinary white without direction coasts before choosing a search side");
    mask=6;for(i=0;i<40;++i)Irtracking_Update();
    check(pwm[1]==72&&pwm[3]==72,"line returns before ordinary deadline can resume cruise");
    printf("%d checks, %d failures\n",checks,failures);return failures?1:0;
}
