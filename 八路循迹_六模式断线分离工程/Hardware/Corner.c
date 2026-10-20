#include "Corner.h"
#include "Irtracking.h"
#include "Course.h"
#include "robot.h"
#if CORNER_EXIT_PWM > IRTRACKING_STRAIGHT_SPEED
#error Corner exit speed must not exceed the normal tracking speed
#endif
enum { FINISHED=0, FAILED=9, APPROACH=10, AFTER=12, TURN=16, ALIGN=17, RECOVER=18 };
/* Outermost probe of each side; a branch cue needs one of them plus centre. */
#define CORNER_FAR_LEFT   0x01u /* X1 */
#define CORNER_FAR_RIGHT  0x80u /* X8 */
static uint8_t phase, right_turn, armed, cue_count, clear_count, line_cleared, line_count;
static uint8_t left_pwm, right_pwm;
static uint8_t left_exit_search, searching, found_count;
static uint16_t phase_ticks;
static uint8_t continuous, automatic, cue_side;
static uint8_t exit_ticks, white_count;
static uint8_t corners_done;
static uint8_t flipped, after_clear_seen;
static int8_t last_side;

/* Re-arm the approach after one corner so mode 2 can take the next one. */
static void next_corner(void)
{
    phase=APPROACH;
    phase_ticks=armed=cue_count=clear_count=0;
    line_cleared=line_count=cue_side=0;
    white_count=flipped=after_clear_seen=0;
    searching=found_count=0;
}
static uint8_t finish(uint8_t result)
{
    if(result!=FINISHED || !continuous) Irtracking_Reset();
    phase=result;
    return phase;
}
static uint8_t ramp(uint8_t current,uint8_t target)
{
    if(current<target) return target-current>CORNER_PWM_STEP?current+CORNER_PWM_STEP:target;
    return current-target>CORNER_PWM_STEP?current-CORNER_PWM_STEP:target;
}
static void pivot(uint8_t target)
{
    if(right_turn) { left_pwm=ramp(left_pwm,target); right_pwm=0; }
    else { left_pwm=0; right_pwm=ramp(right_pwm,target); }
    robot_speed(left_pwm,0,right_pwm,0);
}
static uint8_t center_line(uint8_t sample)
{
    return Irtracking_IsInnerLine(sample) && (sample & IRTRACKING_CENTER_MASK);
}
/* Blend from the corner speed up to the normal tracking speed, so the handover
   from the manoeuvre to closed-loop following has no speed step. */
static uint8_t exit_speed(uint16_t elapsed) {
    uint16_t span=(uint16_t)(IRTRACKING_STRAIGHT_SPEED-CORNER_EXIT_PWM);
    if(elapsed>CORNER_EXIT_TICKS) elapsed=CORNER_EXIT_TICKS;
    return (uint8_t)(CORNER_EXIT_PWM+(uint16_t)elapsed*span/CORNER_EXIT_TICKS);
}
static void exit_straight(void)
{
    /* Remove the software yaw tail immediately instead of ramping one wheel from zero. */
    left_pwm=right_pwm=CORNER_EXIT_PWM;
    robot_speed(left_pwm,0,right_pwm,0);
}
static int8_t weighted_side(uint8_t sample)
{
    static const int8_t weights[8]={-7,-5,-3,-1,1,3,5,7};
    int16_t sum=0;
    uint8_t i;
    for(i=0;i<8;++i) if(sample & (1u<<i)) sum+=weights[i];
    if(sum<0) return -1;
    return sum>0?1:0;
}
static void commit_turn(void)
{
    Irtracking_Reset();
    /* The cue commits the turn. Do not coast while the front sensors move past
       the corner; a following white sample still pivots. */
    phase=TURN;
    phase_ticks=0;
    line_cleared=line_count=clear_count=white_count=0;
    flipped=after_clear_seen=0;
    left_pwm=right_pwm=0;
    pivot(CORNER_PIVOT_PWM);
}
static void start(uint8_t right, uint8_t course)
{
    continuous=course;
    automatic=cue_side=0;
    phase=APPROACH;
    right_turn=right?1:0;
    armed=cue_count=clear_count=line_cleared=line_count=left_pwm=right_pwm=0;
    left_exit_search=searching=found_count=0;
    exit_ticks=white_count=0;
    flipped=after_clear_seen=0;
    last_side=0;
    corners_done=0;
    phase_ticks=0;
    if(!continuous) Irtracking_Reset();
}
void Corner_Start(uint8_t right) { start(right,0); }
void Corner_StartAuto(void) { start(0,1); automatic=1; }
void Corner_StartCourse(uint8_t right) { start(right,1); }
void Corner_StartCourseAfterLeftRing(void)
{
    start(1,1);
    left_exit_search=1;
    corners_done=0;
}

uint8_t Corner_GetDisplay(void)
{
    if(phase==TURN||phase==ALIGN) return 11u;
    return phase==RECOVER?12u:phase;
}
uint8_t Corner_Tick(void)
{
    uint8_t sample, regions, cue;
    if(phase==FINISHED||phase==FAILED) return phase;
    sample=Irtracking_ReadState();
    regions=Irtracking_GetRegions(sample);
    if(phase==APPROACH)
    {
        if(left_exit_search && sample==0)
        {
            searching=1;
            found_count=0;
            Irtracking_SetSearchDirection(0);
            Irtracking_UpdateRingState(0);
            return Corner_GetDisplay();
        }
        if(searching)
        {
            if(++found_count<CORNER_LEFT_EXIT_FOUND_TICKS)
            {
                Irtracking_SetSearchDirection(0);
                Irtracking_UpdateRingState(0);
                return Corner_GetDisplay();
            }
            searching=found_count=0;
        }
        if(++phase_ticks>=(continuous?COURSE_APPROACH_MAX_TICKS:CORNER_APPROACH_MAX_TICKS)) return finish(FAILED);
        if(automatic)
        {
            if(sample) { int8_t side=weighted_side(sample); if(side) last_side=side; }
            /* The outermost probe on one side plus central evidence marks a
               branch. Requiring the far probe keeps X6/X7 or X2/X3 offset
               stripes on a straight from committing a turn. */
            cue=0;
            if(sample & IRTRACKING_CENTER_MASK)
            {
                if((sample&CORNER_FAR_LEFT) && !(sample&IRTRACKING_RIGHT_OUTER_MASK)) cue=1;
                else if((sample&CORNER_FAR_RIGHT) && !(sample&IRTRACKING_LEFT_OUTER_MASK)) cue=2;
            }
            if(cue)
            {
                if(cue_side!=cue) { cue_count=0; cue_side=cue; }
                right_turn=cue==2;
            }
            else
            {
                cue_side=cue_count=0;
                if((sample&IRTRACKING_LEFT_OUTER_MASK) && (sample&IRTRACKING_RIGHT_OUTER_MASK))
                {
                    /* An ambiguous junction does not supply a direction. */
                    clear_count=0;
                    white_count=0;
                    Irtracking_Forward();
                    return Corner_GetDisplay();
                }
                /* Missed cue: walking straight through a right-angle corner
                   leaves the bar on white. Pivot towards the side the line was
                   last seen on, defaulting to the right branch. */
                if(armed && sample==0)
                {
                    if(++white_count>=CORNER_WHITE_COMMIT_TICKS)
                    {
                        right_turn = last_side<0 ? 0u : 1u;
                        commit_turn();
                        return Corner_GetDisplay();
                    }
                }
                else white_count=0;
            }
        }
        else cue=(regions & (right_turn?8u:1u)) && (sample & IRTRACKING_CENTER_MASK);
        if(cue)
        {
            if(!armed) return finish(FAILED);
            Irtracking_Forward();
            if(++cue_count>=CORNER_CONFIRM_TICKS) commit_turn();
        }
        else
        {
            cue_count=0;
            if(Irtracking_IsInnerLine(sample) && (!automatic || (sample&IRTRACKING_CENTER_MASK)))
            {
                if(clear_count<CORNER_CONFIRM_TICKS) ++clear_count;
                if(clear_count>=CORNER_CONFIRM_TICKS) armed=1;
            }
            else clear_count=0;
            Irtracking_UpdateState(sample);
            if(Irtracking_HasFault()) return finish(FAILED);
        }
    }
    else if(phase==TURN||phase==ALIGN)
    {
        if(++phase_ticks>=CORNER_TURN_MAX_TICKS) return finish(FAILED);
        if(sample==0)
        {
            if(white_count<255) ++white_count;
        }
        else white_count=0;
        if((sample & IRTRACKING_INNER_MASK)==0)
        {
            if(clear_count<CORNER_CLEAR_TICKS) ++clear_count;
            if(clear_count>=CORNER_CLEAR_TICKS) line_cleared=1;
        }
        else clear_count=0;
        if(line_cleared && sample!=0) { phase=ALIGN; after_clear_seen=1; }
        /* Committed the wrong way round: the pivot never reaches the outgoing
           line, so the bar stays white far longer than the gap between the two
           lines. Flip the pivot once and give the other side a fresh budget. */
        if(!flipped && line_cleared && !after_clear_seen && white_count>=CORNER_FLIP_WHITE_TICKS)
        {
            flipped=1;
            right_turn=right_turn?0u:1u;
            line_cleared=line_count=clear_count=white_count=0;
            left_pwm=right_pwm=0;
            phase_ticks=0;
            pivot(CORNER_PIVOT_PWM);
            return Corner_GetDisplay();
        }
        if(line_cleared && center_line(sample))
        {
            exit_straight();
            if(line_count<CORNER_CONFIRM_TICKS) ++line_count;
            if(phase_ticks>=CORNER_TURN_MIN_TICKS && line_count>=CORNER_CONFIRM_TICKS)
            {
                phase=RECOVER;
                exit_ticks=white_count=line_count=0;
                Irtracking_Resume(left_pwm,right_pwm);
                Irtracking_UpdateStateAtSpeed(sample,exit_speed(0));
            }
        }
        else
        {
            line_count=0;
            pivot(phase==ALIGN?CORNER_ALIGN_PWM:CORNER_PIVOT_PWM);
        }
    }
    else if(phase==RECOVER)
    {
        if(++phase_ticks>=CORNER_TURN_MAX_TICKS) return finish(FAILED);
        if(sample==0)
        {
            if(++white_count>=CORNER_CONFIRM_TICKS)
            {
                phase=ALIGN;
                line_count=exit_ticks=0;
                pivot(CORNER_ALIGN_PWM);
                return Corner_GetDisplay();
            }
        }
        else white_count=0;
        if(center_line(sample))
        {
            if(line_count<CORNER_CONFIRM_TICKS) ++line_count;
        }
        else line_count=0;
        if(sample==IRTRACKING_ALL_BLACK)
        {
            exit_straight();
            Irtracking_Resume(left_pwm,right_pwm);
        }
        else Irtracking_UpdateStateAtSpeed(sample,exit_speed(exit_ticks));
        if(Irtracking_HasFault()) return finish(FAILED);
        if(exit_ticks<CORNER_EXIT_TICKS) ++exit_ticks;
        if(exit_ticks>=CORNER_EXIT_TICKS && line_count>=CORNER_CONFIRM_TICKS)
        {
            if(continuous && automatic)
            {
                /* Mode 2: chain the requested number of right-angle corners. */
                if(++corners_done>=CORNER_COURSE_CORNERS) return finish(FINISHED);
                next_corner();
                return Corner_GetDisplay();
            }
            phase=AFTER;
            phase_ticks=0;
            if(continuous) return finish(FINISHED);
        }
    }
    else
    {
        if(sample==IRTRACKING_ALL_BLACK) return finish(FAILED);
        if(++phase_ticks>=CORNER_AFTER_TICKS) return finish(FINISHED);
        Irtracking_UpdateState(sample);
        if(Irtracking_HasFault()) return finish(FAILED);
    }
    return Corner_GetDisplay();
}
