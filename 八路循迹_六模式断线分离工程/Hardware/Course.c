#include "Course.h"
#include "Corner.h"
#include "Crossing.h"
#include "Roundabout.h"
#include "Gap.h"
#include "Irtracking.h"
#include "Calibration.h"
#include "Buzzer.h"

static uint8_t selected, status, gap_recover, center_count;
static uint16_t elapsed, approach_ticks, recovery_ticks;
static uint8_t finish(uint8_t result)
{
    Irtracking_Reset();
    if(result==9) Buzzer_Stop();
    status=result;
    return status;
}
void Course_StartMode(uint8_t mode)
{
    Irtracking_Reset();
    Buzzer_Stop();
    selected=mode;
    elapsed=approach_ticks=recovery_ticks=0;
    gap_recover=center_count=0;
    status=1;
    switch(mode)
    {
    case 1: Crossing_StartCourse(); break;
    case 2: Corner_StartAuto(); break;
    case 3: Gap_StartCourse(); break;
    case 4: Gap_StartCourseSecond(); break;
    case 5: Roundabout_StartIndependent(0); break;
    case 6: Roundabout_StartIndependent(1); break;
    default: finish(9); break;
    }
}
uint8_t Course_GetMode(void) { return selected; }
uint8_t Course_GetDisplay(void) { return status==1?selected:status; }
uint8_t Course_Tick(void)
{
    uint8_t result=1, phase=0, sample;
    if(status!=1) return status;
    if(++elapsed>=COURSE_MODE_MAX_TICKS) return finish(9);
    if(gap_recover)
    {
        /* The fixed gap arcs retain their tuning; only the final handover
           waits for a real central line at the original straight speed. */
        sample=Irtracking_ReadState();
        if(Irtracking_IsInnerLine(sample) && (sample&IRTRACKING_CENTER_MASK))
        {
            if(center_count<3) ++center_count;
        }
        else center_count=0;
        Irtracking_UpdateStateAtSpeed(sample,CAL_GAP_STRAIGHT_PWM);
        if(Irtracking_HasFault()) return finish(9);
        ++recovery_ticks;
        if(recovery_ticks>=COURSE_GAP_RECOVER_TICKS && center_count>=3) return finish(0);
        if(recovery_ticks>=COURSE_GAP_RECOVER_MAX_TICKS) return finish(9);
        return status;
    }
    switch(selected)
    {
    case 1: phase=Crossing_GetDisplay(); result=Crossing_Tick(); break;
    case 2: phase=Corner_GetDisplay(); result=Corner_Tick(); break;
    case 3: phase=Gap_GetDisplay(); result=Gap_Tick(); break;
    case 4: phase=Gap_GetDisplay(); result=Gap_Tick(); break;
    default: phase=Roundabout_GetDisplay(); result=Roundabout_Tick(); break;
    }
    if(result==9) return finish(9);
    if(result==GAP_REACHED_FINISH) return finish(0);
    if(result==0)
    {
        if(selected==3) { gap_recover=1; return status; }
        return finish(0);
    }
    if(phase==(selected==1?5u:10u))
    {
        if(++approach_ticks>=COURSE_MODE_APPROACH_TICKS) return finish(9);
    }
    return status;
}
