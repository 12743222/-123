#include "Roundabout.h"
#include "Irtracking.h"
#include "Course.h"

enum { FINISHED=0, FAILED=9, APPROACH=10, ENTRY=11, LOOP=12, EXIT=13, AFTER=14 };
static uint8_t phase, clockwise, departed, cue_count, clear_count, continuous;
static uint8_t searching, found_count, exit_recovered, independent, armed, arm_count;
static uint8_t vertex_arm, vertex_clear, vertex_seen;
static uint8_t right_cue_armed;
static uint16_t right_quiet;
static uint16_t phase_ticks;

static uint8_t finish(uint8_t result)
{
    searching=found_count=exit_recovered=0;
    if(result!=FINISHED || !continuous) Irtracking_Reset();
    phase=result;
    return phase;
}
static uint8_t junction(uint8_t sample)
{
    uint8_t regions=Irtracking_GetRegions(sample);
    /* Return-junction rules stay narrow: polygon corners must not end a lap. */
    if(regions==9) return 1;
    if(clockwise) return (regions & 8u) && (regions & 2u);
    return (regions & 1u) && (regions & 4u);
}
static uint8_t entry_junction(uint8_t sample)
{
    uint8_t regions=Irtracking_GetRegions(sample);
    /* Same-side branches must reach the center: X2/X3 or X6/X7 alone
       can be an ordinary narrow line crossing a region boundary. */
    return junction(sample) ||
           ((regions & (clockwise?8u:1u)) && (sample & IRTRACKING_CENTER_MASK));
}
static uint8_t chosen_side(void) { return clockwise?0x80u:0x01u; }
static void track_ring(uint8_t sample)
{
    uint8_t regions=Irtracking_GetRegions(sample);
    if(regions==5 || regions==9 || regions==10 || regions==11 || regions==13)
        sample=chosen_side();
    if(phase==LOOP) Irtracking_UpdateRingStateGentle(sample);
    else Irtracking_UpdateState(sample);
}

static void start(uint8_t right, uint8_t course)
{
    continuous=course;
    independent=armed=arm_count=0;
    phase=APPROACH;
    clockwise=right?1:0;
    departed=cue_count=clear_count=searching=found_count=exit_recovered=0;
    vertex_arm=vertex_clear=vertex_seen=0;
    right_cue_armed=0;
    right_quiet=0;
    phase_ticks=0;
    if(!continuous) Irtracking_Reset();
    Irtracking_PresetSearchDirection(clockwise);
}
void Roundabout_Start(uint8_t right) { start(right,0); }
void Roundabout_StartCourse(uint8_t right) { start(right,1); }
void Roundabout_StartIndependent(uint8_t right) { start(right,1); independent=1; }

static uint8_t find_line(uint8_t sample)
{
    if(!searching && sample!=0) return 0;
    if(sample==0)
    {
        /* The left ring's outgoing line is to the car's left. Do not let a
           recent rightward sensor correction choose the search direction. */
        if((phase==EXIT || phase==AFTER) && !clockwise)
        {
            if(!searching) phase_ticks=0;
            exit_recovered=0;
            Irtracking_SetSearchDirection(0);
        }
        searching=1;
        found_count=0;
        cue_count=clear_count=0;
    }
    else if(++found_count>=ROUND_FOUND_LINE_TICKS)
    {
        searching=found_count=0;
        if(phase==EXIT && !clockwise) exit_recovered=1;
        return 0;
    }
    /* Only confirmed ring phases may pivot continuously on white. */
    Irtracking_UpdateRingState(0);
    return 1;
}
uint8_t Roundabout_GetDisplay(void) { return phase; }
uint8_t Roundabout_Tick(void)
{
    uint8_t sample, at_junction;
    if(phase==FINISHED||phase==FAILED) return phase;
    sample=Irtracking_ReadState();
    if((phase==ENTRY || phase==LOOP || phase==EXIT ||
        (phase==AFTER && !clockwise)) && find_line(sample))
    {
        if(Irtracking_HasFault()) return finish(FAILED);
        return phase;
    }
    at_junction=junction(sample);
    if(phase==APPROACH)
    {
        if(phase_ticks<(continuous?COURSE_APPROACH_MAX_TICKS:ROUND_APPROACH_MAX_TICKS)) ++phase_ticks;
        if(independent)
        {
            if(Irtracking_IsInnerLine(sample) && (sample&IRTRACKING_CENTER_MASK))
            {
                if(arm_count<ROUND_CONFIRM_TICKS) ++arm_count;
                if(arm_count>=ROUND_CONFIRM_TICKS) armed=1;
            }
            else arm_count=0;
        }
        if(entry_junction(sample) && (!independent || (sample&IRTRACKING_CENTER_MASK)))
        {
            if(independent && !armed) return finish(FAILED);
            /* A confirmed route branch can enter directly after a crossing;
               it does not require an extra centered-line arming interval. */
            Irtracking_Forward();
            if(++cue_count>=ROUND_CONFIRM_TICKS)
            {
                phase=ENTRY;
                phase_ticks=0;
                Irtracking_SetSearchDirection(clockwise);
                Irtracking_UpdateRingState(chosen_side());
            }
        }
        else
        {
            cue_count=0;
            track_ring(sample);
        }
    }
    else if(phase==ENTRY)
    {
        Irtracking_UpdateRingState(chosen_side());
        if(++phase_ticks>=ROUND_ENTRY_TICKS)
        {
            phase=LOOP;
            phase_ticks=0;
            clear_count=cue_count=departed=0;
            right_cue_armed=0;
            right_quiet=0;
        }
    }
    else if(phase==LOOP)
    {
        if(phase_ticks<ROUND_LAP_MAX_TICKS) ++phase_ticks;
        if(clockwise)
        {
            /* Right ring (simple logic): no polygon counting and no junction
               rule. The car just tracks the ring; at the exit junction the
               outgoing line shows up on the far-left probes X1/X2, and that is
               the cue to drive straight out. */
            if((sample & IRTRACKING_LEFT_OUTER_MASK) &&
               (sample & IRTRACKING_RIGHT_HALF_MASK))
            {
                /* The entrance junction also shows both sides (approach line on
                   the left, ring line on the right). Only a cue that follows a
                   long stretch of single line may be the exit. */
                if(right_cue_armed && phase_ticks>=ROUND_LAP_MIN_TICKS)
                {
                    if(++cue_count>=ROUND_RIGHT_EXIT_CONFIRM_TICKS)
                    {
                        phase=EXIT;
                        phase_ticks=0;
                        cue_count=clear_count=exit_recovered=0;
                        Irtracking_Forward();
                        return phase;
                    }
                }
                else cue_count=0;
                right_quiet=0;
            }
            else
            {
                cue_count=0;
                if(right_quiet<6000u) ++right_quiet;
                if(right_quiet>=ROUND_RIGHT_ARM_QUIET_TICKS) right_cue_armed=1;
            }
            Irtracking_UpdateRightRingState(sample);
        }
        else if(at_junction)
        {
            clear_count=0;
            vertex_clear=0;
            /* Every polygon vertex looks like a branch to the bar. Only accept
               the return junction after the car has driven the full set of
               straight sides (a whole lap), with a time fallback. */
            if(departed && phase_ticks>=ROUND_LAP_MIN_TICKS &&
               (vertex_seen>=ROUND_LAP_SIDES || phase_ticks>=ROUND_LAP_FALLBACK_TICKS))
            {
                if(++cue_count>=ROUND_CONFIRM_TICKS)
                {
                    phase=EXIT;
                    phase_ticks=0;
                    cue_count=clear_count=exit_recovered=0;
                    Irtracking_Forward();
                    return phase;
                }
            }
            else cue_count=0;
            Irtracking_UpdateRingState(chosen_side());
        }
        else
        {
            cue_count=0;
            if(clear_count<ROUND_DEPART_CLEAR_TICKS) ++clear_count;
            if(clear_count>=ROUND_DEPART_CLEAR_TICKS) departed=1;
            /* The ring is a polygon of seven straight sides. A side counts
               once the line has been centred long enough; when the line then
               bends towards the next side, briefly speed up the outer wheel
               so the vertex is taken smoothly instead of scrubbing. */
            if(Irtracking_IsInnerLine(sample) && (sample & IRTRACKING_CENTER_MASK))
            {
                if(vertex_clear<ROUND_VERTEX_ARM_TICKS)
                {
                    ++vertex_clear;
                    if(vertex_clear==ROUND_VERTEX_ARM_TICKS)
                    {
                        /* One straight side has been driven; count it once. */
                        if(vertex_seen<255) ++vertex_seen;
                        vertex_arm=1;
                    }
                }
            }
            else vertex_clear=0;
            if(vertex_arm &&
               ((clockwise && (sample & IRTRACKING_RIGHT_OUTER_MASK)) ||
                (!clockwise && (sample & IRTRACKING_LEFT_OUTER_MASK))))
            {
                Irtracking_SetRingBoost(clockwise, ROUND_VERTEX_BOOST_TICKS);
                vertex_arm=0;
                vertex_clear=0;
            }
            track_ring(sample);
        }
    }
    else if(phase==EXIT)
    {
        if(phase_ticks<ROUND_EXIT_MAX_TICKS) ++phase_ticks;
        if(clockwise)
        {
            /* Leaving the right ring: drive straight out with a small right
               correction and stop after a fixed run. Waiting for a centred
               X4/X5 line is unreliable here, so the run itself ends the mode. */
            Irtracking_ForwardLean(ROUND_RIGHT_EXIT_LEAN);
            if(phase_ticks>=ROUND_RIGHT_EXIT_RUN_TICKS) return finish(FINISHED);
        }
        else
        {
            /* Keep the left exit under ring control long enough to cross the
               junction. Once clear, let the inner probes correct the outgoing
               line; a lingering outer branch is not handover evidence. */
            if((exit_recovered && !at_junction) ||
               (phase_ticks>=ROUND_EXIT_MIN_TICKS &&
                Irtracking_IsInnerLine(sample)))
                Irtracking_UpdateState(sample);
            else Irtracking_Forward();
            if(at_junction || !Irtracking_IsInnerLine(sample) ||
               (independent && (!Irtracking_IsInnerLine(sample) || !(sample&IRTRACKING_CENTER_MASK))))
                clear_count=0;
            else if(clear_count<ROUND_LEFT_EXIT_LINE_TICKS)
                ++clear_count;
            if(phase_ticks>=ROUND_LEFT_EXIT_MIN_TICKS &&
               clear_count>=ROUND_LEFT_EXIT_LINE_TICKS)
            {
                phase=AFTER;
                phase_ticks=0;
                if(continuous) return finish(FINISHED);
            }
        }
    }
    else
    {
        if(sample==IRTRACKING_ALL_BLACK)
        {
            Irtracking_Forward();
            return phase;
        }
        if(++phase_ticks>=ROUND_AFTER_TICKS) return finish(FINISHED);
        Irtracking_UpdateState(sample);
    }
    if(Irtracking_HasFault()) return finish(FAILED);
    return phase;
}
