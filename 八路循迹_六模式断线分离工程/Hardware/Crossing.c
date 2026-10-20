#include "Crossing.h"
#include "Irtracking.h"
#include "Course.h"
#include "Buzzer.h"

enum { FINISHED = 0, APPROACH = 5, PASSING = 6, AFTER = 7, FAILED = 9 };
static uint8_t phase, clear_count, black_count, armed;
static uint16_t phase_ticks;
static uint8_t continuous;

/* Wide stripe may miss one or two probes at a slanted approach. */
static uint8_t wide_stripe(uint8_t sample)
{
    uint8_t count=0, bits=sample;
    if(!(sample&IRTRACKING_LEFT_OUTER_MASK) || !(sample&IRTRACKING_RIGHT_OUTER_MASK) ||
       !(sample&IRTRACKING_CENTER_MASK)) return 0;
    while(bits) { count+=(bits&1u); bits>>=1; }
    return count>=6;
}

static uint8_t finish(uint8_t result)
{
    if(result!=FINISHED || !continuous) Irtracking_Reset();
    phase = result;
    return phase;
}

static void start(uint8_t course)
{
    continuous=course;
    phase = APPROACH;
    phase_ticks = 0;
    clear_count = black_count = armed = 0;
    if(!continuous) Irtracking_Reset();
}
void Crossing_Start(void) { start(0); }
void Crossing_StartCourse(void) { start(1); }


uint8_t Crossing_Tick(void)
{
    uint8_t sample;
    if (phase == FINISHED || phase == FAILED) return phase;
    sample = Irtracking_ReadState();

    if (phase == APPROACH)
    {
        if (phase_ticks >= (continuous?COURSE_APPROACH_MAX_TICKS:CROSS_APPROACH_MAX_TICKS)) return finish(FAILED);
        ++phase_ticks;
        if (wide_stripe(sample))
        {
            /* Do not launch this test while already on an unknown feature. */
            if (!armed) return finish(FAILED);
            Irtracking_Forward();
            if (++black_count >= CROSS_CONFIRM_TICKS)
            {
                phase = PASSING;
                phase_ticks = 0;
                clear_count = 0;
                Buzzer_Beep();
            }
        }
        else
        {
            black_count = 0;
            if (Irtracking_IsInnerLine(sample) && (sample&IRTRACKING_CENTER_MASK))
            {
                if (clear_count < CROSS_CLEAR_TICKS) ++clear_count;
                if (clear_count >= CROSS_CLEAR_TICKS) armed = 1;
            }
            else clear_count=0;
            Irtracking_UpdateState(sample);
            if (Irtracking_HasFault()) return finish(FAILED);
        }
    }
    else if (phase == PASSING)
    {
        if (++phase_ticks >= CROSS_MAX_TICKS) return finish(FAILED);
        /* Keep the heading command straight even when one sensor leaves the
           transverse stripe before the other. No spin or reverse command. */
        Irtracking_Forward();
        if (Irtracking_IsInnerLine(sample) && (sample&IRTRACKING_CENTER_MASK))
        {
            if (clear_count < CROSS_CLEAR_TICKS) ++clear_count;
        }
        else clear_count=0;
        if (phase_ticks >= CROSS_MIN_TICKS && clear_count >= CROSS_CLEAR_TICKS)
        {
            phase = AFTER;
            phase_ticks = 0;
            if(continuous) return finish(FINISHED);
        }
    }
    else
    {
        /* One crossing per run: do not treat another black feature as a
           second intersection, roundabout or finish without route context. */
        if (wide_stripe(sample)) return finish(FAILED);
        if (++phase_ticks >= CROSS_AFTER_TICKS) return finish(FINISHED);
        Irtracking_UpdateState(sample);
        if (Irtracking_HasFault()) return finish(FAILED);
    }
    return phase;
}

uint8_t Crossing_GetDisplay(void)
{
    return phase;
}
