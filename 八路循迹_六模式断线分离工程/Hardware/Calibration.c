#include "Calibration.h"
#include "Course.h"
#include "Irtracking.h"
#include "Buzzer.h"

/* Six single-feature modes. Each successful run stops automatically. */
enum { READY, WAITING, COURSE_TEST, DONE, ABORTED };
static uint8_t state, selected;
static uint8_t button_candidate, button_stable, button_count;
static uint8_t release_required, released_ticks;
static uint16_t held_ticks, phase_ticks;

static void clear_button(void)
{
    button_candidate = button_stable = button_count = 0;
    held_ticks = released_ticks = 0;
    release_required = 1;
}

void Calibration_Init(void)
{
    state = READY;
    selected = 1;
    phase_ticks = 0;
    Irtracking_Reset();
    Buzzer_Stop();
    clear_button();
}

static void course_result(uint8_t result)
{
    if (result == 0) state = DONE;
    else if (result == 9)
    {
        Irtracking_Reset();
        state = ABORTED;
        Buzzer_Stop();
        clear_button();
    }
}

void Calibration_Tick(uint8_t pressed)
{
    Buzzer_Tick();
    pressed = pressed ? 1u : 0u;

    /* A raw press cancels either the countdown or an active run immediately. */
    if ((state == WAITING || state == COURSE_TEST) && pressed)
    {
        Irtracking_Reset();
        state = ABORTED;
        Buzzer_Stop();
        clear_button();
        return;
    }

    if (release_required)
    {
        if (pressed) released_ticks = 0;
        else if (++released_ticks >= CAL_DEBOUNCE_TICKS) release_required = 0;
        return;
    }

    if (state == WAITING)
    {
        if (++phase_ticks >= CAL_WAIT_TICKS)
        {
            Course_StartMode(selected);
            state = COURSE_TEST;
            course_result(Course_Tick());
        }
        return;
    }
    if (state == COURSE_TEST)
    {
        course_result(Course_Tick());
        return;
    }

    if (pressed != button_candidate)
    {
        button_candidate = pressed;
        button_count = 1;
    }
    else if (button_count < CAL_DEBOUNCE_TICKS) ++button_count;

    if (button_count >= CAL_DEBOUNCE_TICKS && button_stable != button_candidate)
    {
        button_stable = button_candidate;
        if (button_stable) held_ticks = 0;
        else
        {
            if (held_ticks >= CAL_LONG_PRESS_TICKS)
            {
                state = WAITING;
                phase_ticks = 0;
            }
            else
            {
                selected = selected == COURSE_MODE_COUNT ? 1 : selected + 1;
                state = READY;
            }
            held_ticks = 0;
        }
    }
    if (button_stable && held_ticks < CAL_LONG_PRESS_TICKS) ++held_ticks;
}

uint8_t Calibration_GetDisplay(void)
{
    if (state == WAITING || held_ticks >= CAL_LONG_PRESS_TICKS) return 8;
    if (state == COURSE_TEST) return Course_GetDisplay();
    if (state == DONE) return 0;
    if (state == ABORTED) return 9;
    return selected;
}
