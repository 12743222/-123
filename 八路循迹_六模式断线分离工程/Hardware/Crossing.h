#ifndef CROSSING_H
#define CROSSING_H
#include <stdint.h>

/* Isolated single-intersection trial; all durations are nominal 5ms ticks.
   These limits need track validation and are not distance measurements. */
#define CROSS_CONFIRM_TICKS       3u
#define CROSS_CLEAR_TICKS         3u
#define CROSS_MIN_TICKS          20u
#define CROSS_MAX_TICKS         200u
#define CROSS_APPROACH_MAX_TICKS 2000u
#define CROSS_AFTER_TICKS        400u

#if (CROSS_CONFIRM_TICKS < 1 || CROSS_CONFIRM_TICKS > 255 || \
     CROSS_CLEAR_TICKS < 1 || CROSS_CLEAR_TICKS > 255 || \
     CROSS_MIN_TICKS < 1 || CROSS_MAX_TICKS <= CROSS_MIN_TICKS || \
     CROSS_MAX_TICKS > 6000 || CROSS_APPROACH_MAX_TICKS < 1 || \
     CROSS_APPROACH_MAX_TICKS > 6000 || CROSS_AFTER_TICKS < 1 || \
     CROSS_AFTER_TICKS > 6000)
#error Invalid single-crossing trial parameters
#endif

void Crossing_Start(void);
/* Returns display/status: 5 approach, 6 crossing, 7 exit, 0 done, 9 failed. */
uint8_t Crossing_Tick(void);
uint8_t Crossing_GetDisplay(void);
void Crossing_StartCourse(void);
#endif
