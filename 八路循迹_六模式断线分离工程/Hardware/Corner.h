#ifndef CORNER_H
#define CORNER_H
#include <stdint.h>
/* Nominal 5ms ticks; initial physical-test settings, not measured angles.
   V2 corner strategy:
   - a branch cue only needs the outermost probe on one side plus central
     evidence instead of both outer probes, so partial branch patterns commit;
   - if the cue is missed and the bar ends up on white, the pivot falls back to
     the side the line was last seen on;
   - a committed turn that sees the entry line clear but never finds the
     outgoing line flips once to the other side before failing. */
#define CORNER_CONFIRM_TICKS        3u
#define CORNER_LEFT_EXIT_FOUND_TICKS 3u
#define CORNER_CLEAR_TICKS          3u
#define CORNER_TURN_MIN_TICKS      12u
#define CORNER_TURN_MAX_TICKS     500u /* per direction; a flip restarts the budget */
#define CORNER_PIVOT_PWM           68u
#define CORNER_ALIGN_PWM           48u
#define CORNER_EXIT_PWM            54u
#define CORNER_EXIT_TICKS          60u /* 0.30s blend window: the exit speed ramps up to normal tracking */
#define CORNER_COURSE_CORNERS        5u /* mode 2: how many right-angle corners to run in a row */
#define CORNER_PWM_STEP             6u
#define CORNER_APPROACH_MAX_TICKS 2000u
#define CORNER_AFTER_TICKS        400u
#define CORNER_WHITE_COMMIT_TICKS  12u /* 60ms white after arming commits a missed corner */
#define CORNER_FLIP_WHITE_TICKS   220u /* 1.1s white after the entry line cleared flips the pivot */
#if (CORNER_CONFIRM_TICKS < 1 || CORNER_CONFIRM_TICKS > 255 || \
     CORNER_LEFT_EXIT_FOUND_TICKS < 1 || CORNER_LEFT_EXIT_FOUND_TICKS > 255 || \
     CORNER_CLEAR_TICKS < 1 || CORNER_CLEAR_TICKS > 255 || \
     CORNER_TURN_MIN_TICKS < 1 || CORNER_TURN_MAX_TICKS <= CORNER_TURN_MIN_TICKS || \
     CORNER_TURN_MAX_TICKS > 6000 || CORNER_PIVOT_PWM < 1 || CORNER_PIVOT_PWM > 100 || \
     CORNER_ALIGN_PWM < 1 || CORNER_ALIGN_PWM >= CORNER_PIVOT_PWM || \
     CORNER_EXIT_PWM < 1 || CORNER_EXIT_PWM > 100 || \
     CORNER_EXIT_TICKS < CORNER_CONFIRM_TICKS || CORNER_EXIT_TICKS > 255 || \
     CORNER_COURSE_CORNERS < 1 || CORNER_COURSE_CORNERS > 9 || \
     CORNER_PWM_STEP < 1 || CORNER_PWM_STEP > 100 || \
     CORNER_APPROACH_MAX_TICKS < 1 || CORNER_APPROACH_MAX_TICKS > 6000 || \
     CORNER_AFTER_TICKS < 1 || CORNER_AFTER_TICKS > 6000 || \
     CORNER_WHITE_COMMIT_TICKS < 1 || CORNER_WHITE_COMMIT_TICKS > 255 || \
     CORNER_FLIP_WHITE_TICKS <= CORNER_CLEAR_TICKS || \
     CORNER_FLIP_WHITE_TICKS >= CORNER_TURN_MAX_TICKS)
#error Invalid corner trial parameters
#endif
void Corner_Start(uint8_t right);
/* Infer a single left/right branch from the full eight-bit pattern. */
void Corner_StartAuto(void);
uint8_t Corner_Tick(void);
uint8_t Corner_GetDisplay(void); /* A approach, b turn, C outgoing, 0 done, 9 failed */
void Corner_StartCourse(uint8_t right);
/* Only the right corner immediately after the left ring may recover white by turning left. */
void Corner_StartCourseAfterLeftRing(void);
#endif
