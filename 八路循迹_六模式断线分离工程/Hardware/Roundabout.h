#ifndef ROUNDABOUT_H
#define ROUNDABOUT_H
#include <stdint.h>
/* Single shared entrance/exit junction. CCW/left or CW/right are selected
   before starting; route order is supplied by Course, not inferred from sensors. */
#define ROUND_CONFIRM_TICKS          3u
#define ROUND_ENTRY_TICKS           60u
#define ROUND_DEPART_CLEAR_TICKS    10u
#define ROUND_LAP_MIN_TICKS        300u
#define ROUND_LAP_MAX_TICKS       6000u
#define ROUND_APPROACH_MAX_TICKS  2000u
#define ROUND_EXIT_MIN_TICKS        20u /* left exit switches to tracking after this */
#define ROUND_RIGHT_EXIT_RUN_TICKS 200u /* 1.0s straight run out of the right ring, then stop */
#define ROUND_RIGHT_EXIT_LEAN        5u /* gentle right correction while leaving the right ring */
#define ROUND_RIGHT_EXIT_CONFIRM_TICKS 10u /* left+right line must persist before the exit cue */
#define ROUND_RIGHT_ARM_QUIET_TICKS    6u /* single-line ticks required before an exit cue counts */
#define ROUND_LEFT_EXIT_MIN_TICKS   80u
#define ROUND_LEFT_EXIT_LINE_TICKS  10u
#define ROUND_EXIT_MAX_TICKS       400u
#define ROUND_AFTER_TICKS          400u
#define ROUND_VERTEX_ARM_TICKS      15u /* centred side samples arm the next-side assist */
#define ROUND_VERTEX_BOOST_TICKS    20u /* 100ms outer-wheel boost at a polygon vertex */
#define ROUND_LAP_SIDES              7u /* straight sides that must be driven before the exit */
#define ROUND_LAP_FALLBACK_TICKS  3000u /* accept the junction after this anyway */
#define ROUND_FOUND_LINE_TICKS 3u /* resume only after consecutive black evidence */
#if (ROUND_FOUND_LINE_TICKS < 1 || ROUND_FOUND_LINE_TICKS > 255 || \
     ROUND_CONFIRM_TICKS < 1 || ROUND_CONFIRM_TICKS > 255 || \
     ROUND_ENTRY_TICKS < 1 || ROUND_ENTRY_TICKS > 6000 || \
     ROUND_DEPART_CLEAR_TICKS < 1 || ROUND_DEPART_CLEAR_TICKS > 255 || \
     ROUND_LAP_MIN_TICKS < 1 || ROUND_LAP_MAX_TICKS <= ROUND_LAP_MIN_TICKS || \
     ROUND_LAP_MAX_TICKS > 6000 || ROUND_APPROACH_MAX_TICKS < 1 || ROUND_APPROACH_MAX_TICKS > 6000 || \
     ROUND_EXIT_MIN_TICKS < 1 || ROUND_LEFT_EXIT_MIN_TICKS <= ROUND_EXIT_MIN_TICKS || \
     ROUND_EXIT_MAX_TICKS <= ROUND_LEFT_EXIT_MIN_TICKS || \
     ROUND_RIGHT_EXIT_RUN_TICKS < 1 || ROUND_RIGHT_EXIT_RUN_TICKS >= ROUND_EXIT_MAX_TICKS || \
     ROUND_RIGHT_EXIT_LEAN < 1 || ROUND_RIGHT_EXIT_LEAN > 20 || \
     ROUND_RIGHT_EXIT_CONFIRM_TICKS < ROUND_CONFIRM_TICKS || ROUND_RIGHT_EXIT_CONFIRM_TICKS > 255 || \
     ROUND_RIGHT_ARM_QUIET_TICKS < 1 || ROUND_RIGHT_ARM_QUIET_TICKS > 6000 || \
     ROUND_LEFT_EXIT_LINE_TICKS < ROUND_CONFIRM_TICKS || ROUND_LEFT_EXIT_LINE_TICKS > 255 || \
     ROUND_EXIT_MAX_TICKS > 6000 || ROUND_AFTER_TICKS < 1 || ROUND_AFTER_TICKS > 6000 || \
     ROUND_VERTEX_ARM_TICKS < 1 || ROUND_VERTEX_ARM_TICKS > 255 || \
     ROUND_VERTEX_BOOST_TICKS < 1 || ROUND_VERTEX_BOOST_TICKS > 255 || \
     ROUND_LAP_SIDES < 1 || ROUND_LAP_SIDES > 255 || \
     ROUND_LAP_FALLBACK_TICKS < ROUND_LAP_MIN_TICKS || ROUND_LAP_FALLBACK_TICKS > 6000)
#error Invalid roundabout trial parameters
#endif
void Roundabout_Start(uint8_t clockwise);
/* Single-feature control: centered approach and centered exit are required. */
void Roundabout_StartIndependent(uint8_t clockwise);
uint8_t Roundabout_Tick(void);
uint8_t Roundabout_GetDisplay(void); /* A approach, b entry, C loop, d exit, E outgoing, 0 done, 9 failed */
void Roundabout_StartCourse(uint8_t right);
#endif
