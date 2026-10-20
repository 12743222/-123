#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <stdint.h>

#define CAL_TICK_MS          5u
#define CAL_DEBOUNCE_TICKS   4u
#define CAL_LONG_PRESS_TICKS 160u
#define CAL_WAIT_TICKS       400u
#define CAL_PWM_STEP         5u

/* First break (mode 3) actions, nominal 5ms ticks. Read black from the start of
   STRAIGHT, excluding the initial arc. The second arc feeds directly into
   tracking. Zero PWM means coasting, not measured standstill or braking. */
#define CAL_GAP_ARC1_TICKS          200u  /* 1.00s right arc, 76/59 */
#define CAL_GAP_ARC1_LEFT             76u  /* first-break right arc, slightly stronger */
#define CAL_GAP_ARC1_RIGHT            59u
#define CAL_GAP_STRAIGHT_MAX_TICKS  600u  /* no confirmed black by 3.00s: abort */
#define CAL_GAP_LINE_SAMPLES          2u  /* consecutive samples: either black */
#define CAL_GAP_SETTLE_TICKS         40u  /* 0.20s zero PWM before second right arc */
#define CAL_GAP_EXIT_ARC_TICKS      100u  /* 0.50s; stronger right arc, angle uncalibrated */
#define CAL_GAP_EXIT_ARC_LEFT        85u  /* both wheels forward */
#define CAL_GAP_EXIT_ARC_RIGHT       50u
#define CAL_GAP_STRAIGHT_PWM         60u

/* Second break (mode 4): one continuous right arc over 3.00s. The left/right
   pair keeps the same average speed as the old 75/60 but a smaller differential
   (11 instead of 15), so the car travels the same distance with less rotation.
   Reduce the difference further if it still turns too much; raise it if short. */
#define CAL_GAP2_ARC_TICKS          600u  /* 3.00s right arc */
#define CAL_GAP2_ARC_LEFT            72u
#define CAL_GAP2_ARC_RIGHT           63u

#if (CAL_GAP_ARC1_TICKS < 1 || CAL_GAP_STRAIGHT_MAX_TICKS < 1 || \
     CAL_GAP_LINE_SAMPLES < 1 || CAL_GAP_LINE_SAMPLES > 255 || \
     CAL_GAP_STRAIGHT_MAX_TICKS + 1 < CAL_GAP_LINE_SAMPLES || \
     CAL_GAP_ARC1_TICKS + CAL_GAP_STRAIGHT_MAX_TICKS > 6000 || \
     CAL_GAP_SETTLE_TICKS < 1 || CAL_GAP_SETTLE_TICKS > 6000 || \
     CAL_GAP_EXIT_ARC_TICKS < 1 || CAL_GAP_EXIT_ARC_TICKS >= CAL_GAP_ARC1_TICKS || \
     CAL_GAP_EXIT_ARC_RIGHT < 1 || CAL_GAP_EXIT_ARC_LEFT > 100 || \
     CAL_GAP_EXIT_ARC_LEFT <= CAL_GAP_EXIT_ARC_RIGHT || \
     CAL_GAP_STRAIGHT_PWM < 1 || CAL_GAP_STRAIGHT_PWM > 100 || \
     CAL_GAP2_ARC_TICKS < 1 || CAL_GAP2_ARC_TICKS > 6000 || \
     CAL_GAP2_ARC_RIGHT < 1 || CAL_GAP2_ARC_LEFT > 100 || \
     CAL_GAP_ARC1_RIGHT < 1 || CAL_GAP_ARC1_LEFT > 100 || \
     CAL_GAP_ARC1_LEFT <= CAL_GAP_ARC1_RIGHT || \
     CAL_GAP2_ARC_LEFT <= CAL_GAP2_ARC_RIGHT)
#error Invalid gap detection or motion parameters
#endif

void Calibration_Init(void);
void Calibration_Tick(uint8_t pressed);
uint8_t Calibration_GetDisplay(void);
#endif
