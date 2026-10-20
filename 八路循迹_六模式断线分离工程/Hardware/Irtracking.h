#ifndef __IRTRACKING_H
#define __IRTRACKING_H
#include "stm32f10x.h"
/* From the rear looking forward: X1, X2, ... X8 (confirmed mounting).
   GPIO output: LED on / black => LOW. All eight channels are enabled. */
#define IRTRACKING_X1_PIN GPIO_Pin_14
#define IRTRACKING_X2_PIN GPIO_Pin_12
#define IRTRACKING_X3_PIN GPIO_Pin_13
#define IRTRACKING_X4_PIN GPIO_Pin_15
#define IRTRACKING_X5_PIN GPIO_Pin_5 /* PB0 is reserved for the buzzer. */
#define IRTRACKING_X6_PIN GPIO_Pin_1
#define IRTRACKING_X7_PIN GPIO_Pin_10
#define IRTRACKING_X8_PIN GPIO_Pin_11
#define IRTRACKING_INPUT_PINS (IRTRACKING_X1_PIN | IRTRACKING_X2_PIN | \
    IRTRACKING_X3_PIN | IRTRACKING_X4_PIN | IRTRACKING_X5_PIN | \
    IRTRACKING_X6_PIN | IRTRACKING_X7_PIN | IRTRACKING_X8_PIN)
#define IRTRACKING_BLACK_LEVEL      0
#define IRTRACKING_ALL_BLACK     255u
#define IRTRACKING_CENTER_MASK   0x18u /* X4 + X5 */
#define IRTRACKING_INNER_MASK    0x3cu /* X3 through X6 */
#define IRTRACKING_LEFT_OUTER_MASK  0x03u /* X1 + X2 */
#define IRTRACKING_RIGHT_HALF_MASK  0xf0u /* X5 through X8: ring line still on the right */
#define IRTRACKING_LEFT_HALF_MASK   0x0fu /* X1 through X4 */
#define IRTRACKING_RIGHT_OUTER_MASK 0xc0u /* X7 + X8 */
#define IRTRACKING_PERIOD_MS        5
#define IRTRACKING_STRAIGHT_SPEED  72
#define IRTRACKING_STEER_GAIN       2
#define IRTRACKING_MAX_DELTA       12
#define IRTRACKING_RING_MAX_DELTA   9u /* gentler differential while cruising a ring */
#define IRTRACKING_RIGHT_RING_MAX_DELTA 6u /* even gentler correction for the right ring */
#define IRTRACKING_TURN_INNER (IRTRACKING_STRAIGHT_SPEED-IRTRACKING_MAX_DELTA)
#define IRTRACKING_TURN_OUTER (IRTRACKING_STRAIGHT_SPEED+IRTRACKING_MAX_DELTA)
#define IRTRACKING_PWM_STEP         3
#define IRTRACKING_FILTER_SAMPLES   3
#define IRTRACKING_SEARCH_DELAY_TICKS 20u /* 100ms coast before the search turn */
#define IRTRACKING_SEARCH_PWM      72
#define IRTRACKING_RING_SEARCH_INNER_PWM 36 /* ring search keeps both wheels moving (arc, not pivot) */
#define IRTRACKING_DEFAULT_SEARCH_RIGHT 1
#define IRTRACKING_BIAS_MAX (IRTRACKING_MAX_DELTA*8) /* Q3 fixed point */
#define IRTRACKING_RING_BOOST_PWM 8u /* extra PWM on the outer wheel at a ring vertex */
#if ((IRTRACKING_BLACK_LEVEL != 0 && IRTRACKING_BLACK_LEVEL != 1) || \
     IRTRACKING_STRAIGHT_SPEED <= 0 || IRTRACKING_STRAIGHT_SPEED > 100 || \
     IRTRACKING_STEER_GAIN < 1 || IRTRACKING_STEER_GAIN > 10 || \
     IRTRACKING_MAX_DELTA < 1 || IRTRACKING_MAX_DELTA >= IRTRACKING_STRAIGHT_SPEED || \
     IRTRACKING_RING_MAX_DELTA < 1 || IRTRACKING_RING_MAX_DELTA > IRTRACKING_MAX_DELTA || \
     IRTRACKING_RIGHT_RING_MAX_DELTA < 1 || IRTRACKING_RIGHT_RING_MAX_DELTA > IRTRACKING_MAX_DELTA || \
     IRTRACKING_TURN_INNER <= 0 || IRTRACKING_TURN_INNER >= IRTRACKING_TURN_OUTER || \
     IRTRACKING_TURN_OUTER > 100 || IRTRACKING_PWM_STEP < 1 || IRTRACKING_PWM_STEP > 100 || \
     IRTRACKING_FILTER_SAMPLES < 1 || IRTRACKING_FILTER_SAMPLES > 255 || \
     IRTRACKING_SEARCH_DELAY_TICKS < 1 || IRTRACKING_SEARCH_DELAY_TICKS > 6000 || \
     IRTRACKING_RING_BOOST_PWM < 1 || IRTRACKING_RING_BOOST_PWM > 30 || \
     IRTRACKING_SEARCH_PWM < 1 || IRTRACKING_SEARCH_PWM > 100 || \
     (IRTRACKING_DEFAULT_SEARCH_RIGHT != 0 && IRTRACKING_DEFAULT_SEARCH_RIGHT != 1))
#error Invalid eight-channel tracking parameters
#endif
/* Bit0 X1 leftmost through bit7 X8 rightmost; black is normalized to 1. */
uint8_t Irtracking_ReadState(void);
/* Route regions: bit0 X1/X2, bit1 X3/X4, bit2 X5/X6, bit3 X7/X8.
   Either probe activates its region; steering always uses the full eight bits. */
uint8_t Irtracking_GetRegions(uint8_t state);
uint8_t Irtracking_IsInnerLine(uint8_t state);
/* Steady differential (PWM units, positive = left faster) that the tracker
   needs to hold the line. Open-loop maneuvers reuse it as a motor-balance and
   line-offset trim while the probes see white. */
int8_t Irtracking_GetBias(void);
/* Ring vertex assist: for the next ticks speed up the outer wheel only.
   clockwise=1 boosts the left wheel, clockwise=0 boosts the right wheel. */
void Irtracking_SetRingBoost(uint8_t clockwise, uint8_t ticks);
/* Ring cruise tracking with a gentler steering limit for a smoother ride. */
void Irtracking_UpdateRingStateGentle(uint16_t state);
/* Normal line following for the right ring with a reduced correction limit. */
void Irtracking_UpdateRightRingState(uint16_t state);
/* Drive each wheel towards the given duty with the normal ramp (0..100). */
void Irtracking_DriveWheels(uint8_t left, uint8_t right);
/* Straight cruise plus a small steering offset; bias>0 turns right. */
void Irtracking_ForwardLean(int8_t bias);
void Irtracking_UpdateState(uint16_t state);
/* One-call speed limit for corner recovery; it cannot leak into later steps. */
void Irtracking_UpdateStateAtSpeed(uint16_t state,uint8_t speed);
/* Explicit ring-only policy; no global enable flag can leak into a gap. */
void Irtracking_UpdateRingState(uint16_t state);
void Irtracking_Forward(void);
/* Retain the last steering direction across centered samples. */
void Irtracking_SetSearchDirection(uint8_t right);
/* Supply a route fallback only when no previous steering direction is known. */
void Irtracking_PresetSearchDirection(uint8_t right);
uint8_t Irtracking_HasFault(void);
void Irtracking_Reset(void);
void Irtracking_Update(void);
void Irtracking_Resume(uint8_t left, uint8_t right);
void Irtracking_Init(void);
uint8_t Left_Irtracking_Get(void);  /* compatibility: left inner X3/X4 */
uint8_t Right_Irtracking_Get(void); /* compatibility: right inner X5/X6 */
#endif
