#ifndef COURSE_H
#define COURSE_H
#include <stdint.h>
/* Independent, single-feature modes; nominal 5ms control ticks. */
#define COURSE_MODE_COUNT 6u
#define COURSE_MODE_APPROACH_TICKS 2000u /* 10s to find the selected feature */
#define COURSE_MODE_MAX_TICKS 10000u    /* 50s total, including ring white search */
#define COURSE_APPROACH_MAX_TICKS 12000u /* module-level compatibility limit */
#define COURSE_GAP_WHITE_SAMPLES 6u
#define COURSE_GAP_ENTRY_SETTLE_TICKS 40u
#define COURSE_GAP_RECOVER_TICKS 20u
#define COURSE_GAP_RECOVER_MAX_TICKS 200u
/* Mode 4 finish: when more than four probes see black the car stops at once. */
#define COURSE_FINISH_STOP_BLACK 5u      /* more than four probes black */
#define COURSE_FINISH_BLACK 6u            /* wide black for the travel guard */
#define COURSE_FINISH_CONFIRM_TICKS 3u    /* consecutive wide-black ticks */
#define COURSE_FINISH_RUN_MAX_TICKS 6000u /* 30s to reach the finish after the break */
/* 1 crossing, 2 automatic left/right corner, 3 first break, 4 second break,
   5 left ring, 6 right ring. Mode 3 stops right after its recovery; mode 4
   continues to the finish line and stops there. */
void Course_StartMode(uint8_t mode);
/* 1 active, 0 completed and stopped, 9 fault and stopped. */
uint8_t Course_Tick(void);
uint8_t Course_GetMode(void);
uint8_t Course_GetDisplay(void);
#endif
