#ifndef GAP_H
#define GAP_H
#include <stdint.h>
/* Test starts the calibrated arc directly; course waits for a missing line. */
void Gap_Start(void);
/* First break: right arc, straight search, exit arc (mode 3). */
void Gap_StartCourse(void);
/* Second break: right arc, then continue to the finish line (mode 4). */
void Gap_StartCourseSecond(void);
/* Gap_Tick result when the finish line has been crossed and the car stopped. */
#define GAP_REACHED_FINISH 3u
/* 4: first-break arc/straight; 5: second-break right arc; 6: arrival settle;
   7: exit arc; 8: finish-line travel; 3: finish reached; 10: approach;
   11: entry settle; 0: handed over; 9: failure. */
uint8_t Gap_Tick(void);
uint8_t Gap_GetDisplay(void);
#endif
