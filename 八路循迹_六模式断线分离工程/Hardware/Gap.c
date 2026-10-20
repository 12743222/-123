#include "Gap.h"
#include "Calibration.h"
#include "Course.h"
#include "Irtracking.h"
#include "robot.h"
enum { FINISHED=0, REACHED_FINISH=3, RUNNING=4, SECOND_ARC=5, SETTLE=6, EXIT_ARC=7,
       FINISH_TRAVEL=8, FAILED=9, APPROACH=10, ENTRY_SETTLE=11 };
static uint8_t phase, left_pwm, right_pwm, black_count, white_count, clear_count, armed;
static uint8_t second_gap;
static uint8_t finish_confirm;
static int8_t trim;
static uint16_t phase_ticks, finish_run_ticks;
static void stop(void) {
    left_pwm=right_pwm=0;
    robot_speed(0,0,0,0);
}
static uint8_t fail(void) {
    Irtracking_Reset();stop();phase=FAILED;return phase;
}
static uint8_t ramp(uint8_t value,uint8_t target) {
    if(value<target) return target-value>CAL_PWM_STEP?value+CAL_PWM_STEP:target;
    return value-target>CAL_PWM_STEP?value-CAL_PWM_STEP:target;
}
static void drive(uint8_t left,uint8_t right) {
    left_pwm=ramp(left_pwm,left);right_pwm=ramp(right_pwm,right);
    robot_speed(left_pwm,0,right_pwm,0);
}
/* Add the learned straight-line differential to an open-loop command. */
static void drive_trim(uint8_t left,uint8_t right) {
    int16_t l=(int16_t)left+trim, r=(int16_t)right-trim;
    if(l<0) l=0; else if(l>100) l=100;
    if(r<0) r=0; else if(r>100) r=100;
    drive((uint8_t)l,(uint8_t)r);
}
static void drive_run(void) {
    if(phase_ticks<CAL_GAP_ARC1_TICKS) drive_trim(CAL_GAP_ARC1_LEFT,CAL_GAP_ARC1_RIGHT);
    else drive_trim(CAL_GAP_STRAIGHT_PWM,CAL_GAP_STRAIGHT_PWM);
}
static void clear_second_state(void) {
    left_pwm=right_pwm=black_count=white_count=clear_count=armed=0;
    finish_confirm=0;
    finish_run_ticks=0;
}
static uint8_t black_probes(uint8_t sample) {
    uint8_t n=0,i;
    for(i=0;i<8;++i) if(sample&(1u<<i)) ++n;
    return n;
}
/* Mode 4 finish: as soon as more than four probes see black the car stops. */
static uint8_t finish_on_wide_black(uint8_t sample) {
    uint8_t black=black_probes(sample);
    if(black>COURSE_FINISH_STOP_BLACK-1u) {
        if(++finish_confirm>=COURSE_FINISH_CONFIRM_TICKS) {
            Irtracking_Reset();stop();phase=REACHED_FINISH;return 1;
        }
    } else finish_confirm=0;
    return 0;
}
void Gap_Start(void) {
    second_gap=0;
    phase=RUNNING;phase_ticks=0;
    clear_second_state();
    drive_run();
}
void Gap_StartCourse(void) {
    second_gap=0;
    phase=APPROACH;phase_ticks=0;
    clear_second_state();
    trim=0;
}
void Gap_StartCourseSecond(void) {
    second_gap=1;
    phase=APPROACH;phase_ticks=0;
    clear_second_state();
    trim=0;
}
uint8_t Gap_GetDisplay(void) { return phase; }
uint8_t Gap_Tick(void) {
    uint8_t sample;
    if(phase==FINISHED||phase==FAILED) return phase;
    if(phase==APPROACH) {
        if(++phase_ticks>=COURSE_APPROACH_MAX_TICKS) return fail();
        sample=Irtracking_ReadState();
        if(Irtracking_IsInnerLine(sample)) {
            if(clear_count<3) ++clear_count;
            if(clear_count>=3) armed=1;
        } else clear_count=0;
        if(armed&&sample==0) {
            /* Only the expected gap stage may turn a missing line into an
               open-loop maneuver. Stop first to reuse static-start tuning. */
            if(white_count==0) {
                /* Remember the differential the closed-loop approach needed so
                   the open-loop arc and straight keep the same balance. */
                trim=Irtracking_GetBias();
                if(trim>10) trim=10; else if(trim<-10) trim=-10;
            }
            if(++white_count>=COURSE_GAP_WHITE_SAMPLES) {
                Irtracking_Reset();stop();phase=ENTRY_SETTLE;phase_ticks=0;
            } else Irtracking_UpdateStateAtSpeed(IRTRACKING_CENTER_MASK,CAL_GAP_STRAIGHT_PWM);
        } else {
            white_count=0;Irtracking_UpdateStateAtSpeed(sample,CAL_GAP_STRAIGHT_PWM);
            if(Irtracking_HasFault()) return fail();
        }
    } else if(phase==ENTRY_SETTLE) {
        if(++phase_ticks>=COURSE_GAP_ENTRY_SETTLE_TICKS) {
            if(second_gap) {
                /* The second break is one continuous right arc: 2.00s is the
                   requested 90-degree tuning, then hand back to tracking. */
                phase=SECOND_ARC;phase_ticks=0;
                drive_trim(CAL_GAP2_ARC_LEFT,CAL_GAP2_ARC_RIGHT);
            } else Gap_Start();
        }
    } else if(phase==SECOND_ARC) {
        sample=Irtracking_ReadState();
        if(finish_on_wide_black(sample)) return phase;
        if(++phase_ticks>=CAL_GAP2_ARC_TICKS) {
            Irtracking_Resume(left_pwm,right_pwm);
            phase=FINISH_TRAVEL;phase_ticks=finish_run_ticks=0;
        } else drive_trim(CAL_GAP2_ARC_LEFT,CAL_GAP2_ARC_RIGHT);
    } else if(phase==FINISH_TRAVEL) {
        uint8_t black;
        sample=Irtracking_ReadState();
        black=black_probes(sample);
        if(finish_on_wide_black(sample)) return phase;
        if(++finish_run_ticks>=COURSE_FINISH_RUN_MAX_TICKS) return fail();
        if(black>=COURSE_FINISH_BLACK || sample==0)
            Irtracking_UpdateStateAtSpeed(IRTRACKING_CENTER_MASK,CAL_GAP_STRAIGHT_PWM);
        else
            Irtracking_UpdateStateAtSpeed(sample,CAL_GAP_STRAIGHT_PWM);
        if(Irtracking_HasFault()) return fail();
    } else if(phase==RUNNING) {
        ++phase_ticks;
        if(phase_ticks>=CAL_GAP_ARC1_TICKS) {
            sample=Irtracking_ReadState();
            if(sample!=0) ++black_count;else black_count=0;
            if(black_count>=CAL_GAP_LINE_SAMPLES) {
                stop();phase=SETTLE;phase_ticks=0;return phase;
            }
            if(phase_ticks-CAL_GAP_ARC1_TICKS>=CAL_GAP_STRAIGHT_MAX_TICKS) return fail();
        }
        drive_run();
    } else if(phase==SETTLE) {
        if(++phase_ticks>=CAL_GAP_SETTLE_TICKS) {
            phase=EXIT_ARC;phase_ticks=0;
            drive(CAL_GAP_EXIT_ARC_LEFT,CAL_GAP_EXIT_ARC_RIGHT);
        }
    } else if(phase==EXIT_ARC) {
        if(++phase_ticks>=CAL_GAP_EXIT_ARC_TICKS) {
            Irtracking_Resume(left_pwm,right_pwm);phase=FINISHED;
        } else drive_trim(CAL_GAP_EXIT_ARC_LEFT,CAL_GAP_EXIT_ARC_RIGHT);
    }
    return phase;
}
