/* Eight-channel line follower: X1 at the left, X8 at the right.
 * Modes: 1 crossing, 2 automatic corner, 3 gap, 4 left ring, 5 right ring.
 * Power-on: show 8, then poll the menu immediately (no sensor warmup wait).
 * Short press selects a mode; hold 0.8s and release, then wait 2s to run.
 * A press during countdown or running stops immediately; 0 done, 9 aborted.
 */
#include "stm32f10x.h"
#include "Delay.h"
#include "Key.h"
#include "LEDSEG.h"
#include "robot.h"
#include "Irtracking.h"
#include "Calibration.h"
#include "Buzzer.h"

int main(void)
{
    uint8_t displayed = 255;
    uint8_t value;

    Buzzer_Init();
    /* Release PA15 from JTAG while retaining PA13/PA14 for SWD. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    Key_Init();
    LEDSEG_Init();
    robot_Init();
    Irtracking_Init();
    Calibration_Init();
    Digital_Display(8);

    while (1)
    {
        Calibration_Tick(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0);
        value = Calibration_GetDisplay();
        if (value != displayed)
        {
            Digital_Display(value);
            displayed = value;
        }
        Delay_ms(CAL_TICK_MS);
    }
}
