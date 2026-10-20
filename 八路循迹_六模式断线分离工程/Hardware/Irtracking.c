#include "Irtracking.h"
#include "robot.h"

static uint8_t tracking_state = IRTRACKING_ALL_BLACK;
static uint8_t candidate_state = IRTRACKING_ALL_BLACK, candidate_count;
static uint8_t left_pwm, right_pwm, tracking_fault;
static uint16_t lost_ticks;
static int8_t last_direction;
static int16_t bias_q; /* learned straight-line differential in Q3 fixed point */
static uint8_t ring_boost_ticks, ring_boost_clockwise;

static uint8_t ramp(uint8_t current, uint8_t target)
{
    if (target > current)
        return target-current > IRTRACKING_PWM_STEP ? current+IRTRACKING_PWM_STEP : target;
    return current-target > IRTRACKING_PWM_STEP ? current-IRTRACKING_PWM_STEP : target;
}

static void cut_pwm(void)
{
    left_pwm = right_pwm = 0;
    robot_speed(0, 0, 0, 0);
}

static void fault_stop(void)
{
    tracking_fault = 1;
    cut_pwm();
}

void Irtracking_Reset(void)
{
    tracking_state = candidate_state = IRTRACKING_ALL_BLACK;
    candidate_count = tracking_fault = 0;
    last_direction = 0;
    lost_ticks = 0;
    ring_boost_ticks = 0;
    cut_pwm();
}

void Irtracking_Resume(uint8_t left, uint8_t right)
{
    if (left > 100 || right > 100) { fault_stop(); return; }
    left_pwm = left;
    right_pwm = right;
    tracking_state = candidate_state = IRTRACKING_CENTER_MASK;
    candidate_count = tracking_fault = 0;
    last_direction = 0;
    lost_ticks = 0;
}

uint8_t Irtracking_HasFault(void) { return tracking_fault; }

uint8_t Irtracking_ReadState(void)
{
    static const uint16_t pins[8] = {
        IRTRACKING_X1_PIN, IRTRACKING_X2_PIN, IRTRACKING_X3_PIN, IRTRACKING_X4_PIN,
        IRTRACKING_X5_PIN, IRTRACKING_X6_PIN, IRTRACKING_X7_PIN, IRTRACKING_X8_PIN
    };
    uint16_t inputs = GPIO_ReadInputData(GPIOB);
    uint8_t mask = 0, i;
    for (i=0;i<8;++i)
        if (((inputs & pins[i]) != 0) == IRTRACKING_BLACK_LEVEL) mask |= (uint8_t)(1u<<i);
    return mask;
}

uint8_t Irtracking_GetRegions(uint8_t state)
{
    uint8_t regions = 0, i;
    for (i=0;i<4;++i)
        if (state & (3u<<(2*i))) regions |= (uint8_t)(1u<<i);
    return regions;
}

uint8_t Irtracking_IsInnerLine(uint8_t state)
{
    return state != 0 && (state & ~IRTRACKING_INNER_MASK) == 0;
}

void Irtracking_Update(void) { Irtracking_UpdateState(Irtracking_ReadState()); }

void Irtracking_Forward(void)
{
    if (tracking_fault) return;
    tracking_state = candidate_state = IRTRACKING_CENTER_MASK;
    candidate_count = 0;
    lost_ticks = 0;
    Irtracking_UpdateState(IRTRACKING_CENTER_MASK);
}

void Irtracking_SetSearchDirection(uint8_t right)
{
    last_direction = right ? 1 : -1;
}

void Irtracking_PresetSearchDirection(uint8_t right)
{
    if (!last_direction) Irtracking_SetSearchDirection(right);
}

static void ring_search_arc(uint8_t right)
{
    /* Keep rolling forward while turning: the outer wheel searches, the inner
       wheel still advances so the car does not spin on the spot. */
    left_pwm = ramp(left_pwm, right ? IRTRACKING_SEARCH_PWM : IRTRACKING_RING_SEARCH_INNER_PWM);
    right_pwm = ramp(right_pwm, right ? IRTRACKING_RING_SEARCH_INNER_PWM : IRTRACKING_SEARCH_PWM);
    robot_speed(left_pwm, 0, right_pwm, 0);
}
static void drive_search(uint8_t right)
{
    left_pwm = ramp(left_pwm, right ? IRTRACKING_SEARCH_PWM : 0);
    right_pwm = ramp(right_pwm, right ? 0 : IRTRACKING_SEARCH_PWM);
    robot_speed(left_pwm, 0, right_pwm, 0);
}

static void update_state(uint16_t state, uint8_t ring_search, uint8_t speed, uint8_t delta_limit)
{
    uint8_t left_target = speed, right_target = speed;
    uint8_t out_left, out_right;
    uint8_t count = 0, i;
    int8_t sum = 0;
    int16_t correction = 0;
    uint8_t max_delta = (uint8_t)(delta_limit*speed/IRTRACKING_STRAIGHT_SPEED);
    static const int8_t weights[8] = {-7,-5,-3,-1,1,3,5,7};
    if (tracking_fault) return;
    if (!speed || speed > IRTRACKING_STRAIGHT_SPEED) { fault_stop(); return; }
    if (state > IRTRACKING_ALL_BLACK) { fault_stop(); return; }
    if (state == IRTRACKING_ALL_BLACK) { Irtracking_Reset(); return; }
    if (state != candidate_state) { candidate_state = (uint8_t)state; candidate_count = 1; }
    else if (candidate_count < IRTRACKING_FILTER_SAMPLES) ++candidate_count;
    if (candidate_count >= IRTRACKING_FILTER_SAMPLES) tracking_state = candidate_state;
    if (tracking_state == IRTRACKING_ALL_BLACK) return;

    if (tracking_state == 0)
    {
        /* A lost line never stops the car by itself. Keep the last correction
           for a short coast, then turn towards the last known side (or the
           default side) and keep searching until the mode limit or the button. */
        if (ring_search)
        {
            if (!last_direction) Irtracking_SetSearchDirection(IRTRACKING_DEFAULT_SEARCH_RIGHT);
            ring_search_arc(last_direction > 0);
            return;
        }
        if (++lost_ticks <= IRTRACKING_SEARCH_DELAY_TICKS)
        {
            if (!last_direction) { cut_pwm(); return; }
            correction = last_direction < 0 ? -max_delta : max_delta;
        }
        else
        {
            if (!last_direction) Irtracking_SetSearchDirection(IRTRACKING_DEFAULT_SEARCH_RIGHT);
            drive_search(last_direction > 0);
            return;
        }
    }
    else
    {
        /* Route controllers consume their junction cues first. Other partial
           patterns remain usable for steering and must not latch a loss fault. */
        lost_ticks = 0;
        for (i=0;i<8;++i)
            if (tracking_state & (1u<<i)) { sum += weights[i]; ++count; }
        if (sum < 0)
        {
            last_direction = -1;
        }
        else if (sum > 0)
        {
            last_direction = 1;
        }
        else if ((tracking_state & IRTRACKING_LEFT_OUTER_MASK) &&
                 (tracking_state & IRTRACKING_RIGHT_OUTER_MASK) &&
                 !(tracking_state & IRTRACKING_INNER_MASK) && last_direction)
        {
            correction = last_direction < 0 ? -max_delta : max_delta;
        }
        if(sum) correction=(int16_t)(sum*IRTRACKING_STEER_GAIN*speed/(count*IRTRACKING_STRAIGHT_SPEED));
    }
    if(correction>max_delta) correction=max_delta;
    if(correction < -(int16_t)max_delta) correction=-(int16_t)max_delta;
    /* Track the steady differential needed to hold the line. It absorbs the
       left/right motor mismatch that closed-loop steering hides, so open-loop
       gap maneuvers can start from the same balance instead of drifting. */
    if (tracking_state != 0)
    {
        bias_q = bias_q - bias_q/8 + correction;
        if (bias_q > IRTRACKING_BIAS_MAX) bias_q = IRTRACKING_BIAS_MAX;
        else if (bias_q < -IRTRACKING_BIAS_MAX) bias_q = -IRTRACKING_BIAS_MAX;
    }
    left_target=(uint8_t)(speed+correction);
    right_target=(uint8_t)(speed-correction);
    left_pwm = ramp(left_pwm, left_target);
    right_pwm = ramp(right_pwm, right_target);
    out_left = left_pwm;
    out_right = right_pwm;
    if (ring_boost_ticks)
    {
        --ring_boost_ticks;
        if (ring_boost_clockwise)
            out_left = out_left <= 100u - IRTRACKING_RING_BOOST_PWM ? out_left + IRTRACKING_RING_BOOST_PWM : 100u;
        else
            out_right = out_right <= 100u - IRTRACKING_RING_BOOST_PWM ? out_right + IRTRACKING_RING_BOOST_PWM : 100u;
    }
    robot_speed(out_left, 0, out_right, 0);
}

void Irtracking_UpdateState(uint16_t state) { update_state(state, 0, IRTRACKING_STRAIGHT_SPEED, IRTRACKING_MAX_DELTA); }
void Irtracking_UpdateRingState(uint16_t state) { update_state(state, 1, IRTRACKING_STRAIGHT_SPEED, IRTRACKING_MAX_DELTA); }
void Irtracking_UpdateRingStateGentle(uint16_t state) { update_state(state, 1, IRTRACKING_STRAIGHT_SPEED, IRTRACKING_RING_MAX_DELTA); }
void Irtracking_UpdateRightRingState(uint16_t state) { update_state(state, 1, IRTRACKING_STRAIGHT_SPEED, IRTRACKING_RIGHT_RING_MAX_DELTA); }
void Irtracking_UpdateStateAtSpeed(uint16_t state,uint8_t speed) { update_state(state, 0, speed, IRTRACKING_MAX_DELTA); }

void Irtracking_DriveWheels(uint8_t left, uint8_t right)
{
    if(left>100u) left=100u;
    if(right>100u) right=100u;
    left_pwm=ramp(left_pwm,left);
    right_pwm=ramp(right_pwm,right);
    robot_speed(left_pwm,0,right_pwm,0);
}

/* Straight cruise with a small fixed steering offset: bias>0 turns right
   (left wheel faster). Used when leaving the right roundabout. */
void Irtracking_ForwardLean(int8_t bias)
{
    int16_t l, r;
    update_state(IRTRACKING_CENTER_MASK, 0, IRTRACKING_STRAIGHT_SPEED, IRTRACKING_MAX_DELTA);
    if (!bias) return;
    l = (int16_t)left_pwm + bias;
    r = (int16_t)right_pwm - bias;
    if (l < 0) l = 0; else if (l > 100) l = 100;
    if (r < 0) r = 0; else if (r > 100) r = 100;
    robot_speed((uint8_t)l, 0, (uint8_t)r, 0);
}

int8_t Irtracking_GetBias(void) { return (int8_t)(bias_q/8); }

void Irtracking_SetRingBoost(uint8_t clockwise, uint8_t ticks)
{
    ring_boost_clockwise = clockwise ? 1u : 0u;
    ring_boost_ticks = ticks;
}

void Irtracking_Init(void)
{
    GPIO_InitTypeDef cfg;
    bias_q = 0;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    cfg.GPIO_Mode = GPIO_Mode_IPU;
    cfg.GPIO_Pin = IRTRACKING_INPUT_PINS;
    cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &cfg);
}
uint8_t Left_Irtracking_Get(void) { return (Irtracking_ReadState() & 0x0cu) ? 1u : 0u; }
uint8_t Right_Irtracking_Get(void) { return (Irtracking_ReadState() & 0x30u) ? 1u : 0u; }
