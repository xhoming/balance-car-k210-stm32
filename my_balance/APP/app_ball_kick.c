#include "app_ball_kick.h"
#include "AllHeader.h"

volatile BallKickInput_t g_ball_input = {0, 0, 0, 0, 0, 0};
volatile int16_t g_ball_debug_v = 0;
volatile int16_t g_ball_debug_t = 0;
volatile uint8_t g_ball_debug_state = BALL_STATE_SEARCH;
volatile uint32_t g_ball_rx_bytes = 0;
volatile uint32_t g_ball_rx_frames = 0;
volatile uint32_t g_ball_rx_bad_chk = 0;
volatile uint32_t g_ball_rx_bad_tail = 0;
volatile uint8_t g_ball_last_rx = 0;

/*
 * K210 ball frame, 8 bytes:
 *   B6 state error_i8 near confidence y_score checksum 6B
 *   checksum = state + error_u + near + confidence + y_score.
 *
 * The K210 only measures the ball. STM32 owns the action state machine:
 * align, approach, fixed attack pulse, recover.
 */
float BallKick_Align_Kp = 6.0f;
float BallKick_Approach_Kp = 4.0f;
float BallKick_Charge_Kp = 2.2f;
float BallKick_Turn_Kd = 1.10f;
float BallKick_Gyro_Direction = 1.0f;
float BallKick_Max_PWM = 520.0f;

#define BALL_ALIGN_WINDOW          20
#define BALL_ATTACK_ERROR_WINDOW   24
#define BALL_ATTACK_NEAR_SCORE     72
#define BALL_ATTACK_Y_SCORE        70
#define BALL_ALIGN_SPEED           8.0f
#define BALL_APPROACH_SPEED        22.0f
#define BALL_ATTACK_SPEED          38.0f
#define BALL_ATTACK_MS             650U
#define BALL_RECOVER_MS            250U

static uint8_t ball_action_state = BALL_STATE_SEARCH;
static uint32_t ball_action_until_ms = 0;
static int16_t ball_attack_error = 0;

static int16_t ball_limiti16(int16_t value, int16_t min_value,
                             int16_t max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float ball_limitf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static uint8_t ball_measure_valid(void)
{
    uint32_t age = HAL_GetTick() - g_ball_input.last_update_ms;

    if (age > BALL_INPUT_TIMEOUT_MS) return 0;
    if (g_ball_input.state == BALL_STATE_SEARCH) return 0;
    if (g_ball_input.state == BALL_STATE_BRAKE) return 0;
    if (g_ball_input.confidence < BALL_MIN_CONFIDENCE) return 0;
    return 1;
}

static void ball_update_action_state(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t seen = ball_measure_valid();
    int16_t abs_error = ball_limiti16(g_ball_input.error, -100, 100);
    if (abs_error < 0) abs_error = -abs_error;

    if (g_ball_input.state == BALL_STATE_BRAKE) {
        ball_action_state = BALL_STATE_BRAKE;
        ball_action_until_ms = now + BALL_RECOVER_MS;
        g_ball_debug_state = ball_action_state;
        return;
    }

    if (ball_action_state == BALL_STATE_CHARGE) {
        if ((int32_t)(now - ball_action_until_ms) < 0) {
            g_ball_debug_state = ball_action_state;
            return;
        }
        ball_action_state = BALL_STATE_BRAKE;
        ball_action_until_ms = now + BALL_RECOVER_MS;
    }

    if (ball_action_state == BALL_STATE_BRAKE) {
        if ((int32_t)(now - ball_action_until_ms) < 0) {
            g_ball_debug_state = ball_action_state;
            return;
        }
        ball_action_state = BALL_STATE_SEARCH;
    }

    if (!seen) {
        ball_action_state = BALL_STATE_SEARCH;
        g_ball_debug_state = ball_action_state;
        return;
    }

    if (abs_error <= BALL_ATTACK_ERROR_WINDOW &&
        (g_ball_input.area >= BALL_ATTACK_NEAR_SCORE ||
         g_ball_input.speed >= BALL_ATTACK_Y_SCORE)) {
        ball_action_state = BALL_STATE_CHARGE;
        ball_action_until_ms = now + BALL_ATTACK_MS;
        ball_attack_error = g_ball_input.error;
        g_ball_debug_state = ball_action_state;
        return;
    }

    if (abs_error > BALL_ALIGN_WINDOW) {
        ball_action_state = BALL_STATE_ALIGN;
    } else {
        ball_action_state = BALL_STATE_APPROACH;
    }

    g_ball_debug_state = ball_action_state;
}

void BallKick_UpdateInput(uint8_t state, int16_t error, uint8_t area,
                          uint8_t confidence, uint8_t speed)
{
    if (state > BALL_STATE_BRAKE) state = BALL_STATE_SEARCH;
    if (area > 100) area = 100;
    if (confidence > 100) confidence = 100;
    if (speed > 100) speed = 100;
    error = ball_limiti16(error, -100, 100);

    if (confidence < BALL_MIN_CONFIDENCE && state != BALL_STATE_BRAKE) {
        state = BALL_STATE_SEARCH;
        error = 0;
        area = 0;
        speed = 0;
    }

    g_ball_input.state = state;
    g_ball_input.error = error;
    g_ball_input.area = area;
    g_ball_input.confidence = confidence;
    g_ball_input.speed = speed;
    g_ball_input.last_update_ms = HAL_GetTick();
}

float BallKick_TurnCalc(float gyro_z)
{
    float kp;
    float turn_pwm;
    float gyro_feedback;

    ball_update_action_state();

    if (ball_action_state == BALL_STATE_SEARCH ||
        ball_action_state == BALL_STATE_BRAKE) {
        g_ball_debug_t = 0;
        return 0.0f;
    }

    if (ball_action_state == BALL_STATE_CHARGE) {
        kp = BallKick_Charge_Kp;
        turn_pwm = (float)ball_attack_error * kp;
        turn_pwm = ball_limitf(turn_pwm, -BallKick_Max_PWM, BallKick_Max_PWM);
        g_ball_debug_t = (int16_t)turn_pwm;
        return turn_pwm;
    } else if (ball_action_state == BALL_STATE_APPROACH) {
        kp = BallKick_Approach_Kp;
    } else {
        kp = BallKick_Align_Kp;
    }

    gyro_feedback = gyro_z * BallKick_Gyro_Direction;
    turn_pwm = (float)g_ball_input.error * kp - gyro_feedback * BallKick_Turn_Kd;
    turn_pwm = ball_limitf(turn_pwm, -BallKick_Max_PWM, BallKick_Max_PWM);

    g_ball_debug_t = (int16_t)turn_pwm;
    return turn_pwm;
}

float BallKick_SpeedTarget(void)
{
    ball_update_action_state();

    if (ball_action_state == BALL_STATE_CHARGE) {
        g_ball_debug_v = (int16_t)BALL_ATTACK_SPEED;
        return BALL_ATTACK_SPEED;
    }

    if (ball_action_state == BALL_STATE_APPROACH) {
        g_ball_debug_v = (int16_t)BALL_APPROACH_SPEED;
        return BALL_APPROACH_SPEED;
    }

    if (ball_action_state == BALL_STATE_ALIGN) {
        g_ball_debug_v = (int16_t)BALL_ALIGN_SPEED;
        return BALL_ALIGN_SPEED;
    }

    if (ball_action_state == BALL_STATE_BRAKE) {
        g_ball_debug_v = 0;
        return 0.0f;
    }

    g_ball_debug_v = 0;
    return 0.0f;
}

void BallKick_Reset(void)
{
    g_ball_input.state = BALL_STATE_SEARCH;
    g_ball_input.error = 0;
    g_ball_input.area = 0;
    g_ball_input.confidence = 0;
    g_ball_input.speed = 0;
    g_ball_input.last_update_ms = 0;
    g_ball_debug_v = 0;
    g_ball_debug_t = 0;
    g_ball_debug_state = BALL_STATE_SEARCH;
    ball_action_state = BALL_STATE_SEARCH;
    ball_action_until_ms = 0;
    ball_attack_error = 0;
    g_ball_rx_bytes = 0;
    g_ball_rx_frames = 0;
    g_ball_rx_bad_chk = 0;
    g_ball_rx_bad_tail = 0;
    g_ball_last_rx = 0;
}

void BallKick_ParseByte(uint8_t rx)
{
    static uint8_t buf[8];
    static uint8_t idx = 0;
    uint8_t chk;

    g_ball_rx_bytes++;
    g_ball_last_rx = rx;

    if (idx == 0) {
        if (rx != BALL_FRAME_HEAD) return;
        buf[idx++] = rx;
        return;
    }

    if (idx >= sizeof(buf)) {
        idx = 0;
        return;
    }

    buf[idx++] = rx;
    if (idx < sizeof(buf)) return;

    idx = 0;
    if (buf[7] != BALL_FRAME_TAIL) {
        g_ball_rx_bad_tail++;
        return;
    }

    chk = (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4] + buf[5]);
    if (chk != buf[6]) {
        g_ball_rx_bad_chk++;
        return;
    }

    g_ball_rx_frames++;
    BallKick_UpdateInput(buf[1], (int8_t)buf[2], buf[3], buf[4], buf[5]);
}
