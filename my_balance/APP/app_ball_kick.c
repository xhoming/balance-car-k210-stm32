#include "app_ball_kick.h"
#include "AllHeader.h"

volatile BallKickInput_t g_ball_input = {0, 0, 0, 0, 0, 0};
volatile int16_t g_ball_debug_v = 0;
volatile int16_t g_ball_debug_t = 0;
volatile uint8_t g_ball_debug_state = BALL_ACTION_STOP;
volatile uint8_t g_ball_debug_area_x10 = 0;
volatile uint16_t g_ball_debug_missed = 999;
volatile uint32_t g_ball_rx_bytes = 0;
volatile uint32_t g_ball_rx_frames = 0;
volatile uint32_t g_ball_rx_bad_chk = 0;
volatile uint32_t g_ball_rx_bad_tail = 0;

/* turn_pwm = image_error * Kp - gyro_z * Kd */
float BallKick_Track_Kp = 1.2f;
float BallKick_Kick_Kp = 0.6f;
float BallKick_Turn_Kd = 1.10f;
float BallKick_Gyro_Direction = 1.0f;
float BallKick_Track_Max_PWM = 25.0f;
float BallKick_Kick_Max_PWM = 60.0f;

#define BALL_CONFIDENCE_MIN          40U
#define BALL_VALID_HOLD_MS          150U
#define BALL_TURN_DEADBAND            4
#define BALL_TURN_SLEW_PWM         4.0f

#define BALL_ARM_AREA_MAX_X10        80U
#define BALL_STRIKE_AREA_MIN_X10     95U
#define BALL_STRIKE_AREA_MAX_X10    160U
#define BALL_STRIKE_ERROR_LIMIT      12
#define BALL_ARM_CONFIRM_FRAMES       5U
#define BALL_TRACK_MIN_FRAMES        12U
#define BALL_STRIKE_CONFIRM_FRAMES    4U

#define BALL_APPROACH_MIN_SPEED     0.15f
#define BALL_APPROACH_MAX_SPEED     1.00f
#define BALL_APPROACH_STOP_ERROR      40
#define BALL_APPROACH_FULL_ERROR       8

#define BALL_STRIKE_SPEED            5.0f
#define BALL_STRIKE_TICKS            400U
#define BALL_STRIKE_RAMP_TICKS       180U

#define BALL_AREA_FILTER_SIZE          5U

static volatile BallKickInput_t ball_pending_input;
static volatile uint8_t ball_pending_ready = 0;

static uint8_t ball_action_state = BALL_ACTION_STOP;
static uint8_t ball_seen_stop = 0;
static uint8_t ball_session_running = 0;
static uint8_t ball_current_valid = 0;
static uint8_t ball_last_seq = 0;
static uint8_t ball_seq_initialized = 0;
static uint8_t ball_arm_frames = 0;
static uint8_t ball_track_frames = 0;
static uint8_t ball_strike_frames = 0;
static uint8_t ball_approach_armed = 0;
static uint8_t ball_area_history[BALL_AREA_FILTER_SIZE];
static uint8_t ball_area_count = 0;
static uint8_t ball_area_pos = 0;
static uint8_t ball_filtered_area = 0;
static uint16_t ball_action_ticks = 0;
static uint32_t ball_last_valid_ms = 0;
static int16_t ball_filtered_error = 0;
static int16_t ball_strike_error = 0;
static float ball_last_turn_pwm = 0.0f;

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

static void ball_reset_tracking(void)
{
    uint8_t i;

    ball_last_seq = 0;
    ball_seq_initialized = 0;
    ball_arm_frames = 0;
    ball_track_frames = 0;
    ball_strike_frames = 0;
    ball_approach_armed = 0;
    ball_area_count = 0;
    ball_area_pos = 0;
    ball_filtered_area = 0;
    ball_filtered_error = 0;
    ball_strike_error = 0;
    for (i = 0; i < BALL_AREA_FILTER_SIZE; i++) {
        ball_area_history[i] = 0;
    }
    g_ball_debug_area_x10 = 0;
}

static void ball_set_state(uint8_t state)
{
    if (state != ball_action_state) {
        ball_action_state = state;
        Velocity_PI_Reset();
        ball_last_turn_pwm = 0.0f;
    }
    g_ball_debug_state = state;
}

static uint8_t ball_median_area(uint8_t area)
{
    uint8_t values[BALL_AREA_FILTER_SIZE];
    uint8_t i;
    uint8_t j;
    uint8_t temp;

    ball_area_history[ball_area_pos] = area;
    ball_area_pos++;
    if (ball_area_pos >= BALL_AREA_FILTER_SIZE) ball_area_pos = 0;
    if (ball_area_count < BALL_AREA_FILTER_SIZE) ball_area_count++;

    for (i = 0; i < ball_area_count; i++) values[i] = ball_area_history[i];
    for (i = 1; i < ball_area_count; i++) {
        temp = values[i];
        j = i;
        while (j > 0 && values[j - 1] > temp) {
            values[j] = values[j - 1];
            j--;
        }
        values[j] = temp;
    }
    return values[ball_area_count / 2U];
}

static uint8_t ball_consume_pending(void)
{
    BallKickInput_t input;

    if (!ball_pending_ready) return 0;

    input.seq = ball_pending_input.seq;
    input.flags = ball_pending_input.flags;
    input.error = ball_pending_input.error;
    input.area_x10 = ball_pending_input.area_x10;
    input.confidence = ball_pending_input.confidence;
    input.last_update_ms = ball_pending_input.last_update_ms;
    ball_pending_ready = 0;

    g_ball_input.seq = input.seq;
    g_ball_input.error = input.error;
    g_ball_input.area_x10 = input.area_x10;
    g_ball_input.confidence = input.confidence;
    g_ball_input.last_update_ms = input.last_update_ms;
    g_ball_input.flags = input.flags;
    return 1;
}

static uint8_t ball_take_new_seq(void)
{
    if (!ball_seq_initialized || g_ball_input.seq != ball_last_seq) {
        ball_last_seq = g_ball_input.seq;
        ball_seq_initialized = 1;
        return 1;
    }
    return 0;
}

static void ball_stop_session(uint8_t require_new_stop)
{
    ball_session_running = 0;
    ball_current_valid = 0;
    ball_action_ticks = 0;
    if (require_new_stop) ball_seen_stop = 0;
    ball_reset_tracking();
    ball_set_state(BALL_ACTION_STOP);
}

static void ball_process_measurement(void)
{
    int16_t abs_error;

    if (!ball_take_new_seq()) return;

    ball_filtered_area = ball_median_area(g_ball_input.area_x10);
    if (ball_track_frames == 0U) {
        ball_filtered_error = g_ball_input.error;
    } else {
        ball_filtered_error = (int16_t)((ball_filtered_error * 2 +
                                         g_ball_input.error) / 3);
    }
    g_ball_debug_area_x10 = ball_filtered_area;

    if (ball_track_frames < 255U) ball_track_frames++;

    if (!ball_approach_armed) {
        if (ball_filtered_area <= BALL_ARM_AREA_MAX_X10) {
            if (ball_arm_frames < BALL_ARM_CONFIRM_FRAMES) ball_arm_frames++;
        } else {
            ball_arm_frames = 0;
        }
        if (ball_arm_frames >= BALL_ARM_CONFIRM_FRAMES) {
            ball_approach_armed = 1;
        }
    }

    abs_error = ball_filtered_error;
    if (abs_error < 0) abs_error = -abs_error;

    if (ball_approach_armed &&
        ball_track_frames >= BALL_TRACK_MIN_FRAMES &&
        abs_error <= BALL_STRIKE_ERROR_LIMIT &&
        ball_filtered_area >= BALL_STRIKE_AREA_MIN_X10 &&
        ball_filtered_area <= BALL_STRIKE_AREA_MAX_X10) {
        if (ball_strike_frames < BALL_STRIKE_CONFIRM_FRAMES) {
            ball_strike_frames++;
        }
    } else {
        ball_strike_frames = 0;
    }

    if (ball_strike_frames >= BALL_STRIKE_CONFIRM_FRAMES) {
        ball_strike_error = ball_filtered_error;
        ball_action_ticks = BALL_STRIKE_TICKS;
        ball_set_state(BALL_ACTION_STRIKE);
    }
}

static void ball_update_action_state(void)
{
    uint32_t now;
    uint32_t frame_age;
    uint8_t new_input;
    uint8_t running;
    uint8_t valid;

    now = HAL_GetTick();
    new_input = ball_consume_pending();
    frame_age = now - g_ball_input.last_update_ms;
    if (frame_age > 999U) frame_age = 999U;
    g_ball_debug_missed = (uint16_t)frame_age;

    if (g_ball_input.last_update_ms == 0U ||
        frame_age > BALL_INPUT_TIMEOUT_MS) {
        ball_stop_session(1);
        return;
    }

    running = (g_ball_input.flags & BALL_FLAG_RUNNING) != 0U;
    valid = (g_ball_input.flags & BALL_FLAG_VALID) != 0U &&
            g_ball_input.confidence >= BALL_CONFIDENCE_MIN;

    if (!running) {
        ball_seen_stop = 1;
        ball_stop_session(0);
        return;
    }

    if (!ball_session_running) {
        if (!ball_seen_stop) {
            ball_set_state(BALL_ACTION_STOP);
            return;
        }
        ball_seen_stop = 0;
        ball_session_running = 1;
        ball_reset_tracking();
        ball_set_state(BALL_ACTION_TRACK);
    }

    if (ball_action_state == BALL_ACTION_RECOVER) return;

    if (ball_action_state == BALL_ACTION_STRIKE) {
        if (ball_action_ticks > 0U) ball_action_ticks--;
        if (ball_action_ticks == 0U) ball_set_state(BALL_ACTION_RECOVER);
        return;
    }

    if (new_input && valid) {
        ball_current_valid = 1;
        ball_last_valid_ms = now;
        ball_process_measurement();
        return;
    }

    if (new_input && !valid) ball_current_valid = 0;
    if ((now - ball_last_valid_ms) > BALL_VALID_HOLD_MS) {
        ball_current_valid = 0;
        ball_strike_frames = 0;
    }
}

void BallKick_UpdateInput(uint8_t seq, uint8_t flags, int16_t error,
                          uint8_t area_x10, uint8_t confidence)
{
    ball_pending_ready = 0;
    ball_pending_input.seq = seq;
    ball_pending_input.error = ball_limiti16(error, -100, 100);
    ball_pending_input.area_x10 = area_x10;
    ball_pending_input.confidence = (confidence > 100U) ? 100U : confidence;
    ball_pending_input.last_update_ms = HAL_GetTick();
    ball_pending_input.flags = flags;
    ball_pending_ready = 1;
}

float BallKick_TurnCalc(float gyro_z)
{
    float error;
    float kp;
    float max_pwm;
    float turn_pwm;

    if (ball_action_state == BALL_ACTION_STOP ||
        ball_action_state == BALL_ACTION_RECOVER) {
        ball_last_turn_pwm = 0.0f;
        g_ball_debug_t = 0;
        return 0.0f;
    }

    if (ball_action_state == BALL_ACTION_STRIKE) {
        error = (float)ball_strike_error;
        kp = BallKick_Kick_Kp;
        max_pwm = BallKick_Kick_Max_PWM;
    } else {
        if (!ball_current_valid) {
            g_ball_debug_t = 0;
            return 0.0f;
        }
        error = (float)ball_filtered_error;
        kp = BallKick_Track_Kp;
        max_pwm = BallKick_Track_Max_PWM;
    }

    if (error > -(float)BALL_TURN_DEADBAND &&
        error < (float)BALL_TURN_DEADBAND) {
        error = 0.0f;
    }

    turn_pwm = error * kp -
               gyro_z * BallKick_Gyro_Direction * BallKick_Turn_Kd;
    turn_pwm = ball_limitf(turn_pwm, -max_pwm, max_pwm);
    turn_pwm = ball_limitf(turn_pwm,
                           ball_last_turn_pwm - BALL_TURN_SLEW_PWM,
                           ball_last_turn_pwm + BALL_TURN_SLEW_PWM);
    ball_last_turn_pwm = turn_pwm;
    g_ball_debug_t = (int16_t)turn_pwm;
    return turn_pwm;
}

static float ball_approach_speed(void)
{
    int16_t abs_error;
    float distance_scale;
    float center_scale;
    float speed;

    if (!ball_current_valid) return 0.0f;

    abs_error = ball_filtered_error;
    if (abs_error < 0) abs_error = -abs_error;
    if (abs_error >= BALL_APPROACH_STOP_ERROR) return 0.0f;
    if (ball_filtered_area >= BALL_STRIKE_AREA_MIN_X10) return 0.0f;

    distance_scale = (float)(BALL_STRIKE_AREA_MIN_X10 - ball_filtered_area) /
                     (float)BALL_STRIKE_AREA_MIN_X10;
    distance_scale = ball_limitf(distance_scale, 0.0f, 1.0f);
    speed = BALL_APPROACH_MIN_SPEED +
            (BALL_APPROACH_MAX_SPEED - BALL_APPROACH_MIN_SPEED) *
            distance_scale;

    if (abs_error <= BALL_APPROACH_FULL_ERROR) {
        center_scale = 1.0f;
    } else {
        center_scale = (float)(BALL_APPROACH_STOP_ERROR - abs_error) /
                       (float)(BALL_APPROACH_STOP_ERROR -
                               BALL_APPROACH_FULL_ERROR);
    }
    return speed * ball_limitf(center_scale, 0.0f, 1.0f);
}

float BallKick_SpeedTarget(void)
{
    uint16_t elapsed_ticks;
    uint16_t ramp_ticks;
    float speed;

    ball_update_action_state();

    if (ball_action_state == BALL_ACTION_TRACK) {
        speed = ball_approach_speed();
        g_ball_debug_v = (int16_t)(speed * 10.0f + 0.5f);
        return speed;
    }

    if (ball_action_state == BALL_ACTION_STRIKE) {
        elapsed_ticks = BALL_STRIKE_TICKS - ball_action_ticks;
        ramp_ticks = elapsed_ticks;
        if (ball_action_ticks < ramp_ticks) ramp_ticks = ball_action_ticks;

        if (ramp_ticks < BALL_STRIKE_RAMP_TICKS) {
            speed = BALL_STRIKE_SPEED * (float)ramp_ticks /
                    (float)BALL_STRIKE_RAMP_TICKS;
        } else {
            speed = BALL_STRIKE_SPEED;
        }
        g_ball_debug_v = (int16_t)(speed * 10.0f + 0.5f);
        return speed;
    }

    g_ball_debug_v = 0;
    return 0.0f;
}

int BallKick_FilterVelocityPwm(int velocity_pwm)
{
    if (ball_action_state == BALL_ACTION_STOP ||
        ball_action_state == BALL_ACTION_RECOVER) {
        return 0;
    }
    return velocity_pwm;
}

void BallKick_Reset(void)
{
    g_ball_input.seq = 0;
    g_ball_input.flags = 0;
    g_ball_input.error = 0;
    g_ball_input.area_x10 = 0;
    g_ball_input.confidence = 0;
    g_ball_input.last_update_ms = 0;
    g_ball_debug_v = 0;
    g_ball_debug_t = 0;
    g_ball_debug_state = BALL_ACTION_STOP;
    g_ball_debug_area_x10 = 0;
    g_ball_debug_missed = 999;

    ball_pending_ready = 0;
    ball_action_state = BALL_ACTION_STOP;
    ball_seen_stop = 0;
    ball_session_running = 0;
    ball_current_valid = 0;
    ball_action_ticks = 0;
    ball_last_valid_ms = 0;
    ball_last_turn_pwm = 0.0f;
    ball_reset_tracking();
    Velocity_PI_Reset();

    g_ball_rx_bytes = 0;
    g_ball_rx_frames = 0;
    g_ball_rx_bad_chk = 0;
    g_ball_rx_bad_tail = 0;
}

void BallKick_ParseByte(uint8_t rx)
{
    static uint8_t buf[8];
    static uint8_t idx = 0;
    uint8_t chk;

    g_ball_rx_bytes++;

    if (idx == 0U) {
        if (rx != BALL_FRAME_HEAD) return;
        buf[idx++] = rx;
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
    BallKick_UpdateInput(buf[1], buf[2], (int8_t)buf[3],
                         buf[4], buf[5]);
}
