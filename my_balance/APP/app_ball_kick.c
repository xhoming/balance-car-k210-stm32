#include "app_ball_kick.h"
#include "AllHeader.h"

volatile BallKickInput_t g_ball_input = {0, 0, 0, 0, 0};
volatile int16_t g_ball_debug_v = 0;
volatile int16_t g_ball_debug_t = 0;
volatile uint8_t g_ball_debug_state = BALL_ACTION_IDLE;
volatile uint16_t g_ball_debug_missed = 0;
volatile uint32_t g_ball_rx_bytes = 0;
volatile uint32_t g_ball_rx_frames = 0;
volatile uint32_t g_ball_rx_bad_chk = 0;
volatile uint32_t g_ball_rx_bad_tail = 0;

/*
 * K210 通信帧，共 8 字节：
 *   B6 seq flags error_i8 area_x10 confidence checksum 6B
 *   checksum = seq + flags + error_u + area_x10 + confidence
 *
 * K210 只负责识别和测量，STM32 负责完整的一次性撞球动作。
 */
/*
 * 转向控制公式：
 *   turn_pwm = error * Kp - gyro_z * Gyro_Direction * Turn_Kd
 * error 为正表示小球位于画面右侧，error 为负表示小球位于画面左侧。
 */
float BallKick_Align_Kp = 1.8f;  /* B1 对准增益；增大后朝小球转得更快。 */
float BallKick_Creep_Kp = 1.2f;  /* B2 靠近增益；控制靠近过程中的方向修正。 */
float BallKick_Kick_Kp = 0.6f;   /* B4 冲刺增益；控制撞球过程中的方向保持。 */
float BallKick_Turn_Kd = 1.10f;  /* 陀螺仪阻尼；增大可减小转过头和摆动。 */
float BallKick_Gyro_Direction = 1.0f; /* 仅当陀螺仪阻尼方向相反时改为 -1。 */
float BallKick_Track_Max_PWM = 45.0f; /* B1/B2 转向上限，同时限制低速前进补偿。 */
float BallKick_Max_PWM = 200.0f; /* B4 冲刺期间的转向 PWM 绝对值上限。 */

/* 图像判断门槛，error 的范围为 -100 到 100。 */
#define BALL_AIM_ERROR_LIMIT       22 /* 偏差超过该值时进入 B1 对准状态。 */
#define BALL_KICK_ERROR_LIMIT      14 /* 偏差小于该值才允许进入冲刺确认。 */
#define BALL_LAUNCH_AREA_MIN_X10   120 /* 120 表示蓝球像素占全屏面积的 12.0%。 */
#define BALL_ARM_CONFIRM_FRAMES     3 /* 连续多少张不同的有效图像后触发冲刺。 */
#define BALL_TURN_DEADBAND           5 /* 中心死区，忽略小偏差以防止左右抖动。 */
#define BALL_TURN_SLEW_PWM         4.0f /* 每个 5ms 周期允许的最大转向 PWM 变化量。 */

/* 运动参数；V 是速度环目标值，不是实际的米每秒。 */
#define BALL_AIM_SPEED              0.01f /* B1 由单轮差速慢速对准，不额外要求前进。 */
#define BALL_CREEP_SPEED            0.01f /* B2 对准后向小球缓慢靠近的速度目标。 */
#define BALL_KICK_SPEED              5.0f /* B4 速度环冲刺的峰值目标。 */
#define BALL_KICK_TICKS             160U /* 160 * 5ms = 0.8 秒总冲刺时间。 */
#define BALL_KICK_RAMP_TICKS         60U /* 60 * 5ms = 0.3 秒加速/减速时间。 */
#define BALL_BRAKE_TICKS            500U /* 500 * 5ms = 2.5 秒无驱动滑行时间。 */

static uint8_t ball_action_state = BALL_ACTION_IDLE;
static uint8_t ball_last_seq = 0;
static uint8_t ball_seq_initialized = 0;
static uint8_t ball_arm_frames = 0;
static uint8_t ball_was_running = 0;
static volatile uint8_t ball_has_valid_input = 0;
static uint16_t ball_action_ticks = 0;
static uint16_t ball_missed_ticks = BALL_INPUT_TIMEOUT_TICKS + 1U;
static uint32_t ball_last_rx_frames = 0;
static int16_t ball_kick_error = 0;
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

static void ball_update_watchdog(void)
{
    uint32_t rx_frames = g_ball_rx_frames;

    if (rx_frames != ball_last_rx_frames) {
        ball_last_rx_frames = rx_frames;
        ball_missed_ticks = 0;
    } else if (ball_missed_ticks <= BALL_INPUT_TIMEOUT_TICKS) {
        ball_missed_ticks++;
    }
    g_ball_debug_missed = ball_missed_ticks;
}

static uint8_t ball_input_running(void)
{
    return ball_missed_ticks <= BALL_INPUT_TIMEOUT_TICKS &&
           ((g_ball_input.flags & BALL_FLAG_RUNNING) != 0U);
}

static uint8_t ball_input_valid(void)
{
    return ball_input_running() && ball_has_valid_input;
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

static void ball_set_state(uint8_t state)
{
    ball_action_state = state;
    g_ball_debug_state = state;
}

static void ball_reset_arming(void)
{
    ball_arm_frames = 0;
    ball_kick_error = 0;
}

static void ball_update_action_state(void)
{
    int16_t abs_error;

    ball_update_watchdog();

    if (!ball_input_running()) {
        ball_has_valid_input = 0;
        ball_was_running = 0;
        ball_seq_initialized = 0;
        ball_reset_arming();
        ball_set_state(BALL_ACTION_IDLE);
        return;
    }

    if (!ball_was_running) {
        ball_was_running = 1;
        ball_seq_initialized = 0;
        ball_reset_arming();
        ball_set_state(BALL_ACTION_IDLE);
    }

    if (ball_action_state == BALL_ACTION_KICK) {
        if (ball_action_ticks > 0U) ball_action_ticks--;
        if (ball_action_ticks > 0U) return;
        ball_action_ticks = BALL_BRAKE_TICKS;
        ball_set_state(BALL_ACTION_BRAKE);
        return;
    }

    if (ball_action_state == BALL_ACTION_BRAKE) {
        if (ball_action_ticks > 0U) ball_action_ticks--;
        if (ball_action_ticks > 0U) return;
        ball_set_state(BALL_ACTION_DONE);
        return;
    }

    if (ball_action_state == BALL_ACTION_DONE) return;

    if (!ball_input_valid()) {
        ball_reset_arming();
        ball_set_state(BALL_ACTION_IDLE);
        return;
    }

    if (!ball_take_new_seq()) return;

    abs_error = ball_limiti16(g_ball_input.error, -100, 100);
    if (abs_error < 0) abs_error = -abs_error;

    if (abs_error > BALL_AIM_ERROR_LIMIT) {
        ball_reset_arming();
        ball_set_state(BALL_ACTION_AIM);
        return;
    }

    if (g_ball_input.area_x10 < BALL_LAUNCH_AREA_MIN_X10) {
        ball_reset_arming();
        ball_set_state(BALL_ACTION_CREEP);
        return;
    }

    if (abs_error > BALL_KICK_ERROR_LIMIT) {
        ball_reset_arming();
        ball_set_state(BALL_ACTION_AIM);
        return;
    }

    ball_set_state(BALL_ACTION_ARMED);
    if (ball_arm_frames < BALL_ARM_CONFIRM_FRAMES) ball_arm_frames++;
    if (ball_arm_frames >= BALL_ARM_CONFIRM_FRAMES) {
        ball_kick_error = g_ball_input.error;
        ball_action_ticks = BALL_KICK_TICKS;
        ball_set_state(BALL_ACTION_KICK);
    }
}

void BallKick_UpdateInput(uint8_t seq, uint8_t flags, int16_t error,
                          uint8_t area_x10, uint8_t confidence)
{
    g_ball_input.flags = flags;

    if ((flags & BALL_FLAG_RUNNING) == 0U) {
        ball_has_valid_input = 0;
    }

    if ((flags & BALL_FLAG_VALID) == 0U) {
        return;
    }

    g_ball_input.error = ball_limiti16(error, -100, 100);
    g_ball_input.area_x10 = area_x10;
    g_ball_input.confidence = (confidence > 100U) ? 100U : confidence;
    g_ball_input.seq = seq;
    ball_has_valid_input = 1;
}

float BallKick_TurnCalc(float gyro_z)
{
    float kp;
    float error;
    float max_pwm;
    float turn_pwm;
    float gyro_feedback;

    if (ball_action_state == BALL_ACTION_IDLE ||
        ball_action_state == BALL_ACTION_ARMED ||
        ball_action_state == BALL_ACTION_BRAKE ||
        ball_action_state == BALL_ACTION_DONE) {
        ball_last_turn_pwm = 0.0f;
        g_ball_debug_t = 0;
        return 0.0f;
    }

    if (ball_action_state == BALL_ACTION_KICK) {
        kp = BallKick_Kick_Kp;
        max_pwm = BallKick_Max_PWM;
        error = (float)ball_kick_error;
    } else if (ball_action_state == BALL_ACTION_CREEP) {
        kp = BallKick_Creep_Kp;
        max_pwm = BallKick_Track_Max_PWM;
        error = (float)g_ball_input.error;
    } else {
        kp = BallKick_Align_Kp;
        max_pwm = BallKick_Track_Max_PWM;
        error = (float)g_ball_input.error;
    }

    if (error > -(float)BALL_TURN_DEADBAND &&
        error < (float)BALL_TURN_DEADBAND) {
        error = 0.0f;
    }

    gyro_feedback = gyro_z * BallKick_Gyro_Direction;
    turn_pwm = error * kp - gyro_feedback * BallKick_Turn_Kd;
    if ((g_ball_input.flags & BALL_FLAG_VALID) == 0U) {
        turn_pwm = 0.0f;
    }
    turn_pwm = ball_limitf(turn_pwm, -max_pwm, max_pwm);
    turn_pwm = ball_limitf(turn_pwm,
                           ball_last_turn_pwm - BALL_TURN_SLEW_PWM,
                           ball_last_turn_pwm + BALL_TURN_SLEW_PWM);
    ball_last_turn_pwm = turn_pwm;
    g_ball_debug_t = (int16_t)ball_last_turn_pwm;
    return ball_last_turn_pwm;
}

float BallKick_SpeedTarget(void)
{
    uint16_t ramp_ticks;
    float kick_speed;

    ball_update_action_state();

    if (ball_action_state == BALL_ACTION_KICK) {
        ramp_ticks = BALL_KICK_TICKS - ball_action_ticks;
        if (ball_action_ticks < ramp_ticks) {
            ramp_ticks = ball_action_ticks;
        }

        if (ramp_ticks < BALL_KICK_RAMP_TICKS) {
            kick_speed = BALL_KICK_SPEED * (float)ramp_ticks /
                         (float)BALL_KICK_RAMP_TICKS;
        } else {
            kick_speed = BALL_KICK_SPEED;
        }

        g_ball_debug_v = (int16_t)kick_speed;
        return kick_speed;
    }

    if (ball_action_state == BALL_ACTION_AIM) {
        if (g_ball_input.area_x10 < BALL_LAUNCH_AREA_MIN_X10) {
            g_ball_debug_v = (int16_t)BALL_AIM_SPEED;
            return BALL_AIM_SPEED;
        }
        g_ball_debug_v = 0;
        return 0.0f;
    }

    if (ball_action_state == BALL_ACTION_CREEP) {
        g_ball_debug_v = (int16_t)BALL_CREEP_SPEED;
        return BALL_CREEP_SPEED;
    }

    g_ball_debug_v = 0;
    return 0.0f;
}

int BallKick_FilterVelocityPwm(int velocity_pwm)
{
    if (ball_action_state == BALL_ACTION_BRAKE ||
        ball_action_state == BALL_ACTION_DONE) {
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
    g_ball_debug_v = 0;
    g_ball_debug_t = 0;
    g_ball_debug_state = BALL_ACTION_IDLE;
    g_ball_debug_missed = 0;
    ball_action_state = BALL_ACTION_IDLE;
    ball_last_seq = 0;
    ball_seq_initialized = 0;
    ball_arm_frames = 0;
    ball_was_running = 0;
    ball_has_valid_input = 0;
    ball_action_ticks = 0;
    ball_missed_ticks = BALL_INPUT_TIMEOUT_TICKS + 1U;
    ball_last_rx_frames = 0;
    ball_kick_error = 0;
    ball_last_turn_pwm = 0.0f;
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

    if (idx == 0) {
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
