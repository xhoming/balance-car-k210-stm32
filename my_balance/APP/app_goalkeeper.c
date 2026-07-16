#include "app_goalkeeper.h"
#include "AllHeader.h"

volatile GoalkeeperInput_t g_goalkeeper_input =
    {0, 0, 0, 0, GOALKEEPER_TTC_INVALID, 0, 0};
volatile uint8_t g_goalkeeper_debug_state = GOALKEEPER_ACTION_WAIT;
volatile int16_t g_goalkeeper_debug_turn = 0;
volatile int16_t g_goalkeeper_debug_speed_x10 = 0;
volatile uint16_t g_goalkeeper_debug_encoder_speed = 0;
volatile uint16_t g_goalkeeper_debug_age_ms = 999;
volatile uint16_t g_goalkeeper_debug_rx_count = 0;
volatile uint8_t g_goalkeeper_debug_gate = 0;
volatile int8_t g_goalkeeper_debug_lane = 0;

/*
 * 三位置守门：小车初始位于球门中央。
 * 球较远或保持在中央区域时原地观察；面积达到门槛且连续两帧
 * 位于同一侧后锁定方向，走一段固定弧线，随后站住。
 */
float Goalkeeper_Gyro_Kd = 1.2f;          /* 弧线转向阻尼；甩动时增大。 */
float Goalkeeper_Lane_Speed = 60.0f;      /* 速度PI命令量，不是实际车速上限。 */
float Goalkeeper_Lane_Turn_PWM = 642.0f;  /* 随前进速度同比提高，保持弧线曲率。 */

#define GOALKEEPER_INPUT_TIMEOUT_MS          800U /* 超过此时间无串口帧，立即停车。 */
#define GOALKEEPER_CONTROL_PERIOD_MS           5U
#define GOALKEEPER_INPUT_TIMEOUT_TICKS \
    (GOALKEEPER_INPUT_TIMEOUT_MS / GOALKEEPER_CONTROL_PERIOD_MS)

/*
 * K210误差范围为-100到100。球超过左右门槛后立即锁定方向。
 * 门槛越小越灵敏，也越容易被检测抖动误触发。
 */
#define GOALKEEPER_LANE_ERROR_LIMIT              10
#define GOALKEEPER_TRIGGER_AREA_X10               50U /* 50表示全屏面积5.0%。 */
#define GOALKEEPER_TRIGGER_CONFIDENCE             35U /* 单帧触发的最低置信度。 */
#define GOALKEEPER_LANE_CONFIRM_FRAMES             1U /* 满足门槛后立即锁定。 */
#define GOALKEEPER_SIDE_TURN_TICKS               47U /* 高速弧线47*5ms=0.235秒。 */
#define GOALKEEPER_SIDE_STRAIGHT_TICKS           18U /* 高速直行18*5ms=0.09秒。 */
#define GOALKEEPER_SIDE_TOTAL_TICKS \
    (GOALKEEPER_SIDE_TURN_TICKS + GOALKEEPER_SIDE_STRAIGHT_TICKS)

/*
 * 每5ms改变一次速度目标。加速值越大，起步越快；减速值过大会点头。
 */
#define GOALKEEPER_ACCEL_STEP                  5.30f
#define GOALKEEPER_DECEL_STEP                  0.53f

#define GOALKEEPER_TURN_SLEW_PWM             104.0f /* 按速度比例提高转向建立速率。 */

static volatile GoalkeeperInput_t goalkeeper_pending_input;
static volatile uint8_t goalkeeper_pending_ready = 0;

static uint8_t goalkeeper_action = GOALKEEPER_ACTION_WAIT;
static uint16_t goalkeeper_move_ticks = 0;
static int8_t goalkeeper_lane = 0;
static int8_t goalkeeper_candidate_lane = 0;
static uint8_t goalkeeper_candidate_frames = 0;
static uint16_t goalkeeper_input_age_ticks =
    GOALKEEPER_INPUT_TIMEOUT_TICKS + 1U;
static float goalkeeper_speed_target = 0.0f;
static float goalkeeper_encoder_speed = 0.0f;
static float goalkeeper_last_turn_pwm = 0.0f;

static float goalkeeper_limitf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static void goalkeeper_set_state(uint8_t state)
{
    goalkeeper_action = state;
    g_goalkeeper_debug_state = state;
    if (state != GOALKEEPER_ACTION_MOVE) {
        goalkeeper_last_turn_pwm = 0.0f;
        g_goalkeeper_debug_turn = 0;
    }
}

static uint8_t goalkeeper_consume_pending(void)
{
    GoalkeeperInput_t input;

    if (!goalkeeper_pending_ready) return 0;

    input.seq = goalkeeper_pending_input.seq;
    input.flags = goalkeeper_pending_input.flags;
    input.error = goalkeeper_pending_input.error;
    input.area_x10 = goalkeeper_pending_input.area_x10;
    input.ttc_10ms = goalkeeper_pending_input.ttc_10ms;
    input.confidence = goalkeeper_pending_input.confidence;
    input.last_update_ms = goalkeeper_pending_input.last_update_ms;
    goalkeeper_pending_ready = 0;

    g_goalkeeper_input.seq = input.seq;
    g_goalkeeper_input.flags = input.flags;
    g_goalkeeper_input.error = input.error;
    g_goalkeeper_input.area_x10 = input.area_x10;
    g_goalkeeper_input.ttc_10ms = input.ttc_10ms;
    g_goalkeeper_input.confidence = input.confidence;
    g_goalkeeper_input.last_update_ms = input.last_update_ms;
    return 1;
}

static uint8_t goalkeeper_input_is_fresh(void)
{
    return goalkeeper_input_age_ticks <= GOALKEEPER_INPUT_TIMEOUT_TICKS;
}

static uint8_t goalkeeper_input_has_target(void)
{
    return goalkeeper_input_is_fresh() &&
           (g_goalkeeper_input.flags & GOALKEEPER_FLAG_VALID) != 0U;
}

static void goalkeeper_start_move(int8_t lane)
{
    goalkeeper_lane = lane;
    g_goalkeeper_debug_lane = lane;
    goalkeeper_candidate_lane = 0;
    goalkeeper_candidate_frames = 0;
    goalkeeper_move_ticks = 0;
    goalkeeper_set_state(GOALKEEPER_ACTION_MOVE);
}

static void goalkeeper_update_candidate(int8_t lane)
{
    if (lane == 0) {
        goalkeeper_candidate_lane = 0;
        goalkeeper_candidate_frames = 0;
        return;
    }

    if (lane != goalkeeper_candidate_lane) {
        goalkeeper_candidate_lane = lane;
        goalkeeper_candidate_frames = 1;
    } else if (goalkeeper_candidate_frames < 255U) {
        goalkeeper_candidate_frames++;
    }

    if (goalkeeper_candidate_frames >= GOALKEEPER_LANE_CONFIRM_FRAMES) {
        goalkeeper_start_move(lane);
    }
}

static float goalkeeper_desired_speed(void)
{
    return (goalkeeper_action == GOALKEEPER_ACTION_MOVE) ?
           Goalkeeper_Lane_Speed : 0.0f;
}

static void goalkeeper_update_speed_target(float desired)
{
    desired = goalkeeper_limitf(desired, 0.0f, Goalkeeper_Lane_Speed);
    if (desired > goalkeeper_speed_target) {
        goalkeeper_speed_target += GOALKEEPER_ACCEL_STEP;
        if (goalkeeper_speed_target > desired) {
            goalkeeper_speed_target = desired;
        }
    } else if (desired < goalkeeper_speed_target) {
        goalkeeper_speed_target -= GOALKEEPER_DECEL_STEP;
        if (goalkeeper_speed_target < desired) {
            goalkeeper_speed_target = desired;
        }
    }

    g_goalkeeper_debug_speed_x10 =
        (int16_t)(goalkeeper_speed_target * 10.0f + 0.5f);
}

void Goalkeeper_UpdateInput(uint8_t seq, uint8_t flags, int16_t error,
                            uint8_t area_x10, uint8_t ttc_10ms,
                            uint8_t confidence)
{
    goalkeeper_pending_ready = 0;
    goalkeeper_pending_input.seq = seq;
    goalkeeper_pending_input.flags = flags;
    goalkeeper_pending_input.error = error;
    goalkeeper_pending_input.area_x10 = area_x10;
    goalkeeper_pending_input.ttc_10ms = ttc_10ms;
    goalkeeper_pending_input.confidence =
        (confidence > 100U) ? 100U : confidence;
    goalkeeper_pending_input.last_update_ms = HAL_GetTick();
    goalkeeper_pending_ready = 1;
    g_goalkeeper_debug_rx_count++;
    if (g_goalkeeper_debug_rx_count > 999U) {
        g_goalkeeper_debug_rx_count = 0;
    }
}

void Goalkeeper_Update(float gyro_z, int encoder_left, int encoder_right)
{
    uint32_t age;
    int encoder_sum;
    float encoder_raw;
    uint8_t new_input;
    uint8_t running;
    uint8_t fresh;
    uint8_t target_now;

    (void)gyro_z;
    new_input = goalkeeper_consume_pending();
    if (new_input) {
        goalkeeper_input_age_ticks = 0U;
    } else if (goalkeeper_input_age_ticks < 0xFFFFU) {
        goalkeeper_input_age_ticks++;
    }
    age = (uint32_t)goalkeeper_input_age_ticks *
          GOALKEEPER_CONTROL_PERIOD_MS;
    if (age > 999U) age = 999U;
    g_goalkeeper_debug_age_ms = (uint16_t)age;

    encoder_sum = encoder_left + encoder_right;
    if (encoder_sum < 0) encoder_sum = -encoder_sum;
    encoder_raw = (float)encoder_sum * 0.5f;
    goalkeeper_encoder_speed = goalkeeper_encoder_speed * 0.70f +
                               encoder_raw * 0.30f;
    g_goalkeeper_debug_encoder_speed =
        (uint16_t)(goalkeeper_encoder_speed + 0.5f);

    fresh = goalkeeper_input_is_fresh();
    running = fresh &&
              (g_goalkeeper_input.flags & GOALKEEPER_FLAG_RUNNING) != 0U;
    target_now = running && goalkeeper_input_has_target();
    g_goalkeeper_debug_gate = (fresh ? 1U : 0U) |
                              (running ? 2U : 0U) |
                              (target_now ? 4U : 0U);

    if (!running) {
        goalkeeper_move_ticks = 0;
        goalkeeper_lane = 0;
        goalkeeper_candidate_lane = 0;
        goalkeeper_candidate_frames = 0;
        g_goalkeeper_debug_lane = 0;
        goalkeeper_set_state(GOALKEEPER_ACTION_WAIT);
        goalkeeper_update_speed_target(0.0f);
        return;
    }

    if (new_input) {
        if (target_now) {
            if (goalkeeper_action == GOALKEEPER_ACTION_WAIT) {
                goalkeeper_set_state(GOALKEEPER_ACTION_DECIDE);
            }

            if (goalkeeper_action == GOALKEEPER_ACTION_DECIDE) {
                if (g_goalkeeper_input.area_x10 <
                        GOALKEEPER_TRIGGER_AREA_X10 ||
                    g_goalkeeper_input.confidence <
                        GOALKEEPER_TRIGGER_CONFIDENCE) {
                    /* 球还远时只观察，不积累左右触发帧。 */
                    goalkeeper_update_candidate(0);
                } else if (g_goalkeeper_input.error <=
                    -GOALKEEPER_LANE_ERROR_LIMIT) {
                    goalkeeper_update_candidate(-1);
                } else if (g_goalkeeper_input.error >=
                           GOALKEEPER_LANE_ERROR_LIMIT) {
                    goalkeeper_update_candidate(1);
                } else {
                    goalkeeper_update_candidate(0);
                }
            }
        } else if (goalkeeper_action == GOALKEEPER_ACTION_DECIDE) {
            goalkeeper_update_candidate(0);
        }
    }

    if (goalkeeper_action == GOALKEEPER_ACTION_MOVE) {
        goalkeeper_move_ticks++;
        if (goalkeeper_move_ticks >= GOALKEEPER_SIDE_TOTAL_TICKS) {
            goalkeeper_set_state(GOALKEEPER_ACTION_HOLD);
        }
    }

    goalkeeper_update_speed_target(goalkeeper_desired_speed());
}

float Goalkeeper_SpeedTarget(void)
{
    return goalkeeper_speed_target;
}

float Goalkeeper_TurnCalc(float gyro_z)
{
    float turn_pwm;

    if (goalkeeper_action != GOALKEEPER_ACTION_MOVE ||
        goalkeeper_lane == 0 ||
        goalkeeper_move_ticks >= GOALKEEPER_SIDE_TURN_TICKS) {
        goalkeeper_last_turn_pwm = 0.0f;
        g_goalkeeper_debug_turn = 0;
        return 0.0f;
    }

    turn_pwm = (float)goalkeeper_lane * Goalkeeper_Lane_Turn_PWM -
               gyro_z * Goalkeeper_Gyro_Kd;
    turn_pwm = goalkeeper_limitf(turn_pwm,
                                 -Goalkeeper_Lane_Turn_PWM,
                                 Goalkeeper_Lane_Turn_PWM);
    turn_pwm = goalkeeper_limitf(turn_pwm,
                                 goalkeeper_last_turn_pwm -
                                     GOALKEEPER_TURN_SLEW_PWM,
                                 goalkeeper_last_turn_pwm +
                                     GOALKEEPER_TURN_SLEW_PWM);
    goalkeeper_last_turn_pwm = turn_pwm;
    g_goalkeeper_debug_turn = (int16_t)turn_pwm;
    return turn_pwm;
}

int Goalkeeper_FilterVelocityPwm(int velocity_pwm)
{
    return velocity_pwm;
}

void Goalkeeper_Reset(void)
{
    g_goalkeeper_input.seq = 0;
    g_goalkeeper_input.flags = 0;
    g_goalkeeper_input.error = 0;
    g_goalkeeper_input.area_x10 = 0;
    g_goalkeeper_input.ttc_10ms = GOALKEEPER_TTC_INVALID;
    g_goalkeeper_input.confidence = 0;
    g_goalkeeper_input.last_update_ms = 0;
    g_goalkeeper_debug_state = GOALKEEPER_ACTION_WAIT;
    g_goalkeeper_debug_turn = 0;
    g_goalkeeper_debug_speed_x10 = 0;
    g_goalkeeper_debug_encoder_speed = 0;
    g_goalkeeper_debug_age_ms = 999;
    g_goalkeeper_debug_rx_count = 0;
    g_goalkeeper_debug_gate = 0;
    g_goalkeeper_debug_lane = 0;

    goalkeeper_pending_ready = 0;
    goalkeeper_action = GOALKEEPER_ACTION_WAIT;
    goalkeeper_move_ticks = 0;
    goalkeeper_lane = 0;
    goalkeeper_candidate_lane = 0;
    goalkeeper_candidate_frames = 0;
    goalkeeper_input_age_ticks = GOALKEEPER_INPUT_TIMEOUT_TICKS + 1U;
    goalkeeper_speed_target = 0.0f;
    goalkeeper_encoder_speed = 0.0f;
    goalkeeper_last_turn_pwm = 0.0f;
    Velocity_PI_Reset();
}

void Goalkeeper_ParseByte(uint8_t rx)
{
    static uint8_t buf[9];
    static uint8_t idx = 0;
    uint8_t chk;

    if (idx == 0U) {
        if (rx != GOALKEEPER_FRAME_HEAD) return;
        buf[idx++] = rx;
        return;
    }

    buf[idx++] = rx;
    if (idx < sizeof(buf)) return;
    idx = 0;

    if (buf[8] != GOALKEEPER_FRAME_TAIL) return;
    chk = (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4] +
                    buf[5] + buf[6]);
    if (chk != buf[7]) return;

    Goalkeeper_UpdateInput(buf[1], buf[2], (int8_t)buf[3],
                           buf[4], buf[5], buf[6]);
}
