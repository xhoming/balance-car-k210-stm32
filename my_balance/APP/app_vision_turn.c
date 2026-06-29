#include "app_vision_turn.h"
#include "AllHeader.h"

volatile VisionInput_t g_vision_input = {0, 0, 0, 0, 0};
volatile float g_vision_target_speed = 0.0f;
volatile float g_vision_turn_pwm = 0.0f;
volatile int16_t g_vision_debug_v = 0;
volatile int16_t g_vision_debug_t = 0;
volatile uint32_t g_vision_frame_count = 0;

float Vision_Base_Speed = 10.0f;
float Vision_Kp_Visual = 0.75f;
float Vision_Kd_Visual = 0.18f;
float Vision_Kff_Slope = 0.85f;

float Vision_Kp_Gyro = 0.0f;
float Vision_Ki_Gyro = 0.0f;
float Vision_Kd_Gyro = 0.0f;

float Vision_Max_YawRate = 95.0f;
float Vision_Max_PWM = 285.0f;
float Vision_Int_Limit = 180.0f;

float Vision_Error_Slowdown = 0.035f;
float Vision_Slope_Slowdown = 0.140f;
float Vision_Min_Run_Speed = 6.0f;
float Vision_Min_Curve_Speed = 0.0f;
float Vision_Min_Hard_Curve_Speed = 0.0f;

float Vision_Hard_Curve_Threshold = 48.0f;
float Vision_Hard_Curve_Boost = 0.62f;

static int16_t vision_absi16(int16_t value)
{
    return value < 0 ? -value : value;
}

static int16_t vision_limiti16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static int16_t vision_calc_speed_i16(int16_t error, int16_t slope, int16_t base_speed)
{
    int16_t error_abs = vision_absi16(error);
    int16_t slope_abs = vision_absi16(slope);
    int16_t min_speed = (int16_t)Vision_Min_Run_Speed;
    int16_t speed;

    speed = base_speed
          - (int16_t)((error_abs * 50 + slope_abs * 150) / 1000);

    if (slope_abs > 55 || error_abs > 70) {
        min_speed = (int16_t)Vision_Min_Hard_Curve_Speed;
        speed = min_speed;
    } else if (slope_abs > 35 || error_abs > 45) {
        min_speed = (int16_t)Vision_Min_Curve_Speed;
    }

    return vision_limiti16(speed, min_speed, base_speed);
}

static int16_t vision_calc_turn_i16(int16_t error, int16_t slope)
{
    int32_t turn;
    int16_t turn_abs;
    int16_t boost;
    int16_t sign;

    turn = (int32_t)((error * 110 + slope * 155) / 100);
    sign = turn >= 0 ? 1 : -1;
    turn_abs = (int16_t)(turn >= 0 ? turn : -turn);

    if (turn_abs > (int16_t)Vision_Hard_Curve_Threshold) {
        boost = (int16_t)((turn_abs - (int16_t)Vision_Hard_Curve_Threshold) *
                          Vision_Hard_Curve_Boost);
        turn += sign * boost;
    }

    return vision_limiti16((int16_t)turn, (int16_t)-Vision_Max_PWM,
                           (int16_t)Vision_Max_PWM);
}

void VisionTurn_UpdateInput(int16_t error, int16_t slope, uint8_t confidence,
                            uint8_t base_speed)
{
    if (confidence > 100) confidence = 100;
    if (base_speed > 30) base_speed = 30;
    if (error > 100) error = 100;
    if (error < -100) error = -100;
    if (slope > 100) slope = 100;
    if (slope < -100) slope = -100;

    g_vision_input.error = error;
    g_vision_input.slope = slope;
    g_vision_input.confidence = confidence;
    g_vision_input.base_speed = base_speed;
    g_vision_input.last_update_ms = HAL_GetTick();
    g_vision_frame_count++;

    if (confidence < VISION_MIN_CONFIDENCE) {
        g_vision_target_speed = 0.0f;
        g_vision_turn_pwm = 0.0f;
        g_vision_debug_v = 0;
        g_vision_debug_t = 0;
        return;
    }

    g_vision_debug_v = vision_calc_speed_i16(error, slope, (int16_t)Vision_Base_Speed);
    g_vision_debug_t = vision_calc_turn_i16(error, slope);
    g_vision_target_speed = (float)g_vision_debug_v;
    g_vision_turn_pwm = (float)g_vision_debug_t;
}

float VisionTurn_Calc(float gyro_z)
{
    int16_t turn_pwm;

    (void)gyro_z;

    if (g_vision_input.confidence < VISION_MIN_CONFIDENCE) {
        g_vision_turn_pwm = 0.0f;
        g_vision_debug_t = 0;
        return 0.0f;
    }

    turn_pwm = vision_calc_turn_i16(g_vision_input.error, g_vision_input.slope);
    g_vision_turn_pwm = (float)turn_pwm;
    g_vision_debug_t = turn_pwm;
    return (float)turn_pwm;
}

float VisionSpeed_Target(float base_speed)
{
    int16_t speed;

    if (g_vision_input.confidence < VISION_MIN_CONFIDENCE) {
        g_vision_target_speed = 0.0f;
        g_vision_debug_v = 0;
        return 0.0f;
    }

    Vision_Base_Speed = base_speed;
    speed = vision_calc_speed_i16(g_vision_input.error,
                                  g_vision_input.slope,
                                  (int16_t)base_speed);
    g_vision_target_speed = (float)speed;
    g_vision_debug_v = speed;
    return g_vision_target_speed;
}

void VisionTurn_Reset(void)
{
    g_vision_input.error = 0;
    g_vision_input.slope = 0;
    g_vision_input.confidence = 0;
    g_vision_input.base_speed = 0;
    g_vision_input.last_update_ms = 0;
    g_vision_frame_count = 0;
    g_vision_target_speed = 0.0f;
    g_vision_turn_pwm = 0.0f;
    g_vision_debug_v = 0;
    g_vision_debug_t = 0;
}

void VisionTurn_ParseByte(uint8_t rx)
{
    static uint8_t buf[7];
    static uint8_t idx = 0;
    uint8_t chk;

    if (idx == 0) {
        if (rx != VISION_FRAME_HEAD) return;
        idx = 0;
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
    if (buf[6] != VISION_FRAME_TAIL) return;

    chk = (uint8_t)(buf[1] + buf[2] + buf[3] + buf[4]);
    if (chk != buf[5]) return;

    VisionTurn_UpdateInput((int8_t)buf[1], (int8_t)buf[2], buf[3], buf[4]);
}
