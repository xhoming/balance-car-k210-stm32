#include "app_vision_turn.h"
#include "AllHeader.h"

volatile VisionInput_t g_vision_input = {0, 0, 0, 0, 0};
volatile int16_t g_vision_debug_v = 0;
volatile int16_t g_vision_debug_t = 0;

/*
 * K210 frame:
 *   A5 error_i8 slope_i8 confidence speed checksum 5A
 *
 * Lightweight Stanley-style steering:
 *   heading_pwm = slope * Heading_Kp
 *   lateral_pwm = atan(error * Crosstrack_Gain / (speed + Speed_Soft))
 *                 * Crosstrack_Kp
 *   Turn_PWM = heading_pwm + lateral_pwm - gyro_z * Kd
 */
float Vision_Heading_Kp = 3.20f;
float Vision_Crosstrack_Kp = 260.0f;
float Vision_Crosstrack_Gain = 0.13f;
float Vision_Speed_Soft = 10.0f;
float Vision_Turn_Kd = 1.20f;
float Vision_Gyro_Direction = 1.0f;
float Vision_Max_PWM = 520.0f;

static int16_t vision_limiti16(int16_t value, int16_t min_value,
                               int16_t max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float vision_limitf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void VisionTurn_UpdateInput(int16_t error, int16_t slope, uint8_t confidence,
                            uint8_t base_speed)
{
    if (confidence > 100) confidence = 100;
    if (base_speed > 30) base_speed = 30;
    error = vision_limiti16(error, -100, 100);
    slope = vision_limiti16(slope, -100, 100);

    if (confidence < VISION_MIN_CONFIDENCE) {
        error = 0;
        slope = 0;
        base_speed = 0;
    }

    g_vision_input.error = error;
    g_vision_input.slope = slope;
    g_vision_input.confidence = confidence;
    g_vision_input.base_speed = base_speed;
    g_vision_input.last_update_ms = HAL_GetTick();
    g_vision_debug_v = base_speed;
    g_vision_debug_t = error;
}

float VisionTurn_Calc(float gyro_z)
{
    float gyro_feedback;
    float speed;
    float heading_pwm;
    float lateral_angle;
    float lateral_pwm;
    float turn_pwm;

    if (g_vision_input.confidence < VISION_MIN_CONFIDENCE) {
        g_vision_debug_t = 0;
        return 0.0f;
    }

    gyro_feedback = gyro_z * Vision_Gyro_Direction;
    speed = (float)g_vision_input.base_speed;
    if (speed < 1.0f) speed = 1.0f;

    heading_pwm = (float)g_vision_input.slope * Vision_Heading_Kp;
    lateral_angle = (float)atan2((double)((float)g_vision_input.error *
                                          Vision_Crosstrack_Gain),
                                 (double)(speed + Vision_Speed_Soft));
    lateral_pwm = lateral_angle * Vision_Crosstrack_Kp;

    turn_pwm = heading_pwm + lateral_pwm - gyro_feedback * Vision_Turn_Kd;
    turn_pwm = vision_limitf(turn_pwm, -Vision_Max_PWM, Vision_Max_PWM);

    g_vision_debug_t = (int16_t)turn_pwm;
    return turn_pwm;
}

float VisionSpeed_Target(float base_speed)
{
    if (g_vision_input.confidence < VISION_MIN_CONFIDENCE ||
        g_vision_input.base_speed == 0 || base_speed <= 0.0f) {
        g_vision_debug_v = 0;
        return 0.0f;
    }

    base_speed = (float)vision_limiti16((int16_t)base_speed, 0, 30);
    g_vision_debug_v = (int16_t)base_speed;
    return base_speed;
}

void VisionTurn_Reset(void)
{
    g_vision_input.error = 0;
    g_vision_input.slope = 0;
    g_vision_input.confidence = 0;
    g_vision_input.base_speed = 0;
    g_vision_input.last_update_ms = 0;
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
