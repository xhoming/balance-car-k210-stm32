#ifndef __APP_VISION_TURN_H_
#define __APP_VISION_TURN_H_

#include "stm32f1xx_hal.h"

#define VISION_FRAME_HEAD 0xA5
#define VISION_FRAME_TAIL 0x5A
#define VISION_MIN_CONFIDENCE 15

typedef struct
{
    int16_t error;
    int16_t slope;
    uint8_t confidence;
    uint8_t base_speed;
    uint32_t last_update_ms;
} VisionInput_t;

extern volatile VisionInput_t g_vision_input;
extern volatile float g_vision_target_speed;
extern volatile float g_vision_turn_pwm;
extern volatile int16_t g_vision_debug_v;
extern volatile int16_t g_vision_debug_t;
extern volatile uint32_t g_vision_frame_count;

extern float Vision_Max_PWM;
extern float Vision_Slope_Slowdown;
extern float Vision_Min_Run_Speed;
extern float Vision_Min_Curve_Speed;
extern float Vision_Min_Hard_Curve_Speed;
extern float Vision_Hard_Curve_Threshold;
extern float Vision_Hard_Curve_Boost;

void VisionTurn_UpdateInput(int16_t error, int16_t slope, uint8_t confidence,
                            uint8_t base_speed);
float VisionTurn_Calc(float gyro_z);
float VisionSpeed_Target(float base_speed);
void VisionTurn_Reset(void);
void VisionTurn_ParseByte(uint8_t rx);

#endif
