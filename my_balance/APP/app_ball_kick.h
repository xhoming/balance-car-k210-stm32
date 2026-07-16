#ifndef __APP_BALL_KICK_H_
#define __APP_BALL_KICK_H_

#include "stm32f1xx_hal.h"

#define BALL_FRAME_HEAD 0xB6
#define BALL_FRAME_TAIL 0x6B

#define BALL_FLAG_RUNNING 0x01
#define BALL_FLAG_VALID   0x02

#define BALL_INPUT_TIMEOUT_MS 300U

typedef enum {
    BALL_ACTION_STOP = 0,
    BALL_ACTION_TRACK = 1,
    BALL_ACTION_STRIKE = 2,
    BALL_ACTION_RECOVER = 3
} BallKickAction_t;

typedef struct
{
    uint8_t seq;
    uint8_t flags;
    int16_t error;
    uint8_t area_x10;
    uint8_t confidence;
    uint32_t last_update_ms;
} BallKickInput_t;

extern volatile BallKickInput_t g_ball_input;
extern volatile int16_t g_ball_debug_v;
extern volatile int16_t g_ball_debug_t;
extern volatile uint8_t g_ball_debug_state;
extern volatile uint8_t g_ball_debug_area_x10;
extern volatile uint16_t g_ball_debug_missed;
extern volatile uint32_t g_ball_rx_bytes;
extern volatile uint32_t g_ball_rx_frames;
extern volatile uint32_t g_ball_rx_bad_chk;
extern volatile uint32_t g_ball_rx_bad_tail;

extern float BallKick_Track_Kp;
extern float BallKick_Kick_Kp;
extern float BallKick_Turn_Kd;
extern float BallKick_Gyro_Direction;
extern float BallKick_Track_Max_PWM;
extern float BallKick_Kick_Max_PWM;

void BallKick_UpdateInput(uint8_t seq, uint8_t flags, int16_t error,
                          uint8_t area_x10, uint8_t confidence);
float BallKick_TurnCalc(float gyro_z);
float BallKick_SpeedTarget(void);
int BallKick_FilterVelocityPwm(int velocity_pwm);
void BallKick_Reset(void);
void BallKick_ParseByte(uint8_t rx);

#endif
