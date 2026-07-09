#ifndef __APP_BALL_KICK_H_
#define __APP_BALL_KICK_H_

#include "stm32f1xx_hal.h"

#define BALL_FRAME_HEAD 0xB6
#define BALL_FRAME_TAIL 0x6B
#define BALL_MIN_CONFIDENCE 20
#define BALL_INPUT_TIMEOUT_MS 800

typedef enum {
    BALL_STATE_SEARCH = 0,
    BALL_STATE_ALIGN = 1,
    BALL_STATE_APPROACH = 2,
    BALL_STATE_CHARGE = 3,
    BALL_STATE_BRAKE = 4
} BallKickState_t;

typedef struct
{
    uint8_t state;       /* K210 measurement state: search/seen/brake */
    int16_t error;       /* ball center error: -100..100 */
    uint8_t area;        /* near score from box size: 0..100 */
    uint8_t confidence;
    uint8_t speed;       /* y score in image: 0..100, kept for frame compatibility */
    uint32_t last_update_ms;
} BallKickInput_t;

extern volatile BallKickInput_t g_ball_input;
extern volatile int16_t g_ball_debug_v;
extern volatile int16_t g_ball_debug_t;
extern volatile uint8_t g_ball_debug_state;
extern volatile uint32_t g_ball_rx_bytes;
extern volatile uint32_t g_ball_rx_frames;
extern volatile uint32_t g_ball_rx_bad_chk;
extern volatile uint32_t g_ball_rx_bad_tail;
extern volatile uint8_t g_ball_last_rx;

extern float BallKick_Align_Kp;
extern float BallKick_Approach_Kp;
extern float BallKick_Charge_Kp;
extern float BallKick_Turn_Kd;
extern float BallKick_Gyro_Direction;
extern float BallKick_Max_PWM;

void BallKick_UpdateInput(uint8_t state, int16_t error, uint8_t area,
                          uint8_t confidence, uint8_t speed);
float BallKick_TurnCalc(float gyro_z);
float BallKick_SpeedTarget(void);
void BallKick_Reset(void);
void BallKick_ParseByte(uint8_t rx);

#endif
