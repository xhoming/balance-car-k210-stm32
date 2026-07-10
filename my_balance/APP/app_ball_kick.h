#ifndef __APP_BALL_KICK_H_
#define __APP_BALL_KICK_H_

#include "stm32f1xx_hal.h"

#define BALL_FRAME_HEAD 0xB6
#define BALL_FRAME_TAIL 0x6B

#define BALL_FLAG_RUNNING 0x01
#define BALL_FLAG_VALID   0x02

/* 连续 60 个 5ms 控制周期未收到完整 K210 帧，即判定通信超时 300ms。 */
#define BALL_INPUT_TIMEOUT_TICKS 60

typedef enum {
    BALL_ACTION_IDLE = 0,  /* B0：空闲，尚无有效的运行目标。 */
    BALL_ACTION_AIM = 1,   /* B1：对准，让小球回到画面中央。 */
    BALL_ACTION_CREEP = 2, /* B2：靠近，直到蓝球面积达到发射门槛。 */
    BALL_ACTION_ARMED = 3, /* B3：确认小球居中并连续满足冲刺条件。 */
    BALL_ACTION_KICK = 4,  /* B4：通过速度环执行平滑的梯形冲刺。 */
    BALL_ACTION_BRAKE = 5, /* B5：关闭前后驱动力，依靠惯性滑行减速。 */
    BALL_ACTION_DONE = 6   /* B6：保持停止，运行开关关闭再打开后复位。 */
} BallKickAction_t;

typedef struct
{
    uint8_t seq;
    uint8_t flags;
    int16_t error;       /* 小球中心偏差，范围 -100 到 100。 */
    uint8_t area_x10;    /* 蓝球像素面积，100 表示占全屏 10.0%。 */
    uint8_t confidence;
} BallKickInput_t;

extern volatile BallKickInput_t g_ball_input;
extern volatile int16_t g_ball_debug_v;
extern volatile int16_t g_ball_debug_t;
extern volatile uint8_t g_ball_debug_state;
extern volatile uint16_t g_ball_debug_missed;
extern volatile uint32_t g_ball_rx_bytes;
extern volatile uint32_t g_ball_rx_frames;
extern volatile uint32_t g_ball_rx_bad_chk;
extern volatile uint32_t g_ball_rx_bad_tail;

extern float BallKick_Align_Kp;
extern float BallKick_Creep_Kp;
extern float BallKick_Kick_Kp;
extern float BallKick_Turn_Kd;
extern float BallKick_Gyro_Direction;
extern float BallKick_Track_Max_PWM;
extern float BallKick_Max_PWM;

void BallKick_UpdateInput(uint8_t seq, uint8_t flags, int16_t error,
                          uint8_t area_x10, uint8_t confidence);
float BallKick_TurnCalc(float gyro_z);
float BallKick_SpeedTarget(void);
int BallKick_FilterVelocityPwm(int velocity_pwm);
void BallKick_Reset(void);
void BallKick_ParseByte(uint8_t rx);

#endif
