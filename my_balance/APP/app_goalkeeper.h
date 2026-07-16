#ifndef __APP_GOALKEEPER_H_
#define __APP_GOALKEEPER_H_

#include "stm32f1xx_hal.h"

/* Protocol: B8 seq flags error area_x10 reserved confidence chk 8B. */
#define GOALKEEPER_FRAME_HEAD 0xB8
#define GOALKEEPER_FRAME_TAIL 0x8B

#define GOALKEEPER_FLAG_RUNNING   0x01
#define GOALKEEPER_FLAG_VALID     0x02
#define GOALKEEPER_FLAG_TTC_VALID 0x04
#define GOALKEEPER_TTC_INVALID    0xFF

typedef enum {
    GOALKEEPER_ACTION_WAIT = 0,
    GOALKEEPER_ACTION_DECIDE = 1,
    GOALKEEPER_ACTION_MOVE = 2,
    GOALKEEPER_ACTION_HOLD = 3
} GoalkeeperAction_t;

typedef struct {
    uint8_t seq;
    uint8_t flags;
    int16_t error;
    uint8_t area_x10;
    uint8_t ttc_10ms;
    uint8_t confidence;
    uint32_t last_update_ms;
} GoalkeeperInput_t;

extern volatile GoalkeeperInput_t g_goalkeeper_input;
extern volatile uint8_t g_goalkeeper_debug_state;
extern volatile int16_t g_goalkeeper_debug_turn;
extern volatile int16_t g_goalkeeper_debug_speed_x10;
extern volatile uint16_t g_goalkeeper_debug_encoder_speed;
extern volatile uint16_t g_goalkeeper_debug_age_ms;
extern volatile uint16_t g_goalkeeper_debug_rx_count;
extern volatile uint8_t g_goalkeeper_debug_gate;
extern volatile int8_t g_goalkeeper_debug_lane;

extern float Goalkeeper_Gyro_Kd;
extern float Goalkeeper_Lane_Speed;
extern float Goalkeeper_Lane_Turn_PWM;

void Goalkeeper_UpdateInput(uint8_t seq, uint8_t flags, int16_t error,
                            uint8_t area_x10, uint8_t ttc_10ms,
                            uint8_t confidence);
void Goalkeeper_Update(float gyro_z, int encoder_left, int encoder_right);
float Goalkeeper_SpeedTarget(void);
float Goalkeeper_TurnCalc(float gyro_z);
int Goalkeeper_FilterVelocityPwm(int velocity_pwm);
void Goalkeeper_Reset(void);
void Goalkeeper_ParseByte(uint8_t rx);

#endif
