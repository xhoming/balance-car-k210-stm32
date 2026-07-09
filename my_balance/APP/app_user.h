#ifndef __APP_USER_H_
#define __APP_USER_H_

#include "AllHeader.h"

#define BT_HEADER    0xA5
#define BT_FOOTER    0x5A
#define BT_LEFT      0x01
#define BT_RIGHT     0x02
#define BT_SPEED_UP  0x04
#define BT_SPEED_DN  0x08
#define BT_FORWARD   0x10
#define BT_BACKWARD  0x20

#define CAR_MAX_SPEED    40
#define CAR_ACCEL_STEP   1
#define CAR_DECEL_STEP   8
#define CAR_TURN_MAX     127
#define CAR_TURN_STEP    2

void Mode_select_v2(void);
void ProcessCarProtocol(void);
void Deal_K210_Vision(uint8_t rx);
void CarTelemSend(void);

void Car_Diff_Turn_Reset(void);
void Car_Diff_UpdateHeading(float gyro_turn);
void Car_Diff_Turn(float gyro_turn);
uint8_t Car_Diff_IsLocked(void);
float Car_Diff_Heading(void);

#endif
