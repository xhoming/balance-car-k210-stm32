#ifndef __APP_CONTROL_H_
#define __APP_CONTROL_H_

#include "AllHeader.h"

void Get_Angle(u8 way);
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right);
int Put_Down(float Angle, int encoder_left, int encoder_right);

#endif
