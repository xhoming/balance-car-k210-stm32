#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = Goalkeeper_Mode;

void Set_Mid_Angle(void)
{
    Mid_Angle = 0;
}

void Set_control_speed(void)
{
    if (mode == Bluetooth_Mode || mode == ChaseLine_Mode ||
        mode == KickBall_Mode || mode == Goalkeeper_Mode) {
        Move_X = 0.0f;
        Move_Z = 0.0f;
        Car_Target_Velocity = 0.0f;
        Velocity_PI_Reset();
    }
}

void Set_angle(void)
{
    angle_max = 40;
}

extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki;

void Set_PID(void)
{
    if (mode == Bluetooth_Mode || mode == ChaseLine_Mode ||
        mode == KickBall_Mode || mode == Goalkeeper_Mode) {
        Balance_Kp = 9700;
        Balance_Kd = 9;
        Velocity_Kp = 7000;
        Velocity_Ki = 34;
    }
}
