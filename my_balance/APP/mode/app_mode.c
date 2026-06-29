#include "app_mode.h"

uint8_t angle_max = 40;
Car_Mode mode = ChaseLine_Mode;

void Mode_select(void)
{
    int16_t mode_cnt = 0;
    mode = ChaseLine_Mode;
    OLED_Draw_Line("2.ChaseLine Mode", 1, true, true);

    while (!Key1_State(1)) {
        mode_cnt += Read_Encoder(MOTOR_ID_ML);
        mode_cnt += -Read_Encoder(MOTOR_ID_MR);
        car_mode(mode_cnt);
        show_mode_oled();
    }

    Set_Mid_Angle();
    Set_angle();
    Set_control_speed();
    Set_PID();
}

void car_mode(int16_t cnt)
{
    static int16_t cnt_old;

    if (myabs(myabs(cnt) - myabs(cnt_old)) > 250) {
        mode = (mode == ChaseLine_Mode) ? Bluetooth_Mode : ChaseLine_Mode;
        cnt_old = cnt;
    }
}

void Set_Mid_Angle(void)
{
    Mid_Angle = 0;
}

void Set_control_speed(void)
{
    if (mode == Bluetooth_Mode || mode == ChaseLine_Mode) {
        Car_Target_Velocity = 20;
        Car_Turn_Amplitude_speed = 24;
    }
}

void Set_angle(void)
{
    angle_max = 40;
}

extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;

void Set_PID(void)
{
    if (mode == Bluetooth_Mode || mode == ChaseLine_Mode) {
        Balance_Kp = 9700;
        Balance_Kd = 9;
        Velocity_Kp = 5500;
        Velocity_Ki = 24;
        Turn_Kp = 1700;
        Turn_Kd = 20;
    }
}
