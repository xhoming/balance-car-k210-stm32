#ifndef __ALLHEADER_H
#define __ALLHEADER_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

#include "myenum.h"
#include "app_mode.h"

#include "bsp.h"
#include "delay.h"
#include "bsp_battery.h"
#include "bsp_beep.h"
#include "bsp_LED.h"
#include "bsp_timer.h"
#include "bsp_key.h"
#include "app.h"

#include "bsp_usart.h"

#include "bsp_bluetooth.h"
#include "app_bluetooth.h"

#include "bsp_usart2.h"
#include "app_user.h"
#include "app_vision_turn.h"
#include "app_ball_kick.h"

#include "bsp_oled.h"
#include "bsp_oled_i2c.h"
#include "oled_show.h"

#include "IOI2C.h"
#include "MPU6050.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "motor.h"
#include "encoder.h"
#include "app_motor.h"


#include "app_control.h"
#include "pid_control.h"
#include "filter.h"
#include "KF.h"

extern float Velocity_Left, Velocity_Right;
extern uint8_t GET_Angle_Way;
extern float Angle_Balance, Gyro_Balance, Gyro_Turn;
extern int Motor_Left, Motor_Right;
extern int Temperature;
extern float Acceleration_Z;
extern int Mid_Angle;
extern float Move_X, Move_Z;
extern float battery;
extern u8 lower_power_flag;
extern u8 Flag_velocity;
extern u8 Stop_Flag;
extern float Car_Target_Velocity;
extern Car_Mode mode;

#endif
