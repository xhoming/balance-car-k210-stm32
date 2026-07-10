#include "bsp_timer.h"

static float battery_sum = 0.0f;
static uint8_t battery_count = 0;
static uint8_t battery_tick = 0;

u16 led_flag = 0;
u16 led_twinkle_count = 0;
u16 led_count = 0;
u8 lower_power_flag = 0;

static void power_detect(void)
{
    static u8 normal_power_flag = 1;

    if (battery < 9.6f) {
        lower_power_flag = 1;
        normal_power_flag = 0;
    } else if (normal_power_flag == 0) {
        lower_power_flag = 0;
        normal_power_flag = 1;
        BEEP_BEEP = 0;
    }
}

static void control_led(void)
{
    if (!led_flag) {
        if (led_count > 300) {
            led_count = 0;
            led_flag = 1;
        }
        return;
    }

    if (led_count <= 20) {
        return;
    }

    led_count = 0;
    if (lower_power_flag == 0) {
        LED = !LED;
    } else {
        BEEP_BEEP = !BEEP_BEEP;
        LED = 1;
    }

    led_twinkle_count++;
    if (led_twinkle_count != 6) {
        return;
    }

    if (lower_power_flag == 0) {
        LED = 0;
    } else {
        BEEP_BEEP = 0;
    }

    led_twinkle_count = 0;
    led_flag = 0;
}

void TIM6_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) {
        return;
    }

    led_count++;
    battery_tick++;

    if (battery_tick > 2) {
        battery_tick = 0;
        battery_sum += Get_Battery_Volotage();
        battery_count++;
        if (battery_count == 50) {
            battery = battery_sum / 50.0f;
            battery_sum = 0.0f;
            battery_count = 0;
            power_detect();
        }
    }

    control_led();
}
