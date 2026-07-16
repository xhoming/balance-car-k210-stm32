#include "bsp.h"

void bsp_init(void)
{
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    delay_init();
    init_led_gpio();
    init_beep();

    Motor_start();
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();

    HAL_Delay(300);

    MPU6050_initialize();
    DMP_Init();

    OLED_I2C_Init();
    Battery_init();
}

void bsp_mode_init(void)
{
    if (mode == Bluetooth_Mode) {
        bluetooth_init();
    } else if (mode == ChaseLine_Mode || mode == KickBall_Mode ||
               mode == Goalkeeper_Mode) {
        bluetooth_init();
        USART2_init();
    }

    TIM6_Init();
}
