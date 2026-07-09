#include "oled_show.h"

void show_mode_oled(void)
{
    static uint8_t mode_old = 0xff;

    if (mode == mode_old) {
        return;
    }

    mode_old = (uint8_t)mode;

    switch (mode_old) {
    case Bluetooth_Mode:
        OLED_Draw_Line("1.Bluetooth Mode", 1, true, true);
        break;
    case ChaseLine_Mode:
        OLED_Draw_Line("2.ChaseLine Mode", 1, true, true);
        break;
    case KickBall_Mode:
        OLED_Draw_Line("3.KickBall Mode", 1, true, true);
        break;
    default:
        OLED_Draw_Line("Unknown Mode", 1, true, true);
        break;
    }
}
