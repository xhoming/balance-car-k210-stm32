#include "app_bluetooth.h"

u8 newLineReceived = 0;
int int9num = 0;
u8 inputString[80] = {0};

static uint8_t rx_index = 0;

void deal_bluetooth(uint8_t rxbuf)
{
    if (rx_index == 0) {
        if (rxbuf != BT_HEADER) {
            return;
        }
        inputString[rx_index++] = rxbuf;
        return;
    }

    inputString[rx_index++] = rxbuf;

    if (rx_index < 4) {
        return;
    }

    rx_index = 0;
    if (inputString[3] != BT_FOOTER) {
        newLineReceived = 0;
        return;
    }

    int9num = 3;
    newLineReceived = 1;
}
