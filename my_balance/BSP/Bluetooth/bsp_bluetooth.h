#ifndef __BSP_BLUETOOTH_H_
#define __BSP_BLUETOOTH_H_

#include "AllHeader.h"

void bluetooth_init(void);
void UART5_RX_deal(uint8_t rx_data);
void UART5_DataByte(uint8_t data_byte);

extern uint8_t g_bluetooth_rx_byte;

#endif
