#include "bsp_bluetooth.h"

extern UART_HandleTypeDef huart5;

uint8_t g_bluetooth_rx_byte;

void bluetooth_init(void)
{
    HAL_UART_Receive_IT(&huart5, &g_bluetooth_rx_byte, 1);
}

void UART5_DataByte(uint8_t data_byte)
{
    HAL_UART_Transmit(&huart5, &data_byte, 1, 0xffff);
}

void UART5_RX_deal(uint8_t rx_data)
{
    deal_bluetooth(rx_data);
}
