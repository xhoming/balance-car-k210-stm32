#include "bsp_usart2.h"

extern UART_HandleTypeDef huart2;

uint8_t g_k210_rx_byte;

void USART2_init(void)
{
    HAL_UART_Receive_IT(&huart2, &g_k210_rx_byte, 1);
}

void USART2_RX_deal(uint8_t rx_data)
{
    Deal_K210_Vision(rx_data);
    BallKick_ParseByte(rx_data);
}
