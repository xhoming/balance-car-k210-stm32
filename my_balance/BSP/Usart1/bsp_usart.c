#include "bsp_usart.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;

#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};

FILE __stdout;

void _sys_exit(int x)
{
    (void)x;
}

int fputc(int ch, FILE *f)
{
    (void)f;
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        USART2_RX_deal(g_k210_rx_byte);
        HAL_UART_Receive_IT(&huart2, &g_k210_rx_byte, 1);
    } else if (huart->Instance == UART5) {
        UART5_RX_deal(g_bluetooth_rx_byte);
        HAL_UART_Receive_IT(&huart5, &g_bluetooth_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2 &&
        huart->RxState == HAL_UART_STATE_READY) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart2, &g_k210_rx_byte, 1);
    }
}
