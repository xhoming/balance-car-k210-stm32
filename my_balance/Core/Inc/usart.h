#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;

void MX_UART5_Init(void);
void MX_USART2_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif
