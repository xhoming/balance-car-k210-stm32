#include <bsp_usart.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
uint8_t RxTemp = 0;

/* USER CODE BEGIN 0 */
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  //Add the following code to support the printf function without selecting use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数   Support functions required by the standard library          
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式   Define _sys_exit() to avoid using semihosting mode 
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 Redefine fputc function
int fputc(int ch, FILE *f)
{      
		while((USART1->SR&0X40)==0);//Flag_Show!=0  使用串口1   Use serial port 1
		USART1->DR = (u8) ch;      
		return ch;
}

#endif




/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
//void USART1_UART_Init(void)
//{
//	//配置不配中断,不需要此串口中断 Configuration does not match interruption, this serial port interruption is not required
////  // Start receiving interrupt 启动接收中断
////  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
//}

void USART1_UART_Init(void)
{
  MX_USART1_UART_Init();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    USART2_RX_deal(g_k210_rx_byte);
    HAL_UART_Receive_IT(&huart2, &g_k210_rx_byte, 1);
  } else if (huart->Instance == UART5) {
    UART5_RX_deal(g_bluetooth_rx_byte);
    HAL_UART_Receive_IT(&huart5, &g_bluetooth_rx_byte, 1);
  } else if (huart->Instance == USART1) {
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
    HAL_UART_Transmit(&huart1, (uint8_t *)&RxTemp, 1, 0xFFFF);
  }
}

void USART1_DataByte(uint8_t data_byte)
{
  HAL_UART_Transmit(&huart1, &data_byte, 1, 0xffff);
}

void USART1_DataString(uint8_t *data_str, uint16_t datasize)
{
  HAL_UART_Transmit(&huart1, data_str, datasize, 0xffff);
}
