#ifndef __PRINT_H__
#define __PRINT_H__
#include"stm32f4xx_hal.h"
uint8_t uart_getc(UART_HandleTypeDef *huart);

uint8_t uart_putc(UART_HandleTypeDef *huart,uint8_t c);

void uart_puts(UART_HandleTypeDef *huart,char *str);
#endif
