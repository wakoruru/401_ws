#include"print.h"

uint8_t uart_getc(UART_HandleTypeDef *huart){
	char buf[1];
	HAL_UART_Receive(huart,(uint8_t *)buf,sizeof(buf),0xffff);
	return (uint8_t)buf[0];
}

uint8_t uart_putc(UART_HandleTypeDef *huart,uint8_t c){
	char buf[1];
	buf[0] = c;
	HAL_UART_Transmit(huart,(uint8_t *)buf,sizeof(buf),0xffff);
}

void uart_puts(UART_HandleTypeDef *huart,char *str){
	while(*str){
		uart_putc(huart,*str++);
	}
}
