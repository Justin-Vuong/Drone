#include <stdint.h>
#include "stm32f767xx.h"

#ifndef UART_H
#define UART_H

void UART3_Config(void);
void USART3_SendChar(uint8_t data);
void USART3_Send2Char(uint16_t data);
void USART3_SendString(char* str);

#endif //UART_H