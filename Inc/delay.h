#include <stdint.h>
#include "stm32f767xx.h"

#ifndef DELAY_H
#define DELAY_H

void Timer_Config(void);
void delay_us(uint32_t time);
void delay_ms(uint32_t time);

#endif //DELAY_H