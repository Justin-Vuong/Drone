#include "stm32f767xx.h"

#ifndef CLOCK_H
#define CLOCK_H

#define PLL_M		25
#define PLL_N		432
#define PLL_P		0 //PLL_P is /2

void Clock_Config(void);

#endif //CLOCK_H