#include "delay.h"

#define TIM6EN 	1<<4

void Timer_Config(void)
{
	//Uses TIM6
	/*
	STEPS
		1. Enable the timer clock
		2. Set the prescaler and the ARR
		3. Enable the Timer, and wait for the update flag to be set
	*/
	
	//Enable timer 6 (basic timer) in APB1 peripheral clock enable register. Pg 190
	RCC->APB1ENR |= TIM6EN;
	
	//The APB1 Timer clocks are run on a 108 MHz clock. Set prescalar to 108 to get microsecond tick. Note prescalar adds one to given value
	TIM6->PSC = 108-1;
	//Set timer auto reload register (ARR) ie overflow value for the timer to 65535. Pg 1091
	TIM6->ARR = 0xffff;
	
	//Start timer. Pg 1088
	TIM6->CR1 |= TIM_CR1_CEN;
	
	//Wait for the first cycle of the timer. Pg 1090
	while(!(TIM6->SR & (1 << 0)));
}

void delay_us(uint32_t time)
{
	TIM6->CNT = 0;
	while(TIM6->CNT < time);
}

void delay_ms(uint32_t time)
{
	//Set reload value to 1000
	TIM6->ARR = 0x3E8;
	for (uint32_t a = 0; a < time; a++)
	{
		delay_us(1000);
	}
	TIM6->ARR = 0xffff;
}