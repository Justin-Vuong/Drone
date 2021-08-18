#include "clock.h"

void Clock_Config(void)
{
	/*
	STEPS0
		1. Enable high speed external clock (HSE) and wait for it to be ready
		2. Set the "power enable clock" and "voltage regulator"
		3. Configure the flash related settings
		4. Configure prescalars for the clocks: main CLU clock (HCLK) and peripheral clocks (PCLK1 and PCLK2)
		5. Configure the main phase-locked loop (PLL)
		6. Enable PLL and wait for ready bit
		7. Select the Clock Source and wait for it to be set
	*/
	
	//Using 25MHz external oscillator found in ST-Link.
	//Enable HSE and wait for ready bit in Reset and Control Clock register (RCC). Pg 163
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));
	 
	//Set power interface clock enable in Advanced Peripheral Bus register (APB). Pg 188
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//Set to VOS to 3 (reset value) Pg 142 
	PWR->CR1 |= PWR_CR1_VOS; 
	
	//Enable Prefetch buffer and set flash latency set to 5 clock cycles in flash access control register. Pg 108
	FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	
	//Set Advanced High-performance Bus (AHB) Prescalar = /1, APB1 Prescaler = /4, APB2 Prescaler = /2 in RCC register. Pg 169
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_HPRE_DIV1;
	
	//Set PLL_N = 432, PLL_M = 25, PLL_P = 2 and PLL_SRC to HSE in RCC PLL config register. Pg 166-167
	RCC->PLLCFGR |= (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE;
	
	//Enable PLL and wait for ready bit in RCC clock config register. Pg 163
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	
	//Set PLL_P as clock source and wait for ready bit in the RCC clock config register. Pg 170
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}