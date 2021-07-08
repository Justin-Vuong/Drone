//Using data sheet: https://www.st.com/resource/en/reference_manual/dm00224583-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// and https://www.st.com/resource/en/datasheet/stm32f765zi.pdf
#include "stm32f767xx.h"

#define PLL_M		25
#define PLL_N		432
#define PLL_P		0 //PLL_P is /2

#define TIM6EN 	1<<4
void sysClockConfig(void)
{
	/*
	STEPS
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
	
	//Set PLL_N = 216, PLL_M = 4, PLL_P = 2 and PLL_SRC to HSE in RCC PLL config register. Pg 166-167
	RCC->PLLCFGR |= (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE;
	
	//Enable PLL and wait for ready bit in RCC clock config register. Pg 163
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	
	//Set PLL_P as clock source and wait for ready bit in the RCC clock config register. Pg 170
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void TimerConfig(void)
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

void GPIO_Config(void)
{
	/*
	STEPS
		1. Enable GPIO clock
		2. Set the pin as output
		3. Configure the output mode
	*/
	
	//Note PB0(bits 1:0) has the LED.
	//Enable GPIO B in RCC Advanced High-performance Bus 1 (AHB1) peripheral clock register. Pg 184
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	//Set the pin to output mode (0b01) in GPIO port mode register. pg 229
	GPIOB->MODER |= (1UL << 0);
	
	//Set the output mode to push-pull. Pg 230
	GPIOB->OTYPER &= ~(1UL << 0);
	//Set GPIO pin speed to fast (0b10)
	GPIOB->OSPEEDR |= (1UL << 1);
	//No pull up, pull down
	GPIOB->PUPDR &= ~((1UL << 0) | (1UL << 1));
}

void PWM_Config(void)
{
	
	//USING PA0 Pin
	/*
	STEPS
		1. Select the counter clock (internal, external, prescalar)
		2. Set the frequency in TIMx_ARR register (Overflow value)
		3. Set the duty cycle in TIMxCRRx register
		4. Set the PWM settings by using OCxM, OCxPE, CCxP, CCxE bits in the TIMxCCMRx register
		5. Set the auto-reload preload register (for upcounting and center aligned modes) by setting ARPE bit in TIMx_CR1 register
		6. Initialize the registers using UG bits in TIMx_EGR register
		7. Enable the counter by setting the CEN bit in the TIMx_CR1 register.
	*/
	//Enable GPIO A in RCC Advanced High-performance Bus 1 (AHB1) peripheral clock register. Pg 184
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	//Enable TIM5 as the alternate function in APB1 peripheral clock enable register. pg 190
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	
	//Set the pin to alternate function mode (0b10) in GPIO port mode register. pg 229
	GPIOA->MODER |= (1UL << 1);
	
	//Set the alternate function mode to AF2. pg 233
	//Look up table at https://www.st.com/resource/en/datasheet/stm32f765zi.pdf. pg 89
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL0_1;
	
	//Set GPIO pin speed to very high speed (0b11). pg 230
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR0_1 | GPIO_OSPEEDR_OSPEEDR1_1;
	
	//Enable output for Capture/Compare Register 1 on TIM5. pg 1022
	TIM5->CCER |= TIM_CCER_CC1E;
	
	//Enable Auto-reload Proload Enable (ARPE) in TIM5 Control Register 1
	TIM5->CR1 |= TIM_CR1_ARPE;
	
	//Enable Preload and set output mode to PWM mode 1
	TIM5->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	
	//Trigger an interrupt on Update Interrupt Flag (UIF) when the timer overflows in DMA/interrupt enable register (DIER)
	//TIM5->DIER |= TIM_DIER_UIE;
	
	//The APB1 Timer clocks are run on a 108 MHz clock. Set prescalar to 108 to get millionth of a second tick. Note prescalar adds one to given value
	TIM5->PSC = (unsigned int) (10800-1);
	//Set timer auto reload register (ARR) ie overflow value for the timer to 100 to easily set dutycycle. Pg 1091
	TIM5->ARR = (unsigned int) 10;
	
	//Clear the counter, reinitialize it, and update the Event Generation Register (EGR)
	TIM5->EGR |= TIM_EGR_UG;
	
	//Enable the counter
	TIM5->CR1 |= TIM_CR1_CEN;
	
	//Wait for the first cycle of the timer. Pg 1090
	while(!(TIM5->SR & (1 << 0)));
}

void delay_us(uint32_t time)
{
	TIM6->CNT = 0;
	while(TIM6->CNT < time);
}

void delay_ms(uint32_t time)
{
	for (uint32_t a = 0; a < time; a++)
	{
		delay_us(1000);
	}
}

int main(void)
{
	sysClockConfig();
	TimerConfig();
	GPIO_Config();
	PWM_Config();
	unsigned int payload = 0;
	while(1)
	{
		//Reset the pin PB0
		//GPIOB->BSRR = GPIO_BSRR_BR0;
		//delay_ms(500);
		//Set the pin PB0
		//GPIOB->BSRR = GPIO_BSRR_BS0;
		//delay_ms(500);
		if (payload == 10)
		{
			payload = 0;
		}
		else
		{
			payload += 1;
		}
		TIM5->CCR1 = payload;
		delay_ms(100);
	}
	//Find interrupt TIM5->SR & TIM_SR_UIF
	//Set duty cycle with 	TIM5->CCR1
}
