//Using data sheet: https://www.st.com/resource/en/reference_manual/dm00224583-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// and https://www.st.com/resource/en/datasheet/stm32f765zi.pdf
#include <math.h>
#include <stdint.h>
#include "stm32f767xx.h"


#define PLL_M		25
#define PLL_N		432
#define PLL_P		0 //PLL_P is /2

#define TIM6EN 	1<<4

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

void Clock_Config(void)
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
	
	//Set PLL_N = 432, PLL_M = 25, PLL_P = 2 and PLL_SRC to HSE in RCC PLL config register. Pg 166-167
	RCC->PLLCFGR |= (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE;
	
	//Enable PLL and wait for ready bit in RCC clock config register. Pg 163
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	
	//Set PLL_P as clock source and wait for ready bit in the RCC clock config register. Pg 170
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

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
	
	//Set GPIO pin speed to very high speed. pg 230
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
	TIM5->PSC = (unsigned int) (108-1);
	//Set timer auto reload register (ARR) ie overflow value for the timer to 100 to easily set dutycycle. Pg 1091
	TIM5->ARR = (unsigned int) 1000;
	
	//Clear the counter, reinitialize it, and update the Event Generation Register (EGR)
	TIM5->EGR |= TIM_EGR_UG;
	
	//Enable the counter
	TIM5->CR1 |= TIM_CR1_CEN;
	
	//Wait for the first cycle of the timer. Pg 1090
	while(!(TIM5->SR & (1 << 0)));
}

void ADC_Config(void)
{
	//Using ADC 1 (pin PA1)
	/*
	STEPS
		1. Enable ADC and GPIO clock
		2. Set the prescalar in the Common Control Register (CCR)
		3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
		4. Set the Continuous Conversion, End of Conversion (EOC), and Data Alignment in Control Register 2 (CR2)
		5. Set the Sampling Time for the channels in teh ADC_SMPRx
		6. Set the Regualr channel sequence length in ADC_SQR1
		7. Set the Respective GPIO pins in the Analog Mode
	*/
	
	//Enable ADC1 in APB2 peripheral clock enable register. pg 191
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	//Enable GPIO A clock in AHB1 peripheral clock register. pg 184
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	//Set the prescalar to /4 to get APB2 clock to a speed the ADC can use (max of 36MHz)
	ADC->CCR |= ADC_CCR_ADCPRE_0;
	
	//Set resolution to 12 bits and enable scan mode using ADC control register pg. 469
	ADC1->CR1 &= ~(ADC_CR1_RES_0 | ADC_CR1_RES_1);
	ADC1->CR1 |= ADC_CR1_SCAN;
	
	//Set data alignment to right, set end of conversion (EOC) to end of each regular 
	//and enable continuous conversion in ADC Control Register 2. pg 471
	ADC1->CR2 &= ~ADC_CR2_ALIGN;
	ADC1->CR2 |= ADC_CR2_EOCS | ADC_CR2_CONT;
	
	//Set SMP1 channel to refresh every 3 cycles in ADC Sample Time Register 2 (SMPR2) register 
	ADC1->SMPR2 &= ~(ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2);

	//Set 1 channel for use in the ADC regular sequence register 1 (ADC_SQR1). pg 476
	ADC1->SQR1 &= ~(ADC_SQR1_L_0 | ADC_SQR1_L_1 | ADC_SQR1_L_2);
	
	//Set pin PA4 to an analog output pin in GPIOA_MODER register. pg 229
	GPIOA->MODER |= GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1;
	
	//Enable ADC by setting ADON bit in CR2 register. pg 471
	ADC1->CR2 |= ADC_CR2_ADON;
	
	//Wait for ADC to stabilize
	delay_us(10);
}


//Pass in the channel that is going to be read. Only pass in 0-18
void ADC_Start(uint32_t channel)
{
	/*
	STEPS
		1. Set the channel sequence (the order that the channels of the ADC are read)
			-The number of the channel in the SQ1 is read first then SQ2... etc.
		2. Clear the Status register
		3. Start the Conversion by setting the SWSTART bit in CR2
	*/
	
	//Set the channel number in the SQ1 bits so it is read first using the ADC regular sequence register 3. pg 477
	ADC1->SQR3 = channel;
	
	//Clear the ADC status register. pg 468 
	ADC1->SR = 0;
	
	//Start a conversion using SWSTART in the ADC control register 2. pg 471
	ADC1->CR2 |= ADC_CR2_SWSTART;
	
	//Wait for end of conversion bit in the ADC status register. pg 468
	while (!(ADC1->SR & ADC_SR_EOC));
}

uint32_t ADC_Read(void)
{
	//Read the data from the ADC regular data register. pg 479
	return ADC1->DR;
}

void USART3_SendChar(uint8_t data)
{
	/*
	STEPS
		1. Write the data into the USART_TDR register which will clear the TXE bit
		2. Wait for the TC =1 which indicates that the transmission of the last frame has completed
	*/
	
	//Load the data into the USART transmite data register. pg 1306
	USART3->TDR = data;
	
	//Wait for Transmission Complete bit in the Interrupt and Status Register register. pg 1303
	while (!(USART3->ISR & USART_ISR_TC));
}

void UART3_Config(void)
{
	//Using pins PD8 (TX) and PD9 (RX) but will route the signal to the ST-Link so the date can be read from its mini-usb interface
	/*
	STEPS
		1. Enable UART Clock and GPIO clock
		2. Configure the UART PINs for alternate functions
		3. Program the M bit in the USART_CR1 to define the word length	
		4. Enable the USART by writing the UE bit in USART_CR1 register to 1
		5. Select the desired baud rate using the USART_BRR register
		6. Enable the Tx/Rx by setting the TE and RE bits in the USART_CR1 register
	*/
	
	//Enable UART3 and GPIO D in the RCC_APB1ENR register. pg 187
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->DCKCFGR2 &= ~(RCC_DCKCFGR2_USART3SEL_0 | RCC_DCKCFGR2_USART3SEL_1);
	
	//Configure the alternate functions for the GPIO pins PD8 and PD9 in the GPIOx_MODER registers. pg 229
	GPIOD->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	//Set the GPIO port output speed to high speed in GPIO port output speed register. pg 230 
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1;
	//Set the TX line to have ouptut open drain in the Output type register. pg 229
	GPIOD->OTYPER |= GPIO_OTYPER_OT8;
	//Set the alternate functions 7 to route the data to the ST-Link in the GPIO alternate function registers. pg 233
	GPIOD->AFR[1] |= GPIO_AFRH_AFRH0_0 | GPIO_AFRH_AFRH0_1 | GPIO_AFRH_AFRH0_2 | GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_2;
		
	//Clear the control register then set the M0 and M1 bit to 0 to set 1 start bit, 8 data bits, and 1 stop bit. pg 1285
	USART3->CR1 = 0x00000000;
	USART3->CR2 = 0x00000000;
	USART3->CR3 = 0x00000001;
	
	//USART3 is connected to APB1 clock but feeds USART peripheral with 20MHz. I want baud rate of 230400 so need 
	//USARTDIV to be = 20000000/230400 = 87 in the Baud Rate Register. pg 1296 and 1257
	USART3->BRR = 87UL;
	
	//Enable USART in the USART_CR1 register. pg 1285
	USART3->CR1 |= USART_CR1_UE;
	
	//Enable the Tx/Rx by setting Transmitter enable (TE) and Receiver enable (RE) bits in the USART_CR1 register. pg 1285
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
	while (!(USART3->ISR & USART_ISR_TEACK));
}

void I2C_Config(void)
{
	//PB8 for SCL and PB9 for SDA
	/*
	STEPS
		1. Enable I2C clock and GPIO clock
		2. Configure I2C pins with Alternate functions
		3. Program the peripheral input clock to get the right timings
		4. Configure the clock control registers
		5. Configure the rise time register
		6. Program the I2C_CR1 register to enable the peripheral
	*/
	
	//Enable I2C1 and GPIOB in the RCC_APB1ENR register. pg 187
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	//Set the GPIO pins for alternate functions. pg 229
	GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	//Set the GPIO pins to output open drain in the output type register. pg 229
	GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;
	//Set the speed of the pins in the output speed registers. pg 230
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1 | GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_0;
	//Set the pin to pull up in the pull-up/pull-down register. pg 230
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0; 
	//Set the alternate functions to AF4
	GPIOB->AFR[1] |= GPIO_AFRL_AFRL0_2 | GPIO_AFRL_AFRL1_2;
	
	//Configure the I2C master clock in the timing register. pg 1206
	I2C1->TIMINGR = 0x6000030D;
	
	//Enable error, transfer complete, not acknowledged, RX, TX interrupts in I2C control register 1. pg 1225
	I2C1->CR1 |= I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_NACKIE | I2C_CR1_RXIE | I2C_CR1_TXIE;
	//Set the peripheral enable bit in the I2C_CR1 register. pg 1225
	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_ReadMpuRegister(uint8_t reg_addr)
{
	//https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
	//I2C_CR2_RD_WRN = 0 is a write
	//I2C_CR2_RD_WRN = 1 is a read
	/* From datasheet pg 1198
	The number of TXIS events during the transfer corresponds to the value programmed in
	NBYTES[7:0]. If the total number of data bytes to be sent is greater than 255, reload mode
	must be selected by setting the RELOAD bit in the I2C_CR2 register. In this case, when
	NBYTES data have been transferred, the TCR flag is set and the SCL line is stretched low
	until NBYTES[7:0] is written to a non-zero value.
	
	To read the internal MPU-9250 registers, the master sends a start condition, followed by the I2C address and
	a write bit, and then the register address that is going to be read. Upon receiving the ACK signal from the MPU9250, the master transmits a start signal followed by the slave address and read bit. As a result, the MPU9250 sends an ACK signal and the data. The communication ends with a not acknowledge (NACK) signal and
	a stop bit from master. The NACK condition is defined such that the SDA line remains high at the 9th clock
	cycle. 
	*/
	
	//Set the slave address to 0x68 and the number of bytes to be transfered to 1 in the I2C control register 2. pg 1227
	I2C1->CR2 = 0;
	I2C1->CR2 |= (0x68 << 1) | (1 << 16);
	//Set the addressing mode to 7 bits and set the master transfer direction to write in the I2C control register 2. pg 1227
	I2C1->CR2 &= ~(I2C_CR2_ADD10 | I2C_CR2_RD_WRN);
	//Set the START bit in I2C_CR2 register. pg 1227
	I2C1->CR2 |= I2C_CR2_START;
	
	//Wait for register to be ready for data
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	//Send sensor data register that you want to read
	I2C1->TXDR = reg_addr;
	
	//Wait for the data to be written in the register
	while(!(I2C1->ISR & I2C_ISR_TC));
	
	//Send start signal with read, set numBytes = 1
	I2C1->CR2 |= (0x68 << 1) | (1 << 16) | I2C_CR2_RD_WRN; //| I2C_CR2_AUTOEND ;
	I2C1->CR2 &= ~(I2C_CR2_ADD10);
	
	//Send another start condition
	I2C1->CR2 |= I2C_CR2_START;
	
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	USART3_SendChar(I2C1->RXDR);
	
	//Send stop bit 
	I2C1->CR2 |= I2C_CR2_STOP;
}

void I2C_WriteMpuRegister(uint8_t reg_addr, uint8_t reg_value)
{
	//https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
	//I2C_CR2_RD_WRN = 0 is a write
	//I2C_CR2_RD_WRN = 1 is a read
	/* From datasheet
	To write the internal MPU-9250 registers, the master transmits the start condition (S), followed by the I2C
	address and the write bit (0). At the 9th clock cycle (when the clock is high), the MPU-9250 acknowledges the
	transfer. Then the master puts the register address (RA) on the bus. After the MPU-9250 acknowledges the
	reception of the register address, the master puts the register data onto the bus. This is followed by the ACK
	signal, and data transfer may be concluded by the stop condition (P). To write multiple bytes after the last ACK
	signal, the master can continue outputting data rather than transmitting a stop signal.
	*/
	
	//Set the slave address to 0x68 and the number of bytes to be transfered to 2 in the I2C control register 2. pg 1227
	I2C1->CR2 = 0;
	I2C1->CR2 |= (0x68 << 1) | (2 << 16);
	//Set the addressing mode to 7 bits and set the master transfer direction to write in the I2C control register 2. pg 1227
	I2C1->CR2 &= ~(I2C_CR2_ADD10 | I2C_CR2_RD_WRN);
	//Set the START bit in I2C_CR2 register. pg 1227
	I2C1->CR2 |= I2C_CR2_START;
	
	//Wait for register to be ready for data
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	//Send sensor data register that you want to read
	I2C1->TXDR = reg_addr;
	
	//Wait for register to be ready for data
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	//Send sensor data register that you want to read
	I2C1->TXDR = reg_value;
	
	while(!(I2C1->ISR & I2C_ISR_TC));

	//Send stop
	I2C1->CR2 |= I2C_CR2_STOP;
}

void PWM_test(void)
{
	static unsigned int payload = 0;
	if (payload == 10)
		{
			payload = 0;
		}
		else
		{
			payload += 1;
		}
		TIM5->CCR1 = payload;

}

void blinky(void)
{
	//Reset the pin PB0
		GPIOB->BSRR = GPIO_BSRR_BR0;
		delay_ms(500);
		//Set the pin PB0
		GPIOB->BSRR = GPIO_BSRR_BS0;
		delay_ms(500);
}

void ADC_test(void)
{
	ADC_Start(4);
	double adcPercentage = ADC_Read()/pow(2,12);
	TIM5->CCR1 = (uint16_t) (adcPercentage*800); //Can raise to 1000 if more power is needed
}

void USART3_test(void)
{
	if (USART3->ISR & USART_ISR_RXNE)
	{
		uint32_t data = USART3->RDR;
		USART3_SendChar(data);
	}
}

int main(void)
{
	Clock_Config();
	Timer_Config();
	GPIO_Config();
	PWM_Config();
	ADC_Config();
	UART3_Config();
	I2C_Config();
	I2C_ReadMpuRegister(35);
	//Write FIFO Enable register
	I2C_WriteMpuRegister(35,1 << 6);
	while(1)
	{
		//PWM_test();
		//blinky();
		//ADC_test();
		//USART3_test();
		I2C_ReadMpuRegister(115);
	}
}

