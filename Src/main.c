//Using data sheet: https://www.st.com/resource/en/reference_manual/dm00224583-stm32f76xxx-and-stm32f77xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// and https://www.st.com/resource/en/datasheet/stm32f765zi.pdf
#include <math.h>
#include <stdint.h>
#include "stm32f767xx.h"

#include "clock.h"
#include "delay.h"
#include "I2C.h"
#include "UART.h"

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

void MPU9250_Config()
{
	//Set up int pin on ST board's PC6
	//Enable GPIO C in RCC Advanced High-performance Bus 1 (AHB1) peripheral clock register. Pg 184
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	
	//Set the pin to output mode (0b00) in GPIO port mode register. pg 229
	GPIOC->MODER &= ~(3 << 12);

	//Write FIFO Enable register
	//Gyro enable
	//I2C_WriteMpuRegister(35,7 << 4);
	
	//Enable the interrupt and add compare to last sample
	I2C_WriteMpuRegister(105, 3 << 6);
	
	//Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin.
	I2C_WriteMpuRegister(56, 1);

	//Enable Accel data reads to fifo
	I2C_WriteMpuRegister(35, 1 << 3);

	//Enable the fifo in the user control register
	I2C_WriteMpuRegister(106,1 << 6);

}

void MPU9250_Gyro_Test()
{
	uint16_t dataCountInFifo = 0;
	uint16_t dataFromFifo[3];
	dataCountInFifo = I2C_Read2ByteMpuRegister(114);
	if (dataCountInFifo > 0)
	{
		I2C_ReadMpuGyroRegisters(dataFromFifo);
		
		//print num bytes in fifo
		USART3_SendChar(dataCountInFifo>>8);
		USART3_SendChar(dataCountInFifo);
		//print x
		USART3_SendChar(dataFromFifo[0]>>8);
		USART3_SendChar(dataFromFifo[0]);
		//print y
		USART3_SendChar(dataFromFifo[1]>>8);
		USART3_SendChar(dataFromFifo[1]);
		//print z
		USART3_SendChar(dataFromFifo[2]>>8);
		USART3_SendChar(dataFromFifo[2]);
		}
}

void MPU9250_Accel_Test()
{
	//Wait for the pin to go high
	if(!(GPIOC->IDR & (1 << 6))){
		//USART3_SendString("Interrupt Pin is low\n");
		return;
	}
	
	uint8_t interruptStatus = I2C_ReadMpuRegister(58);
	//USART3_SendString("Interrupt Status is: ");
	//USART3_SendChar(interruptStatus);
	//USART3_SendChar('\n');
	if (!(interruptStatus & 1))
	{
		//If the RAW_DATA_RDY_INT interrupt was not triggered
		USART3_SendString("Interrupt RAW_DATA_RDY_INT not detected");
		return;
	}
	
	uint16_t dataCountInFifo = 0;
	uint16_t dataFromFifo[3];
	
	//Read Fifo count register
	dataCountInFifo = (I2C_ReadMpuRegister(114) & 0x1F);
	dataCountInFifo = (dataCountInFifo << 8) | I2C_ReadMpuRegister(115);
	
	if (dataCountInFifo > 0 && dataCountInFifo < 512)
	{
		
		I2C_ReadMpuAccelRegisters(dataFromFifo);
		//USART3_SendString("Num Bytes in FIFO is: ");
	  USART3_Send2Char(dataCountInFifo);
		//print x
		//USART3_SendString("X: ");
		USART3_SendChar(dataFromFifo[0]>>8 & 0xFF);
		USART3_SendChar(dataFromFifo[0] & 0xFF);
		//print y
		//USART3_SendString(",Y: ");
		USART3_SendChar(dataFromFifo[1]>>8 & 0xFF);
		USART3_SendChar(dataFromFifo[1] & 0xFF);
		//print z
		//USART3_SendString(",Z: ");
		USART3_SendChar(dataFromFifo[2]>>8 & 0xFF);
		USART3_SendChar(dataFromFifo[2] & 0xFF);
		USART3_SendString("\n");
		
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
	MPU9250_Config();
	while(1)
	{
		//PWM_test();
		//blinky();
		//ADC_test();
		//USART3_test();
		//MPU9250_Gyro_Test();	
		MPU9250_Accel_Test();
		//delay_ms(100);
	}
}
