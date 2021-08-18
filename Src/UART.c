#include "UART.h"

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

void USART3_Send2Char(uint16_t data)
{
	USART3_SendChar(data >> 8);
	USART3_SendChar(data & 0xFF);
}

void USART3_SendString(char* str)
{
	while (*str != '\0')
	{
		USART3_SendChar(*str);
		str += 1;
	}
}