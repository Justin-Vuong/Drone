#include "stm32f767xx.h"
#include "SPI.h"

void SPI_Transmit(uint8_t* data, int size)
{
	/*
	STEPS
		1. Wait for TXE bit to be set in Status Register
		2. Write the data to the Data Register
		3. After the data has been transmitted, wait for the BSY bit to reset the Status Register
		4. Clear the Overrun flag by reading DR and SR
	*/
	for (int i = 0; i < size; i++)
	{
		//Wait for TXE to be set
		while (!(SPI1->SR & SPI_SR_TXE));

		//Send data
		SPI1->DR = data[i];
	}

	//Wait for BSY bit to be cleared in the Status Register
	while ((SPI1->SR & SPI_SR_BSY));

	//Clear the data that came from shifts that happened during TX sends
	uint8_t tmp = SPI1->DR;
	tmp = SPI1->SR;
	tmp += 1;
}

void SPI_TransmitByte(uint8_t data)
{
	/*
	STEPS
		1. Wait for TXE bit to be set in Status Register
		2. Write the data to the Data Register
		3. After the data has been transmitted, wait for the BSY bit to reset the Status Register
		4. Clear the Overrun flag by reading DR and SR
	*/

    //Wait for TXE to be set
    while (!(SPI1->SR & SPI_SR_TXE));

    //Send data
    SPI1->DR = data;

	//Wait for BSY bit to be cleared in the Status Register
	while ((SPI1->SR & SPI_SR_BSY));

	//Clear the data that came from shifts that happened during TX sends
	uint8_t tmp = SPI1->DR;
	tmp = SPI1->SR;
	tmp += 1;
}

void SPI_Receive(uint8_t* data, int size)
{
	for(int a = 0; a < size; a++)
	{
		//Wait for the SPI Busy bit to be cleared
		while ((SPI1->SR & SPI_SR_BSY));
		SPI1->DR = 0;
		//Wait for data received signaled by RXNE bit in the Status register
		while (!(SPI1->SR & SPI_SR_RXNE));
		data[a] = SPI1->DR;
	}
}

uint8_t SPI_ReceiveByte(void)
{
    //Wait for the SPI Busy bit to be cleared
    while ((SPI1->SR & SPI_SR_BSY));
    SPI1->DR = 0;
    //Wait for data received signaled by RXNE bit in the Status register
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

uint8_t SPI_RW_Byte(uint8_t data)
{
    //Send a byte then read a byte
    SPI1->DR = data; //Load data into the buffer

    //Wait for TXE to be set
    while (!(SPI1->SR & SPI_SR_TXE));

    return SPI1->DR; //Read out data
}