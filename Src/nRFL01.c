#include <string.h>
#include "stm32f767xx.h"
#include "nRFL01.h"
#include "SPI.h"
#include "delay.h"

uint8_t tx_addr[] = {0xE8, 0xE8, 0xE8, 0xE8, 0xE8};
uint8_t rx_addr[] = {0xC1, 0xC1, 0xC1, 0xC1, 0xC1};

void nRFL01_Config(void)
{
	/*
	PB5 used for  MOSI
	PB3 used for SCK
	PF12 used for CS
	PB4 used for MISO
	PC7 used for CE
	
	STEPS
	1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
	2. Write to the SPI_CR1 register:
		a) Configure the serial clock baud rate using the BR[2:0] bits (Note: 4).
		b) Configure the CPOL and CPHA bits combination to define one of the four relationships between the data transfer and the serial clock (CPHA must be cleared in NSSP mode). (Note: 2 - except the case when CRC is enabled at TI mode).
		c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and BIDIOE (RXONLY and BIDIMODE can't be set at the same time).
		d) Configure the LSBFIRST bit to define the frame format (Note: 2).
		e) Configure the CRCL and CRCEN bits if CRC is needed (while SCK clock signal is at idle state).
		f) Configure SSM and SSI (Notes: 2 & 3).
		g) Configure the MSTR bit (in multimaster NSS configuration, avoid conflict state on NSS if master is configured to prevent MODF error).
	3. Write to SPI_CR2 register:
		a) Configure the DS[3:0] bits to select the data length for the transfer.
		b) Configure SSOE (Notes: 1 & 2 & 3).
		c) Set the FRF bit if the TI protocol is required (keep NSSP bit cleared in TI mode).
		d) Set the NSSP bit if the NSS pulse mode between two data units is required (keep CHPA and TI bits cleared in NSSP mode).
		e) Configure the FRXTH bit. The RXFIFO threshold must be aligned to the read access size for the SPIx_DR register.
		f) Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.
	4. Write to SPI_CRCPR register: Configure the CRC polynomial if needed.
	5. Write proper DMA registers: Configure DMA streams dedicated for SPI Tx and Rx in DMA registers if the DMA streams are used.
	*/

	//Open GPIO ABC interface
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOFEN;
	
	//Set CS and CE pins to output
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	GPIOF->MODER |= GPIO_MODER_MODER12_0;

	//Set the SPI pins' mode to Alternate function
	GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	
	//Set the GPIO pins' speed to High Speed
	GPIOB->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEEDR3_Pos) | (3UL << GPIO_OSPEEDR_OSPEEDR4_Pos) | (3UL << GPIO_OSPEEDR_OSPEEDR5_Pos);
	GPIOC->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEEDR7_Pos);
	GPIOF->OSPEEDR |= (3UL << GPIO_OSPEEDR_OSPEEDR12_Pos);

	//Set Alternate Functions to pins to AF5
	GPIOB->AFR[0] |= (5UL << GPIO_AFRL_AFRL3_Pos) | (5UL << GPIO_AFRL_AFRL4_Pos) | (5UL << GPIO_AFRL_AFRL5_Pos);

	//Enable SPI1 clock in RCC APB2 peripheral clock enable register (RCC_APB2ENR) pg. 191
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI1->CR1 = 0;
	//Set software slave management and internal slave select for custom CS control, prescalar to 16 (0b01) in the SPI control register 1 (SPIx_CR1). pg 1356
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_MSTR;

	SPI1->CR2 = 0;
	//Set data size to 8 bits in SPI control register 2 (SPIx_CR2). pg 1358
	SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_SSOE;

	//Enable SPI peripheral
	SPI1->CR1 |= SPI_CR1_SPE;
}

void CS_Enable(void)
{
    GPIOF->BSRR |= (GPIO_BSRR_BR12);
}

void CS_Disable (void)
{
	GPIOF->BSRR |= (GPIO_BSRR_BS12);
}

void CE_Enable(void)
{
	GPIOC->BSRR |= (GPIO_BSRR_BS7);	
}

void CE_Disable (void)
{
	GPIOC->BSRR |= (GPIO_BSRR_BR7);
}

void nRFL01_Write_Reg(uint8_t addr, uint8_t value)
{
    //Writes one byte to addr
    CS_Enable();
    SPI_TransmitByte(W_REGISTER(addr));
    SPI_TransmitByte(value);
    CS_Disable();
}

uint8_t nRFL01_Read_Reg(uint8_t addr)
{
    uint8_t data = 0;

    //Writes one byte to addr
    CS_Enable();
    SPI_TransmitByte(R_REGISTER(addr));
    data = SPI_ReceiveByte();
    CS_Disable();
    return data;
}

void nRFL01_Write_Regs(uint8_t addr, uint8_t* values, int size)
{
    //Writes multiple byte to addr
    CS_Enable();
    SPI_TransmitByte(W_REGISTER(addr));
    SPI_Transmit(values, size);
    CS_Disable();

}

void nRFL01_TX_Init(void)
{
    delay_ms(100);

    //Configure config register
    nRFL01_Write_Reg(0x00, 0x0E);

    delay_ms(2);

    //Enable Auto Acknowledge on all pipes
    nRFL01_Write_Reg(0x01, 0x3F);

    //Enable RX_addresses 0 and 1
    nRFL01_Write_Reg(0x02, 0x01);

    //Set RX address length to 5 bytes
    nRFL01_Write_Reg(0x03, 0x03);

    //Set auto retransmit delay to 4000 us
    nRFL01_Write_Reg(0x04, 0xFF);

    //Set frequency channel
    //Might need to change based on other RF devices nearby
    nRFL01_Write_Reg(0x05, 0xF0);

    //Set data rate to 2Mbps
    nRFL01_Write_Reg(0x06, 0x0E);

    //Reset IRQ Flags in Status register
    nRFL01_Write_Reg(0x07, 0x7E);

    //Set TX_ADDR
    nRFL01_Write_Regs(0x10, tx_addr, 5);

    //Set RX_ADDR_P0  (Acknowledgement packet pipe)
    nRFL01_Write_Regs(0x0A, tx_addr, 5);

    //Set RX_ADDR_P1  (Data pipe)
    nRFL01_Write_Regs(0x0B, rx_addr, 5);

    //Set RX_PW_P0 to receive payloads of length 6
    nRFL01_Write_Reg(0x11, 0x06);

    //Set RX_PW_P1 to receive payloads of length 6
    nRFL01_Write_Reg(0x12, 0x06);

    //Reset TX Max Retry Flag in status register
    nRFL01_clear_max_retries();

    //Send FLUSH_TX to clear TX FIFO
    CS_Enable();
    SPI_TransmitByte(FLUSH_TX);
    CS_Disable();

    //Send FLUSH_RX to clear RX FIFO
    CS_Enable();
    SPI_TransmitByte(FLUSH_RX);
    CS_Disable();
    
    delay_ms(100);
}

//Loads up to 32 bytes in TX FIFO
void nRFL01_Write_Tx_Payload(uint8_t* values, int size)
{
    //Send FLUSH_TX to clear TX FIFO
    CS_Enable();
    SPI_TransmitByte(FLUSH_TX);
    CS_Disable();

    //Writes multiple byte to addr
    CS_Enable();
    SPI_TransmitByte(W_TX_PAYLOAD);
    SPI_Transmit(values, size);
    CS_Disable();

    delay_ms(10);
    //Set chip enable pin to enter standby mode 2 (send data)
    CE_Enable(); 
    delay_ms(20);
    CE_Disable(); 
    delay_ms(10);

}

void nRFL01_RX_Init(void)
{
    delay_ms(100);

    //Configure config register
    nRFL01_Write_Reg(0x00, 0x0F);

    delay_ms(2);

    //Enable auto acknowledgement packets for all pipes
    nRFL01_Write_Reg(0x01, 0x3F);

    //Enable RX pipe 1
    nRFL01_Write_Reg(0x02, 0x02);

    //Set RX address length to 5 bytes
    nRFL01_Write_Reg(0x03, 0x03);

    //Set auto retransmit delay to 4000 us alnd Auto Retransmit Count to 15
    nRFL01_Write_Reg(0x04, 0xFF);

    //Set frequency channel
    //Might need to change based on other RF devices nearby
    nRFL01_Write_Reg(0x05, 0xF0);

    //Set data rate to 2Mbps
    nRFL01_Write_Reg(0x06, 0x0E);

    //Reset IRQ Flags in Status register
    nRFL01_Write_Reg(0x07, 0x7E);

    //Set TX_ADDR for the RX device
    nRFL01_Write_Regs(0x10, rx_addr, 5);

    //Set RX_ADDR_P0 (acknowledgement packet pipe)
    nRFL01_Write_Regs(0x0A, rx_addr, 5);

    //Set RX_ADDR_P1 (data pipe)
    nRFL01_Write_Regs(0x0B, tx_addr, 5);

    //Set data length on pipe 0 to 6
    nRFL01_Write_Reg(0x11, 0x06);    

    //Set data length on pipe 1 to 6
    nRFL01_Write_Reg(0x12, 0x06);    

    //Reset TX Max Retry Flag in status register
    nRFL01_clear_max_retries();

    //Send FLUSH_TX to clear TX FIFO
    CS_Enable();
    SPI_TransmitByte(FLUSH_TX);
    CS_Disable();

    //Send FLUSH_RX to clear RX FIFO
    CS_Enable();
    SPI_TransmitByte(FLUSH_RX);
    CS_Disable();

    //Set enable pin to enter standby mode 2
    CE_Enable();

    delay_ms(100);
}

void nRFL01_RX_Read_Payload(uint8_t* data, uint8_t dataSz)
{   
    //Read RX payload through SPI interface
    strcpy((char*) data, "FILLER");
    CS_Enable();
    SPI_TransmitByte(R_RX_PAYLOAD);
    SPI_Receive(data, dataSz);
    CS_Disable();

    //Store the status register in status_reg
    uint8_t status_reg = 0;
    CS_Enable();
    status_reg = SPI_RW_Byte(NOP);
    CS_Disable();

    //Write a 1 to RX_DR to clear IRQ
    CS_Enable();
    SPI_TransmitByte(W_REGISTER(0x07));
    SPI_TransmitByte(status_reg | (1 << 6));
    CS_Disable();

    delay_ms(10);
}

void nRFL01_heartbeat(uint8_t* data)
{
    /*
    xxx represents place holders depending where data can be read in RX
        -> 000-101 are data pipe numbers with data
        -> 110 means not used
        -> 111 means RX FIFO Empty
    For TX expect: data[0] = 0x0E and data[1] = 0b0000xxx0
    For RX expect: data[0] = 0x0F and data[1] = 0b0000xxx0
    */

    //Read status register
    CS_Enable();
    data[0] = SPI_RW_Byte(NOP);
    CS_Disable();
    
    //Read config register
    data[1] = nRFL01_Read_Reg(0x00);
}

uint8_t nRFL01_check_registers(uint8_t* data, bool isRX)
{
    //PASS IN AN ARRAY OF SIZE 22 AS DATA
    //Checks registers against expected values for RX/TX (isRX ? RX:TX)
    //Return 1 else negative of index with incorrect register value

    /*
    For RX you should expect indexes to have:
        Error Code  | Register  | Value
        1           | 0x00      | 0x0F
        2           | 0x01      | 0x3F
        3           | 0x02      | 0x02
        4           | 0x03      | 0x03
        5           | 0x06      | 0x0E
        6           | 0x10      | 0xC1C1C1C1C1
        7           | 0x11      | 0x06
        8           | 0x12      | 0x06
        9           | 0x0A      | 0xC1C1C1C1C1 
        10          | 0x0B      | 0xE8E8E8E8E8
    For TX you should expect data to have:
        Error Code  | Register  | Value
        1           | 0x00      | 0x0E
        2           | 0x01      | 0x3F
        3           | 0x02      | 0x03 
        4           | 0x03      | 0x03
        5           | 0x06      | 0x0E
        6           | 0x10      | 0xE8E8E8E8E8
        7           | 0x11      | 0x06
        8           | 0x12      | 0x06  
        9           | 0x0A      | 0xE8E8E8E8E8
        //10          | 0x0B      | 0xC1C1C1C1C1
    
    */

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x00));
    data[0] = SPI_ReceiveByte();
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x01));
    data[1] = SPI_ReceiveByte();
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x02));
    data[2] = SPI_ReceiveByte();
    CS_Disable();
    
    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x03));
    data[3] = SPI_ReceiveByte();
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x06));
    data[4] = SPI_ReceiveByte();
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x10));
    SPI_Receive(&data[5], 5);
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x11));
    data[10] = SPI_ReceiveByte();
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x12));
    data[11] = SPI_ReceiveByte();
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x0A));
    SPI_Receive(&data[12], 5);
    CS_Disable();

    CS_Enable();
    SPI_TransmitByte(R_REGISTER(0x0B));
    SPI_Receive(&data[17], 5);
    CS_Disable();

    if(isRX)
    {
        if (data[0] != 0x0F)
        {
            return 1;
        }
        else if (data[1] != 0x3F)
        {
            return 2;
        }
        else if (data[2] != 0x02)
        {
            return 3;
        }
        else if (data[3] != 0x03)
        {
            return 4;
        }
        else if (data[4] != 0x0E)
        {
            return 5;
        }
        else if (data[5] != 0xC1)
        {
            return 6;
        }
        else if (data[6] != 0xC1)
        {
            return 6;
        }
        else if (data[7] != 0xC1)
        {
            return 6;
        }
        else if (data[8] != 0xC1)
        {
            return 6;
        }
        else if (data[9] != 0xC1)
        {
            return 6;
        }
        
        else if (data[10] != 0x06)
        {
            return 7;
        }
        else if (data[11] != 0x06)
        {
            return 8;
        }
        
        else if (data[12] != 0xC1)
        {
            return 9;
        }
        else if (data[13] != 0xC1)
        {
            return 9;
        }
        else if (data[14] != 0xC1)
        {
            return 9;
        }
        else if (data[15] != 0xC1)
        {
            return 9;
        }
        else if (data[16] != 0xC1)
        {
            return 9;
        }
        else if (data[17] != 0xE8)
        {
            return 10;
        }
        else if (data[18] != 0xE8)
        {
            return 10;
        }
        else if (data[19] != 0xE8)
        {
            return 10;
        }
        else if (data[20] != 0xE8)
        {
            return 10;
        }
        else if (data[21] != 0xE8)
        {
            return 10;
        }
    }
    else
    {
        if (data[0] != 0x0E)
        {
            return 1;
        }
        else if (data[1] != 0x3F)
        {
            return 2;
        }
        else if (data[2] != 0x01)
        {
            return 3;
        }
        else if (data[3] != 0x03)
        {
            return 4;
        }
        else if (data[4] != 0x0E)
        {
            return 5;
        }
        else if (data[5] != 0xE8)
        {
            return 6;
        }
        else if (data[6] != 0xE8)
        {
            return 6;
        }
        else if (data[7] != 0xE8)
        {
            return 6;
        }
        else if (data[8] != 0xE8)
        {
            return 6;
        }
        else if (data[9] != 0xE8)
        {
            return 6;
        }
        else if (data[10] != 0x06)
        {
            return 7;
        }
        else if (data[11] != 0x06)
        {
            return 8;
        }
        else if (data[12] != 0xE8)
        {
            return 9;
        }
        else if (data[13] != 0xE8)
        {
            return 9;
        }
        else if (data[14] != 0xE8)
        {
            return 9;
        }
        else if (data[15] != 0xE8)
        {
            return 9;
        }
        else if (data[16] != 0xE8)
        {
            return 9;
        }
        else if (data[17] != 0xC1)
        {
            return 10;
        }
        else if (data[18] != 0xC1)
        {
            return 10;
        }
        else if (data[19] != 0xC1)
        {
            return 10;
        }
        else if (data[20] != 0xC1)
        {
            return 10;
        }
        else if (data[21] != 0xC1)
        {
            return 10;
        }
    }
    
    return 0;
}

void nRFL01_clear_max_retries(void)
{
    uint8_t status_reg = 0;
    CS_Enable();
    status_reg = SPI_RW_Byte(NOP);
    CS_Disable();

    //Write 1 to MAX_RT in Status register
    CS_Enable();
    SPI_TransmitByte(W_REGISTER(0x07));
    SPI_TransmitByte(status_reg | (1 << 4));
    CS_Disable();
}