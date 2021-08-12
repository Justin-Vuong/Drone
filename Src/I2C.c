#include "I2C.h"

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

uint16_t I2C_Read2ByteMpuRegister(uint8_t reg_addr)
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
	
	//Send start signal with read, set numBytes = 2
	I2C1->CR2 = 0;
	I2C1->CR2 |= (0x68 << 1) | (2 << 16) | I2C_CR2_RD_WRN; //| I2C_CR2_AUTOEND ;
	I2C1->CR2 &= ~(I2C_CR2_ADD10);
	
	//Send another start condition
	I2C1->CR2 |= I2C_CR2_START;
	
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	uint16_t recvData = I2C1->RXDR<<8;
		
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	recvData |= I2C1->RXDR;

	//Send stop bit 
	I2C1->CR2 |= I2C_CR2_STOP;
	return recvData;
}

uint8_t I2C_ReadMpuRegister(uint8_t reg_addr)
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
	I2C1->CR2 = 0;
	I2C1->CR2 |= (0x68 << 1) | (1 << 16) | I2C_CR2_RD_WRN; //| I2C_CR2_AUTOEND ;
	I2C1->CR2 &= ~(I2C_CR2_ADD10);
	
	//Send another start condition
	I2C1->CR2 |= I2C_CR2_START;
	
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	uint8_t recvData = I2C1->RXDR;
	
	//Send stop bit 
	I2C1->CR2 |= I2C_CR2_STOP;
	return recvData;
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

//Pass in an array of size 3 in gyroData
void I2C_ReadMpuGyroRegisters(uint16_t* gyroData)
{ 
	//GYRO_XOUT (high and low) are read first as they are registers 67-68
	//GYRO_YOUT (high and low) are read second as they are registers 68-69
	//GYRO_ZOUT (high and low) are read third as they are registers 70-71 
	
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
	I2C1->TXDR = 116;
	
	//Wait for the data to be written in the register
	while(!(I2C1->ISR & I2C_ISR_TC));
	
	//Send start signal with read, set numBytes = 6
	I2C1->CR2 = 0;
	I2C1->CR2 |= (0x68 << 1) | (6 << 16) | I2C_CR2_RD_WRN;
	I2C1->CR2 &= ~(I2C_CR2_ADD10);
	
	//Send another start condition
	I2C1->CR2 |= I2C_CR2_START;
	
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	gyroData[0] = I2C1->RXDR<<8;
		
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	gyroData[0] |= I2C1->RXDR;

	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	gyroData[1] = I2C1->RXDR<<8;
		
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	gyroData[1] |= I2C1->RXDR;

	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	gyroData[2] = I2C1->RXDR<<8;
		
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	gyroData[2] |= I2C1->RXDR;

	//Send stop bit 
	I2C1->CR2 |= I2C_CR2_STOP;
}

//Pass in an array of size 3 in accelData
void I2C_ReadMpuAccelRegisters(uint16_t* accelData)
{
	//ACCEL_XOUT (high and low) are read first as they are registers 59-60
	//ACCEL_YOUT (high and low) are read second as they are registers 61-62
	//ACCEL_ZOUT (high and low) are read third as they are registers 63-64 
	
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
	I2C1->TXDR = 116;
	
	//Wait for the data to be written in the register
	while(!(I2C1->ISR & I2C_ISR_TC));
	
	//Send start signal with read, set numBytes = 6
	I2C1->CR2 = 0;
	I2C1->CR2 |= (0x68 << 1) | (6 << 16) | I2C_CR2_RD_WRN;
	I2C1->CR2 &= ~(I2C_CR2_ADD10);
	
	//Send another start condition
	I2C1->CR2 |= I2C_CR2_START;
	
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	accelData[0] = I2C1->RXDR<<8;
		
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	accelData[0] |= I2C1->RXDR;

	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	accelData[1] = I2C1->RXDR<<8;
		
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	accelData[1] |= I2C1->RXDR;

	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	accelData[2] = I2C1->RXDR<<8;
		
	//Wait for data received
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	accelData[2] |= I2C1->RXDR;

	//Send stop bit 
	I2C1->CR2 |= I2C_CR2_STOP;
}

void I2C_VL53L0X_WriteMulti(uint8_t devAddr, uint8_t registerNum, uint8_t* pData, uint32_t dataSz)
{
	//Set the address to 7 bits and write
	I2C1->CR2 = 0;
	//Set address and size of data
	I2C1->CR2 |= (devAddr << 1) | ((dataSz+1) << 16);
	I2C1->CR2 &= ~(I2C_CR2_ADD10 | I2C_CR2_RD_WRN);
	
	//Send Start Bit
	I2C1->CR2 |= I2C_CR2_START;

	//Wait for ACK from slave
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	//Send sensor data register that you want to read
	I2C1->TXDR = registerNum;

	for (uint32_t index = 0; index < dataSz; index++)
	{
		//Wait for ACK from slave
		while(!(I2C1->ISR & I2C_ISR_TXIS));
		//Send data
		I2C1->TXDR = pData[index];
	}

	while(!(I2C1->ISR & I2C_ISR_TC));

	//Send stop
	I2C1->CR2 |= I2C_CR2_STOP;
}

void I2C_VL53L0X_ReadMulti(uint8_t devAddr, uint8_t registerNum, uint8_t* pData, uint32_t dataSz)
{
	//Send the Address and Write bit

	//Set the address to 7 bits and write
	I2C1->CR2 = 0;
	//Configure to send the 1 byte address/write
	I2C1->CR2 |= (devAddr << 1) | (1 << 16);

	//Send Start Bit
	I2C1->CR2 |= I2C_CR2_START;

	//Wait for ACK from slave
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	//Send sensor data register that you want to read
	I2C1->TXDR = registerNum;
	while(!(I2C1->ISR & I2C_ISR_TC));

	I2C1->CR2 |= I2C_CR2_STOP;

	//Read sequence

	I2C1->CR2 = 0;
	//Set device address, number of bytes to read and read bit
	I2C1->CR2 |= (devAddr << 1) | (dataSz << 16) | I2C_CR2_RD_WRN; 

	//Send Start Bit
	I2C1->CR2 |= I2C_CR2_START;
	
	for (uint32_t index = 0; index < dataSz; index++)
	{
		//Wait for data received
		while(!(I2C1->ISR & I2C_ISR_RXNE));
		pData[index] = I2C1->RXDR;
	}

	//Send stop bit
	I2C1->CR2 |= I2C_CR2_STOP;
}
