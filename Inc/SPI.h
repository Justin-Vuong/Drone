#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void SPI_Transmit(uint8_t* data, int size);
void SPI_TransmitByte(uint8_t data);
void SPI_Receive (uint8_t* data, int size);
uint8_t SPI_ReceiveByte(void);
uint8_t SPI_RW_Byte(uint8_t data);

#endif //SPI_H