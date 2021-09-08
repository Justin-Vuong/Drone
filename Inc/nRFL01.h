#ifndef NRFL01_H
#define NRFL01_H

#include <stdbool.h>
#include <stdint.h>

//SPI commands for transceiver
#define R_REGISTER(ADDR)        (0x00 | ADDR)
#define W_REGISTER(ADDR)        (0x20 | ADDR)
#define R_RX_PAYLOAD            0x61
#define W_TX_PAYLOAD            0xA0
#define FLUSH_TX                0xE1
#define FLUSH_RX                0xE2
#define REUSE_TX_PL             0xE3
#define R_RX_PL_WID             0x60
#define NOP                     0xFF 

void nRFL01_Config(void);
void CS_Enable(void);
void CS_Disable (void);
void CE_Enable(void);
void CE_Disable (void);
void nRFL01_Write_Reg(uint8_t addr, uint8_t value);
uint8_t nRFL01_Read_Reg(uint8_t addr);
void nRFL01_Write_Regs(uint8_t addr, uint8_t* values, int size);
void nRFL01_TX_Init(void);
void nRFL01_Write_Tx_Payload(uint8_t* values, int size);
void nRFL01_RX_Init(void);
void nRFL01_RX_Read_Payload(uint8_t* data, uint8_t dataSz);
void nRFL01_heartbeat(uint8_t* data);
uint8_t nRFL01_check_registers(uint8_t* data, bool isRX);
void nRFL01_clear_max_retries(void);

#endif //NRFL01_H