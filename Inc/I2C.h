#include <stdint.h>
#include "stm32f767xx.h"

#ifndef I2C_H
#define I2C_H

void I2C_Config(void);

//MPU9250 functions
uint16_t I2C_Read2ByteMpuRegister(uint8_t reg_addr);
uint8_t I2C_ReadMpuRegister(uint8_t reg_addr);
void I2C_WriteMpuRegister(uint8_t reg_addr, uint8_t reg_value);
void I2C_ReadMpuGyroRegisters(int16_t* gyroData);
void I2C_ReadMpuAccelRegisters(int16_t* accelData);

//VL53L0X functions
void I2C_VL53L0X_WriteMulti(uint8_t devAddr, uint8_t registerNum, uint8_t* pData, uint32_t dataSz);
void I2C_VL53L0X_ReadMulti(uint8_t devAddr, uint8_t registerNum, uint8_t* pData, uint32_t dataSz);

#endif //I2C_H
