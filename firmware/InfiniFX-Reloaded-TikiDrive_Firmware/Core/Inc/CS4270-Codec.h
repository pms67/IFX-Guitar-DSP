#ifndef CS4270_CODEC_H
#define CS4270_CODEC_H

#include "main.h"

// I2C handle and reset pin definitions
#define CS4270_I2C_HANDLE		hi2c1
#define CS4270_NRST_GPIO_PORT 	CODEC_NRST_GPIO_Port
#define CS4270_NRST_GPIO_PIN	CODEC_NRST_Pin

// I2C address
#define CS4270_I2C_ADDRESS				(0x48 << 1) // 0b[1001 A2 A1 A0], A[2:0] = 0 --> 0b1001000 = 0x48

// Device ID
#define CS4270_DEVICEID					0xC0

// Register addresses
#define CS4270_REG_DEVICEID				0x01
#define CS4270_REG_POWERCONTROL			0x02

#define CS4270_REG_MODECONTROL			0x03
#define CS4270_REG_ADCDACCONTROL		0x04
#define CS4270_REG_TRANSITIONCONTROL	0x05
#define CS4270_REG_MUTECONTROL			0x06
#define CS4270_REG_DACAVOLCONTROL		0x07
#define	CS4270_REG_DACBVOLCONTROL		0x08


// Register config (default, set on init())
extern uint8_t CS4270_REG_CONFIG_SETTINGS[8];

// I2C handle
extern I2C_HandleTypeDef CS4270_I2C_HANDLE;

// Functions
uint8_t CS4270_Init();
void 	CS4270_Reset();

HAL_StatusTypeDef CS4270_RegWrite(uint8_t regAddr, uint8_t  regData);
HAL_StatusTypeDef CS4270_RegRead (uint8_t regAddr, uint8_t *regData);

#endif
