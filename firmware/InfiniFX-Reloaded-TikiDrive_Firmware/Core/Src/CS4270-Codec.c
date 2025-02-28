#include "CS4270-Codec.h"

uint8_t CS4270_REG_CONFIG_SETTINGS[8] = {CS4270_DEVICEID, 	// Device ID
										 0x00,				// Power up
										 0x31,				// Mode (Slave, normal speed, pop guard enable)
										 0x09,				// ADC/DAC (disable HPF, I2S format)
										 0x00,				// Transition (Independent volume controls)
										 0x12,				// Mute right channel
										 0x00,				// DAC A Vol (0dB)
										 0xFF}; 			// DAC B Vol (Mute (-127.5dB))

uint8_t CS4270_Init() {

	HAL_StatusTypeDef i2cStatus;


	// Hard reset of codec
	CS4270_Reset();

	// Put CODEC in power down mode before adjusting configuration (set freeze bit, power down ADC, DAC, enter low power mode)
	i2cStatus = CS4270_RegWrite(CS4270_REG_POWERCONTROL, 0xA3);

	if (i2cStatus != HAL_OK) {

		return 1;

	}


	// Check device ID
	uint8_t deviceID = 0x00;

	i2cStatus = CS4270_RegRead(CS4270_REG_DEVICEID, &deviceID);

	if ( (i2cStatus != HAL_OK) || ((deviceID & 0xF0) != CS4270_DEVICEID)) { // Upper 4 bits = device ID

		return 2;

	}



	// Set configuration registers
	int8_t  regIndex;
	uint8_t regData;

	for (regIndex = CS4270_REG_DACBVOLCONTROL; regIndex >= CS4270_REG_POWERCONTROL; regIndex--) {

		// Write 'default' value to register (Note: not checking reserved bits - default values are 0 in datasheet p.31!)
		i2cStatus = CS4270_RegWrite(regIndex, CS4270_REG_CONFIG_SETTINGS[regIndex - 1]);

		if (i2cStatus != HAL_OK) {

			return regIndex;

		}

		// Read back register value
		i2cStatus = CS4270_RegRead(regIndex, &regData);

		if ((i2cStatus != HAL_OK) || (regData != CS4270_REG_CONFIG_SETTINGS[regIndex - 1])) {

			return regIndex;

		}

	}

	return 0;

}

void CS4270_Reset() {

	HAL_GPIO_WritePin(CS4270_NRST_GPIO_PORT, CS4270_NRST_GPIO_PIN, GPIO_PIN_RESET);
	HAL_Delay(25);

	HAL_GPIO_WritePin(CS4270_NRST_GPIO_PORT, CS4270_NRST_GPIO_PIN, GPIO_PIN_SET);
	HAL_Delay(25);

}

HAL_StatusTypeDef CS4270_RegWrite(uint8_t regAddr, uint8_t regData) {

	return HAL_I2C_Mem_Write(&CS4270_I2C_HANDLE, CS4270_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &regData, 1, HAL_MAX_DELAY);

}

HAL_StatusTypeDef CS4270_RegRead(uint8_t regAddr, uint8_t *regData) {

	return HAL_I2C_Mem_Read(&CS4270_I2C_HANDLE, CS4270_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, regData, 1, HAL_MAX_DELAY);

}
