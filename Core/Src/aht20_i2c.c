/*
 * aht20.c
 *
 *  Created on: Oct 25, 2025
 *      Author: washindeiru
 */

#include "aht20_i2c.h"
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "FreeRTOS.h"

HAL_StatusTypeDef AHT20_Get_Status(uint8_t* statusWord) {
	return HAL_I2C_Mem_Read(&HI2C_DEF, AHT_ADDRESS, AHT_STATUS_CODE, 1, statusWord, 1, 1000);
}

HAL_StatusTypeDef AHT20_Initialize() {
	uint8_t commands[3] = {0xbe, 0x08, 0x00};
	HAL_StatusTypeDef returnValue;

	returnValue = HAL_I2C_Master_Transmit(&HI2C_DEF, AHT_ADDRESS, commands, 3, 1000);
	vTaskDelay(pdMS_TO_TICKS(10));

	return returnValue;
}

HAL_StatusTypeDef AHT20_Make_Measurement(aht20* aht) {
	uint8_t aht_buffer[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t commands[3] = {0xAC, 0x33, 0x00};
	HAL_StatusTypeDef returnValue;

	returnValue = HAL_I2C_Master_Transmit(&HI2C_DEF, AHT_ADDRESS, commands, 3, 1000);
	if (returnValue != HAL_OK) {
		return returnValue;
	}

	vTaskDelay(pdMS_TO_TICKS(80));

	uint8_t statusWord = 0x80;

	while ((statusWord>>7 & 0x01) == 1) {
		AHT20_Get_Status(&statusWord);
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	HAL_I2C_Master_Receive(&HI2C_DEF, AHT_ADDRESS, aht_buffer, 7, 1000);

	uint32_t hum_data = (aht_buffer[1]<<16)|(aht_buffer[2]<<8)|aht_buffer[3];
	hum_data = hum_data>>4;
	aht->humidity = (float) ((hum_data / pow(2, 20)) * 100);

	uint32_t temp_data = (aht_buffer[3]<<16)|(aht_buffer[4]<<8)|aht_buffer[5];
	temp_data = temp_data & 0xFFFFF;
	aht->temperature = (float) (((temp_data / pow(2, 20)) * 200) - 50);

	return returnValue;
}
