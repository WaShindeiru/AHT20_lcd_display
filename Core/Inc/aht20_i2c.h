/*
 * aht20_i2c.h
 *
 *  Created on: Oct 25, 2025
 *      Author: washindeiru
 */

#ifndef INC_AHT20_I2C_H_
#define INC_AHT20_I2C_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define HI2C_DEF hi2c1

#define AHT_ADDRESS 0x38<<1
#define AHT_STATUS_CODE 0x71

typedef struct {
	float humidity;
	float temperature;
} aht20;

HAL_StatusTypeDef AHT20_Get_Status(uint8_t* statusWord);

HAL_StatusTypeDef AHT20_Initialize();

HAL_StatusTypeDef AHT20_Make_Measurement(aht20* aht);

#endif /* INC_AHT20_I2C_H_ */
