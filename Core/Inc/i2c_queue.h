/*
 * i2c_queue.h
 *
 *  Created on: Oct 25, 2025
 *      Author: washindeiru
 */

#ifndef INC_I2C_QUEUE_H_
#define INC_I2C_QUEUE_H_

#include "lcd_i2c.h"
#include "aht20_i2c.h"
#include "FreeRTOS.h"

#define I2C_QUEUE_LENGTH 2

typedef enum {
	LCDWrite,
	AHT20Read,
} I2C_Event;

typedef struct {
	I2C_Event EventType;
	void* pvData;
} I2C_Task;

BaseType_t vHandleI2CTask(I2C_Task* task);

#endif /* INC_I2C_QUEUE_H_ */
