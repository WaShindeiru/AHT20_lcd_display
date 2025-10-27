/*
 * i2c_queue.c
 *
 *  Created on: Oct 25, 2025
 *      Author: washindeiru
 */

#include "i2c_queue.h"

BaseType_t vHandleI2CTask(I2C_Task* task) {
	switch( task->EventType ) {
	case LCDWrite:
		lcd_display((struct lcd_disp*) task->pvData);

		return pdPASS;
		break;
	case AHT20Read:
		AHT20_Make_Measurement((aht20*) task->pvData);

		return pdPASS;
		break;
	default:
		return pdFAIL;
	}
}
