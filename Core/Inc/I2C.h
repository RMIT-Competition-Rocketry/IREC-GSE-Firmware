/*
 * I2C.h
 *
 *  Created on: Feb 11, 2025
 *      Author: lucas
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_


#include "stm32f4xx_hal.h"


typedef enum{
	THERMOCOUPLE,
	TEMPERATURE_SENSOR
} Type;


typedef struct{
	I2C_TypeDef *interface;
	GPIO_TypeDef *port;
	Type device;
	uint8_t address;
	void (*send)(struct I2C *, uint8_t,uint8_t);
	void (*receive)(struct I2C *,uint8_t, volatile uint8_t *);
	void (*sendBurst)(struct I2C *, uint8_t[], uint8_t, uint8_t);
	void (*MultiReceive)(struct I2C *, volatile uint8_t *, uint8_t, uint8_t);
}I2C;

void I2C_send(I2C *, uint8_t, uint8_t );
void I2C_sendByte(I2C *, uint8_t);
void I2C_sendBurst(I2C *, uint8_t[], uint8_t, uint8_t);
void I2C_receive(I2C *, uint8_t, volatile uint8_t *);
void I2C_MultiReceive(I2C *, volatile uint8_t *, uint8_t, uint8_t);
void I2C_init(I2C *, I2C_TypeDef *, GPIO_TypeDef *,Type, uint8_t);



void I2C_TempExtract(I2C *, volatile uint8_t *, uint8_t , uint8_t);


#endif /* INC_I2C_H_ */
