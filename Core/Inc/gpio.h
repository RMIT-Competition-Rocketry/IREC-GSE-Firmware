/*
 * gpio.h
 *
 *  Created on: Feb 11, 2025
 *      Author: lucas
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f4xx_hal.h"

#define GPIO_MODER_INPUT 0x00
#define GPIO_MODER_GENERAL_PURPOSE_OUTPUT 0x01
#define GPIO_MODER_ALTERNATE_FUNCTION 0x02
#define GPIO_MODER_ANALOG_MODE 0x03

#define GPIO_OTYPER_PUSH 0x00
#define GPIO_OTYPER_DRAIN 0x01

#define GPIO_OSPEEDR_LOW 0x00
#define GPIO_OSPEEDR_MEDIUM 0x01
#define GPIO_OSPEEDR_HIGH 0x02
#define GPIO_OSPEEDR_VERY_HIGH 0x03


#define GPIO_PUPDRy_NO 0x00
#define GPIO_PUPDRy_UP 0x01
#define GPIO_PUPDRy_DOWN 0x02











typedef struct{
	uint8_t MODER;
	uint8_t OTYPER;
	uint8_t OSPEEDR;
	uint8_t PUPDR;
	uint8_t POS;
	GPIO_TypeDef *port;
	uint8_t state; //for storing state values (read and write)
	uint8_t (*read)(struct GPIO *); //check for mode
	uint8_t (*write)(struct GPIO *); //essentially get to choose when to toggle //check for mode first
}GPIO;



//@param Pos is defined via #include "stm32f4xx_hal.h"

void GPIO_init(GPIO *, GPIO_TypeDef *, uint8_t MODER, uint8_t OTYPER, uint8_t OSPEEDR, uint8_t PUPDR, uint8_t POS);
uint8_t GPIO_read(GPIO *);
void GPIO_write(GPIO *);
void GPIO_init_interrupt(GPIO *, GPIO_TypeDef *, uint8_t MODER, uint8_t OTYPER, uint8_t OSPEEDR, uint8_t PUPDR, uint8_t POS);



#endif /* INC_GPIO_H_ */
