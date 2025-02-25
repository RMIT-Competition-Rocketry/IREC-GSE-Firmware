/*
 * flash.h
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "spi.h"
#include "stm32f4xx_hal.h"

#define DEVICE_NAME_LENGTH 20

#define FLASH_PAGE_PROGRAM           0x02
#define FLASH_READ_DATA              0x03
#define FLASH_WRITE_DISABLE          0x04
#define FLASH_WRITE_ENABLE           0x06
#define FLASH_ERASE_CHIP             0x60
#define FLASH_READ_STATUS_REGISTER_1 0x05
#define FLASH_READ_STATUS_REGISTER_2 0x35
#define FLASH_READ_STATUS_REGISTER_3 0x15
//if you write to the chip using these instructions, you will get exactly that^

typedef struct{
	SPI base;
	int pageSize;
	long PageCount;
	void (*erase)(struct Flash *);
	void (*readPage)(struct Flash *, uint32_t,  uint8_t *);
	void (*writePage)(struct Flash *, uint32_t, volatile uint8_t *);
}Flash;
void Flash_init(Flash *, char[], GPIO_TypeDef *, unsigned long, int, long);
void Flash_writeEnable(Flash *);
void Flash_erase(Flash *);
void Flash_readPage(Flash *, uint32_t, volatile uint8_t *);
void Flash_writePage(Flash *, uint32_t, uint8_t *);
void _Flash_readStatus1(Flash *, uint8_t *);
void _Flash_readStatus2(Flash *, uint8_t *);
void _Flash_readStatus3(Flash *, uint8_t *);

#endif /* FLASH_H_ */
