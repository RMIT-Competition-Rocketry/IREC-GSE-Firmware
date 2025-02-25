/*
 * flash.c
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */


#include "flash.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "spi.h"

//might have to return as a struct
void Flash_init(Flash *flash, char name[DEVICE_NAME_LENGTH], GPIO_TypeDef *port, unsigned long cs, int pageSize, long pageCount)
{
	SPI_init(&flash ->base, MEMORY_FLASH, SPI4, MODE16, port, cs);
	flash->pageSize = pageSize;
	flash->PageCount = pageCount;
	flash->erase = Flash_erase;
	flash->readPage = Flash_readPage;
	flash->writePage = Flash_writePage;
	//actual SPI bus configuration function calling will need to be done in the main.c folder.
}

#ifndef DOXYGEN_PRIVATE

void Flash_writeEnable(Flash *flash)
{
	SPI spi = flash->base;
	spi.port->ODR &= ~spi.cs;
	spi.transmit(&spi, FLASH_WRITE_ENABLE);
	spi.port->ODR |= spi.cs;

}

void _Flash_readStatus1(Flash *flash, uint8_t *status)
{
	_Flash_writeEnable(flash);
	SPI spi = flash->base;

	spi.port->ODR &= !spi.cs;
	*status = spi.transmit(&spi, FLASH_READ_STATUS_REGISTER_1);
	*status = spi.transmit(&spi, 0x0F); //transmitting dummy byte to the Flash IC
	spi.port->ODR |= spi.cs;

}
void _Flash_readStatus2(Flash *flash, uint8_t *status)
{
	_Flash_writeEnable(flash);
		SPI spi = flash->base;

		spi.port->ODR &= ~spi.cs;
		*status = spi.transmit(&spi, FLASH_READ_STATUS_REGISTER_2);
		*status = spi.transmit(&spi, 0x0F); //transmitting dummy byte to the Flash IC
		spi.port->ODR |= spi.cs;
}
void _Flash_readStatus3(Flash *flash, uint8_t *status)
{
	_Flash_writeEnable(flash);
		SPI spi = flash->base;

		spi.port->ODR &= ~spi.cs;
		*status = spi.transmit(&spi, FLASH_READ_STATUS_REGISTER_3);
		*status = spi.transmit(&spi, 0x0F); //transmitting dummy byte to the Flash IC
		spi.port->ODR |= spi.cs;
}




#endif


void Flash_erase(Flash *flash)
{
	_Flash_writeEnable(flash);
	SPI spi = flash->base;
	uint8_t status = 0;

	spi.port->ODR &= ~spi.cs;
	spi.transmit(&spi, FLASH_ERASE_CHIP); //0x60 is written to chip -> will erase chip
	spi.port->ODR |= spi.cs;

	do{
		_Flash_readStatus1(flash, &status);
	}while(status & 0x01); //bit 0 will be set to 1 when a write operation is in progress
}

void Flash_readPage(Flash *flash, uint32_t address, volatile uint8_t *data)
{
	_Flash_writeEnable(flash);
	uint8_t status = 0;

	SPI spi = flash->base;

	spi.port->ODR &= ~ spi.cs;

	spi.transmit(&spi, FLASH_READ_DATA);
	spi.transmit(&spi, address & 0xFF0000 >>16); //first 8 bits of 24 bit address
	spi.transmit(&spi, address & 0xFF00 >>8);
	spi.transmit(&spi, address & 0xFF);

	for(uint16_t i = 0; i<256; i++)
	{
		data[i] = spi.transmit(&spi, 0x0F); //dummy byte
	}
	spi.port->ODR |= spi.cs;
}
void Flash_writePage(Flash *flash, uint32_t address,  uint8_t *data)
{
	_Flash_writeEnable(flash);
		uint8_t status = 0;

		SPI spi = flash->base;

		spi.port->ODR &= ~ spi.cs;

		spi.transmit(&spi, FLASH_PAGE_PROGRAM);
		spi.transmit(&spi, address & 0xFF0000 >>16); //first 8 bits of 24 bit address
		spi.transmit(&spi, address & 0xFF00 >>8);
		spi.transmit(&spi, address & 0xFF);

		for(uint16_t i = 0; i<256; i++)
		{
			spi.transmit(&spi, data[i]);
		}
		spi.port->ODR |= spi.cs;
}

