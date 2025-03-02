/*
 * I2C.c
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */
#include "I2C.h"
#include "stm32f4xx_hal.h"


void I2C_init(I2C *i2c, I2C_TypeDef *interface,GPIO_TypeDef *port,Type device, uint8_t Mst_addr)
{
	i2c->interface = interface;
	i2c->device = device;
	i2c->port = port;
	i2c->receive = I2C_receive;
	i2c->send = I2C_send;
	i2c->address = Mst_addr;
}


//when reading
void I2C_send(I2C *i2c, uint8_t address, uint8_t data)
{
	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB));

	i2c->interface->DR = (address << 1) & ~0x01; //for write bit
	while(!(i2c->interface->SR1 & I2C_SR1_ADDR));//ACK bit
		(void)i2c->interface->SR1; //clears the SR
		(void)i2c->interface->SR2; //clears the SR

	while((i2c->interface->SR1 & I2C_SR1_TXE));
		I2C1->DR = data;
	while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed
	i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit
}

void I2C_sendBurst(I2C *i2c, uint8_t data[], uint8_t size, uint8_t address)
{
	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB));

	i2c->interface->DR = (address << 1) & ~0x01; //for write bit
	while(!(i2c->interface->SR1 & I2C_SR1_ADDR));//ACK bit
	(void)i2c->interface->SR1; //clears the SR
	(void)i2c->interface->SR2; //clears the SR

for(int i = 0; i<size ; i++)
{
	while(i2c->interface->SR1 & I2C_SR1_TXE);
		i2c->interface->DR = data[i];
		while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed
}
	i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit
}


void I2C_receive(I2C *i2c, uint8_t address, volatile uint8_t *data)
{
	i2c->interface->CR1 |= I2C_CR1_START;
	while((i2c->interface->CR1 & I2C_SR1_SB));

	i2c->interface->DR = (address << 1) | 0x01; //for write bit
	while((!i2c->interface->SR1 & I2C_SR1_ADDR));//ACK bit
		(void)i2c->interface->SR1; //clears the SR
		(void)i2c->interface->SR2; //clears the SR


		while((i2c->interface->SR1 & I2C_SR1_RXNE)); //wait for receiver data register to be empty
		data = i2c->interface->DR;

	while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed
		i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit
}


 void I2C_MultiReceive(I2C * i2c, volatile uint8_t *data, uint8_t address, uint8_t size)
{
	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB));

	i2c->interface->DR = (address << 1) | 0x01; //for write bit
	while(!(i2c->interface->SR1 & I2C_SR1_ADDR));//ACK bit -> this hold indefinitely currently
		(void)i2c->interface->SR1; //clears the SR
		(void)i2c->interface->SR2; //clears the SR

	for(uint8_t i = 0; i<size; i++){
		while((i2c->interface->SR1 & I2C_SR1_RXNE)); //wait for receiver data register to be empty
		data[i] = i2c->interface->DR;
	while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed
	}
		i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit

}


void I2C_TempExtract(I2C * i2c, volatile uint8_t *data, uint8_t address, uint8_t command)
{
	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB));

	i2c->interface->DR = ((address << 1) & ~0x01); //for write bit
	while(i2c->interface->SR1 & I2C_SR1_ADDR);//ACK bit
	(void)i2c->interface->SR1; //clears the SR
	(void)i2c->interface->SR2; //clears the SR

	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB));

		while((i2c->interface->SR1 & I2C_SR1_TXE));
				i2c->interface->DR = command;
			while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed

	for(uint8_t i = 0; i<2; i++){
		while((i2c->interface->SR1 & I2C_SR1_RXNE)); //wait for receiver data register to be empty
		data[i] = i2c->interface->DR;
		while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed
	}
		i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit


}

