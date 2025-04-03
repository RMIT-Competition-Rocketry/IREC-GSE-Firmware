/*
 * I2C.c
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */
#include "I2C.h"
#include "stm32f4xx_hal.h"

//volatile uint8_t timeout_i2c = 0;
void I2C_init(I2C *i2c, I2C_TypeDef *interface,GPIO_TypeDef *port,Type device, uint8_t Mst_addr, uint8_t error_flag)
{
	i2c->interface = interface;
	i2c->device = device;
	i2c->port = port;
	i2c->receive = I2C_receive;
	i2c->send = I2C_send;
	i2c->address = Mst_addr;
	i2c->error_flag = error_flag;
}

void I2C_recovery(I2C *i2c)
{
	i2c->port->MODER &= ~GPIO_MODER_MODER1;
	i2c->port->MODER |= (1<<GPIO_MODER_MODER1_Pos);//output mode
	for (uint8_t i = 0; i<16; i++)
	{
		i2c->port->BSRR = GPIO_BSRR_BR_1;
		delay_software_us(5);
		i2c->port->BSRR = GPIO_BSRR_BS_1;
		delay_software_us(5);
	}
	i2c->port->MODER |= (2<<GPIO_MODER_MODER1_Pos);//output mode
}
void I2C_send(I2C *i2c, uint8_t address, uint8_t data)
{
	uint32_t timeout = 0;
	i2c->interface->CR1 |= I2C_CR1_START;
	delay_software_us(10);

	while(!(i2c->interface->SR1 & I2C_SR1_SB)) //bitmask first bit of error flag
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_BSY)
		{
			I2C_recovery(i2c);
			i2c->interface->CR1 &= ~I2C_CR1_SWRST;
			i2c->interface->CR1 |= I2C_CR1_SWRST;
			i2c->error_flag |= 1;
			if(i2c->interface->SR1 & I2C_SR1_TIMEOUT){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			if(i2c->interface->SR1 & I2C_SR1_BERR){i2c->error_flag |=(0x01<<3);}else{i2c->error_flag &=~(0x01<<3);}
			return;
		}
	}
	i2c->error_flag &= ~1;
	timeout = 0;
	i2c->interface->DR = (address << 1) & ~0x01; //for write bit
	while(!(i2c->interface->SR1 & I2C_SR1_ADDR))//ACK bit
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_ACK)
		{
			if(i2c->interface->SR1 & I2C_SR1_AF){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			return; //breaks out of loop, failed I2C transaction
		}
	}
		i2c->error_flag &= ~(0x01<<1);

		(void)i2c->interface->SR1; //clears the SR
		(void)i2c->interface->SR2; //clears the SR

	while((i2c->interface->SR1 & I2C_SR1_TXE));
		I2C1->DR = data;
	while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed
	i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit
}

void I2C_sendBurst(I2C *i2c, uint8_t data[], uint8_t size, uint8_t address)
{
	uint32_t timeout = 0;
	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB)&& (i2c->error_flag == (0 & 0x01)))
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_BSY)
		{
			I2C_recovery(i2c);
			i2c->interface->CR1 &= ~I2C_CR1_SWRST;
			i2c->interface->CR1 |= I2C_CR1_SWRST;
			i2c->error_flag |= 1;
			if(i2c->interface->SR1 & I2C_SR1_TIMEOUT){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			if(i2c->interface->SR1 & I2C_SR1_BERR){i2c->error_flag |=(0x01<<3);}else{i2c->error_flag &=~(0x01<<3);}
			return;
		}
	}

	i2c->error_flag &= ~1;
	i2c->interface->DR = (address << 1) & ~0x01; //for write bit -> loading the address into the I2C Data register
	while(!(i2c->interface->SR1 & I2C_SR1_ADDR))//ACK bit -> waits for ACK bit to be asserted!
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_ACK)
		{
			if(i2c->interface->SR1 & I2C_SR1_AF){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			return; //breaks out of loop, failed I2C transaction
		}
	}
	(void)i2c->interface->SR1; //clears the SR
	(void)i2c->interface->SR2; //clears the SR

for(int i = 0; i<size ; i++)
{
	while(i2c->interface->SR1 & I2C_SR1_TXE);
		i2c->interface->DR = data[i];
		while(!(i2c->interface->SR1 & I2C_SR1_BTF)); //wait for byte transfer to be completed
}
	i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit
}


void I2C_receive(I2C *i2c, uint8_t address, volatile uint8_t *data)
{
	uint32_t timeout = 0;
	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB)&& (i2c->error_flag == (0 & 0x01)))
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_BSY)
		{
			I2C_recovery(i2c);
			i2c->interface->CR1 &= ~I2C_CR1_SWRST;
			i2c->interface->CR1 |= I2C_CR1_SWRST;
			i2c->error_flag |= 1;
			if(i2c->interface->SR1 & I2C_SR1_TIMEOUT){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			if(i2c->interface->SR1 & I2C_SR1_BERR){i2c->error_flag |=(0x01<<3);}else{i2c->error_flag &=~(0x01<<3);}
			return;
		}
	}
	i2c->error_flag &= ~1;
	i2c->interface->DR = (address << 1) | 0x01; //for write bit
	while(!(i2c->interface->SR1 & I2C_SR1_ADDR))//ACK bit
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_ACK)
		{
			if(i2c->interface->SR1 & I2C_SR1_AF){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			return; //breaks out of loop, failed I2C transaction
		}
	}
		(void)i2c->interface->SR1; //clears the SR
		(void)i2c->interface->SR2; //clears the SR


		while((i2c->interface->SR1 & I2C_SR1_RXNE)); //wait for receiver data register to be empty
		data = i2c->interface->DR;

	while(i2c->interface->SR1 & I2C_SR1_BTF); //wait for byte transfer to be completed
		i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit
}


 void I2C_MultiReceive(I2C * i2c, volatile uint8_t *data, uint8_t address, uint8_t size)
{
		uint32_t timeout = 0;
		i2c->interface->CR1 |= I2C_CR1_START;
		while(!(i2c->interface->CR1 & I2C_SR1_SB)&& (i2c->error_flag == (0 & 0x01)))
		{
			timeout++;
			if(timeout> TIMEOUT_I2C_BSY)
			{
				I2C_recovery(i2c);
				i2c->interface->CR1 &= ~I2C_CR1_SWRST;
				i2c->interface->CR1 |= I2C_CR1_SWRST;
				i2c->error_flag |= 1;
				if(i2c->interface->SR1 & I2C_SR1_TIMEOUT){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
				if(i2c->interface->SR1 & I2C_SR1_BERR){i2c->error_flag |=(0x01<<3);}else{i2c->error_flag &=~(0x01<<3);}
				return;
			}
		}
		i2c->error_flag &= ~1;
	i2c->interface->DR = (address << 1) | 0x01; //for write bit
	while(!(i2c->interface->SR1 & I2C_SR1_ADDR))//ACK bit -> this hold indefinitely currently
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_ACK)
		{
			if(i2c->interface->SR1 & I2C_SR1_AF){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			return; //breaks out of loop, failed I2C transaction
		}
	}
		(void)i2c->interface->SR1; //clears the SR
		(void)i2c->interface->SR2; //clears the SR

	for(uint8_t i = 0; i<size; i++){
		while(!(i2c->interface->SR1 & I2C_SR1_RXNE)); //wait for receiver data register to be empty
		data[i] = i2c->interface->DR;
	while(!(i2c->interface->SR1 & I2C_SR1_BTF)); //wait for byte transfer to be completed
	}
		i2c->interface->CR1 |= I2C_CR1_STOP; //clears start bit

}


void I2C_TempExtract(I2C * i2c, volatile uint8_t *data, uint8_t address, uint8_t command)
{
	uint32_t timeout = 0;
	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB)&& (i2c->error_flag == (0 & 0x01)))
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_BSY)
		{
			I2C_recovery(i2c);
			i2c->interface->CR1 &= ~I2C_CR1_SWRST;
			i2c->interface->CR1 |= I2C_CR1_SWRST;
			i2c->error_flag |= 1;
			if(i2c->interface->SR1 & I2C_SR1_TIMEOUT){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			if(i2c->interface->SR1 & I2C_SR1_BERR){i2c->error_flag |=(0x01<<3);}else{i2c->error_flag &=~(0x01<<3);}
			return;
		}
	}
	i2c->error_flag &= ~1;
	i2c->interface->DR = ((address << 1) & ~0x01); //for write bit
	while(i2c->interface->SR1 & I2C_SR1_ADDR)//ACK bit
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_ACK)
		{
			if(i2c->interface->SR1 & I2C_SR1_AF){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			return; //breaks out of loop, failed I2C transaction
		}
	}
	(void)i2c->interface->SR1; //clears the SR
	(void)i2c->interface->SR2; //clears the SR

	i2c->interface->CR1 |= I2C_CR1_START;
	while(!(i2c->interface->CR1 & I2C_SR1_SB)&& (i2c->error_flag == (0 & 0x01)))
	{
		timeout++;
		if(timeout> TIMEOUT_I2C_BSY)
		{
			I2C_recovery(i2c);
			i2c->interface->CR1 &= ~I2C_CR1_SWRST;
			i2c->interface->CR1 |= I2C_CR1_SWRST;
			i2c->error_flag |= 1;
			if(i2c->interface->SR1 & I2C_SR1_TIMEOUT){i2c->error_flag |=(0x01<<2);}else{i2c->error_flag &=~(0x01<<2);}
			if(i2c->interface->SR1 & I2C_SR1_BERR){i2c->error_flag |=(0x01<<3);}else{i2c->error_flag &=~(0x01<<3);}
			break;
		}
	}
	i2c->error_flag &= ~1;
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

