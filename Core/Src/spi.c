/*
 * spi.c
 *
 *  Created on: Feb 11, 2025
 *      Author: lucas
 */

#include "spi.h"

static void SPI_send8(SPI *, uint16_t);
static void SPI_send16(SPI *, uint16_t);

static void SPI_receive8(SPI *, volatile uint16_t *);
static void SPI_receive16(SPI *, volatile uint16_t *);

/* =============================================================================== */
/**
 * @brief Initialiser for an SPI device interface.
 *
 * @param *spi 				Pointer to SPI struct to be initialised.
 * @param device 			Enum specifier for device type.
 * @param *interface 	Pointer to SPI interface struct.
 * @param *port 			Pointer to GPIO port struct.
 * @param cs 					Device chip select address.
 * @return @c NULL.
 **
 * =============================================================================== */
void SPI_init(SPI *spi, DeviceType device, SPI_TypeDef *interface, DataFormat df, GPIO_TypeDef *port, unsigned long cs) {
  spi->device    = device;
  spi->interface = interface; //should be interface 0x40003C00
  spi->port      = port;
  spi->cs        = cs;

  spi->send      = (df == MODE8) ? SPI_send8 : SPI_send16;
  spi->receive   = (df == MODE8) ? SPI_receive8 : SPI_receive16; //if Data format is MODE8
  spi->transmit  = SPI_transmit;
}

/* =============================================================================== */
/**
 * @brief Instance method to communicate a SPI transaction with slave device.
 *
 * @param 	*spi 			Pointer to SPI struct.
 * @param 	data 			Data payload to be sent to slave device.
 * @retval 	response 	Returns the slave device response from the transaction.
 **
 * =============================================================================== */
uint16_t SPI_transmit(SPI *spi, uint16_t data) {
  volatile uint16_t response = 0;
  spi->send(spi, data);
  spi->receive(spi, &response);
  while (spi->interface->SR & SPI_SR_BSY);
  return response;

}

/* =============================================================================== */
/**
 * @brief Send data through the SPI interface.
 *
 * @param 	*spi 			Pointer to SPI struct.
 * @param   data      The data to send.
 * @return @c NULL.
 **
 * =============================================================================== */
static void SPI_send8(SPI *spi, uint16_t data) {
  while (!(spi->interface->SR & SPI_SR_TXE));
  spi->interface->DR = (uint8_t) data;
}

static void SPI_send16(SPI *spi, uint16_t data) {
  while (!(spi->interface->SR & SPI_SR_TXE));
  spi->interface->DR = data;
}


/* =============================================================================== */
/**
 * @brief Receive data through the SPI interface.
 *
 * @param 	*spi 			Pointer to SPI struct.
 * @param   data      Pointer to variable to receive data into.
 * @return @c NULL.
 **
 * =============================================================================== */
static void SPI_receive8(SPI *spi, volatile uint16_t *data) {
  while (!(spi->interface->SR & SPI_SR_RXNE));
  *data = (uint8_t) spi->interface->DR;
}

static void SPI_receive16(SPI *spi, volatile uint16_t *data) {
  while (!(spi->interface->SR & SPI_SR_RXNE));
  *data = spi->interface->DR;
}

/*
uint16_t SPI6_TransmitReceive( uint16_t data) {
    volatile uint16_t response = 0;

    // Wait until SPI is not busy
    while (SPI6->SR & SPI_SR_BSY);

    // Assert CS (low)
    GPIOG->ODR &= ~(GPIO_ODR_OD11);

    // Wait until TXE flag is set and write data to DR
    while (!(SPI6->SR & SPI_SR_TXE));
    SPI6->DR = data;

    // Wait until RXNE flag is set and read data from DR
    while (!(SPI6->SR & SPI_SR_RXNE));
    response = SPI6->DR;

    // De-assert CS (high)
    GPIOG->ODR |= GPIO_ODR_OD11;

    return response;
}*/



