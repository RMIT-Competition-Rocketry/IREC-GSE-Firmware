/*
 * spi.h
 *
 *  Created on: Feb 11, 2025
 *      Author: lucas
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_
#include "stm32f4xx_hal.h"

/**
 * @brief Device type enum
 * Describes the type of peripheral implementing an SPI interface.
 */
typedef enum {
  MEMORY_FLASH, //!< Flash memory.
  COMM_LORA,     //!< LoRa module
  ADC_TRANSDUCER //ADC -> SPI bus for load cells and such
} DeviceType;

/**
 * @brief Data format enum
 * Describes the format of dataframes sent on the SPI data bus.
 */
typedef enum {
  MODE8,
  MODE16
} DataFormat;

/**
 * @brief Struct definition for \ref SPI "SPI interface".
 * Provides the interface for API consumers to interact with the SPI peripheral.
 */
typedef struct SPI {
  DeviceType device;                                  //!< Enum specifier for device type.
  SPI_TypeDef *interface;                             //!< Pointer to SPI interface struct.
  GPIO_TypeDef *port;                                 //!< Pointer to GPIO port struct.
  unsigned long cs;                                   //!< Device chip select address.
  void (*send)(struct SPI *, uint16_t);               //!< SPI send method.     @see SPI_send
  void (*receive)(struct SPI *, volatile uint16_t *); //!< SPI receive method.  @see SPI_receive
  uint16_t (*transmit)(struct SPI *, uint16_t);       //!< SPI transmit method. @see SPI_transmit
} SPI;

void SPI_init(SPI *, DeviceType, SPI_TypeDef *, DataFormat, GPIO_TypeDef *, unsigned long);
void SPI_send(SPI *, uint16_t);
void SPI_receive(SPI *, volatile uint16_t *);
uint16_t SPI_transmit(SPI *, uint16_t);

#endif /* INC_SPI_H_ */
