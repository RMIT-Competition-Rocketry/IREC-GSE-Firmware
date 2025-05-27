/*
 * lora.h
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */

#ifndef SRC_LORA_H_
#define SRC_LORA_H_

#include "stm32f439xx.h"
#include "string.h"
#include "spi.h"
#include "stdbool.h"
#include "gpio.h"


extern GPIO LORA_CS_GPIO;
#define LORA_CS GPIO_ODR_OD11


#define SX1272_REG_FIFO                  0x00
#define SX1272_REG_FIFO_ADDR_PTR         0x0D
#define SX1272_REG_FIFO_TX_BASE_ADDR     0x0E
#define SX1272_REG_FIFO_RX_BASE_ADDR     0x0F
#define SX1272_REG_FIFO_RX_CURR_ADDR     0x10

#define SX1272_REG_RX_BYTES              0x13

#define SX1272_REG_DIO_MAPPING1          0x40
#define SX1272_DIO_MAPPING_DIO0_Pos      0x06
#define SX1272_DIO_MAPPING_DIO1_Pos      0x04
#define SX1272_DIO_MAPPING_DIO2_Pos      0x02
#define SX1272_DIO_MAPPING_DIO3_Pos      0x00
#define SX1272_DIO_MAPPING_DIO4_Pos      0x06
#define SX1272_DIO_MAPPING_DIO5_Pos      0x04

#define SX1272_LORA_DIO_RXDONE           0x00 << SX1272_DIO_MAPPING_DIO0_Pos
#define SX1272_LORA_DIO_TXDONE           0x01 << SX1272_DIO_MAPPING_DIO0_Pos

#define SX1272_REG_FR_MIB				 0x07
#define SX1272_REG_FR_LSB				 0x08

#define SX1272_REG_IRQ_FLAGS_MASK        0x11
#define SX1272_REG_IRQ_FLAGS             0x12
#define SX1272_LORA_IRQ_RXDONE           0x40
#define SX1272_LORA_IRQ_TXDONE           0x08

#define SX1272_REG_OP_MODE               0x01
#define SX1272_OP_MODE_LONG_RANGE_Pos    0x07
#define SX1272_OP_MODE_MODE_Pos          0x00

#define SX1272_REG_MODEM_CONFIG1         0x1D
#define SX1272_REG_MODEM_CONFIG1_BW_Pos  0x06
#define SX1272_REG_MODEM_CONFIG1_CR_Pos  0x03
#define SX1272_REG_MODEM_CONFIG1_CRC_Pos 0x01

#define SX1272_REG_MODEM_CONFIG2         0x1E
#define SX1272_REG_MODEM_CONFIG2_SF_Pos  0x04

#define SX1272_REG_PA_CONFIG             0x09
#define SX1272_PA_SELECT                 0x80

#define SX1272_REG_LNA                   0x0C

#define SX1272_REG_PAYLOAD_LENGTH        0x22
#define SX1272_REG_MAX_PAYLOAD_LENGTH    0x23

// TODO:
// Extract these out of the device driver
// in favour of passing as argument

#define LORA_MSG_LENGTH                  0x20
#define LORA_MSG_PAYLOAD_LENGTH          (LORA_MSG_LENGTH - 1)

#define DEVICE_NAME_LENGTH				0x20

#define CadDetect						0x01
#define FhssChangeChannel				0x01 << 1
#define CadDone							0x01 << 2
#define TxDone							0x01 << 3
#define ValidHeader						0x01 << 4
#define PayloadCRCError					0x01 << 5
#define RxDone							0x01 << 6
#define RxTimeout						0x01 << 7






/**
 * @addtogroup LoRa
 * @{
 */

typedef enum {
  SX1272_BW125, // 125kHz
  SX1272_BW250, // 250kHz
  SX1272_BW500, // 500kHz
} SX1272_Bandwidth;

typedef enum {
  SX1272_CR5 = 1, // 4/5
  SX1272_CR6,     // 4/6
  SX1272_CR7,     // 4/7
  SX1272_CR8,     // 4/8
} SX1272_CodingRate;

typedef enum {
  SX1272_SF6 = 6,
  SX1272_SF7,
  SX1272_SF8,
  SX1272_SF9,
  SX1272_SF10,
  SX1272_SF11,
  SX1272_SF12,
} SX1272_SpreadingFactor;

typedef enum {
  SX1272_MODE_SLEEP,        // Low power mode. Only SPI and config registers available
  SX1272_MODE_STDBY,        // Standby mode. Chip is active, RF is disabled
  SX1272_MODE_FSTX,         // Frequency synthesis transmission mode
  SX1272_MODE_TX,           // Transmission mode
  SX1272_MODE_FSRX,         // Frequency synthesis receive mode
  SX1272_MODE_RXCONTINUOUS, // Continuous receive mode
  SX1272_MODE_RXSINGLE,     // Single receive mode
  SX1272_MODE_CAD           // Channel activity detection mode
} SX1272_Mode;

typedef struct {
  uint8_t id;                            //!< Packet header ID - hence minus 1
  uint8_t data[LORA_MSG_PAYLOAD_LENGTH]; //!< Packet payload
} LoRa_Packet;

/** @extends SPI */
typedef struct SX1272 {
  SPI_t * base;                                                //!< Parent SPI interface
  SX1272_Mode currentMode;                                  //!< Current operating mode.
  void (*enableBoost)(struct SX1272 *, bool);               //!< Power amp boost toggle method.          @see SX1272_enableBoost
  void (*standby)(struct SX1272 *);                         //!< SX1272 standby method.                  @see SX1272_standby
  void (*transmit)(struct SX1272 *, uint8_t *);             //!< SX1272 LoRa transmit method.            @see SX1272_transmit
  void (*startReceive)(struct SX1272 *);                    //!< SX1272 LoRa continuous receive method.  @see SX1272_startReceive
  bool (*readReceive)(struct SX1272 *, uint8_t *, uint8_t); //!< SX1272 LoRa receive buffer read method. @see SX1272_readReceiveBuffer
  void (*clearIRQ)(struct SX1272 *, uint8_t);               //!< SX1272 LoRa IRQ flag clear method.      @see SX1272_clearIRQ
} SX1272_t;


void SX1272_init(SX1272_t *, char[DEVICE_NAME_LENGTH], GPIO_TypeDef *, unsigned long, SX1272_Bandwidth, SX1272_SpreadingFactor, SX1272_CodingRate);
void SX1272_enableBoost(SX1272_t *, bool);
void SX1272_standby(SX1272_t *);
void SX1272_transmit(SX1272_t *, uint8_t *);
void SX1272_startReceive(SX1272_t *);
bool SX1272_readReceive(SX1272_t *, uint8_t *, uint8_t);
void SX1272_clearIRQ(SX1272_t *, uint8_t);
void _SX1272_setMode(SX1272_t *, SX1272_Mode);

void SX1272_writeRegister(SX1272_t *, uint8_t, uint8_t);
uint8_t SX1272_readRegister(SX1272_t *, uint8_t);




LoRa_Packet LoRa_Command (uint8_t[LORA_MSG_LENGTH]);
LoRa_Packet Dummy_Transmit();
LoRa_Packet Dummy_Transmit_2();

//for the packet that will receive information, to a memcpy operation to move FIFO data to the custom struct! for RX data
void _LoRa_setMode(SX1272_t *, SX1272_Mode);
void LoRa_TimerInt();

#endif /* SRC_LORA_H_ */
