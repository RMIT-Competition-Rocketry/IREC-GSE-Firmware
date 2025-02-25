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
#include "sensors.h"
#include "spi.h"

#define LORA_REG_FIFO                   0x00
#define LORA_REG_FIFO_ADDR_PTR          0x0D
#define LORA_REG_FIFO_TX_BASE_ADDR      0x0E
#define LORA_REG_FIFO_RX_BASE_ADDR      0x0F
#define LORA_REG_FIFO_RX_CURR_ADDR      0x10
#define LORA_REG_IRQ_FLAGS_MASK         0x11
#define LORA_REG_IRQ_FLAGS              0x12

#define LORA_REG_OP_MODE                0x01
#define LORA_REG_OP_MODE_LONG_RANGE_Pos 0x07
#define LORA_REG_OP_MODE_MODE_Pos       0x00

#define LORA_REG_MODEM_CONFIG1          0x1D
#define LORA_REG_MODEM_CONFIG1_BW_Pos   0x06
#define LORA_REG_MODEM_CONFIG1_CR_Pos   0x03
#define LORA_REG_MODEM_CONFIG1_CRC_Pos  0x01

#define LORA_REG_MODEM_CONFIG2          0x1E
#define LORA_REG_MODEM_CONFIG2_SF_Pos   0x04

#define LORA_REG_PAYLOAD_LENGTH         0x22
#define LORA_REG_MAX_PAYLOAD_LENGTH     0x23

#define RegSymbTimeoutLsb               0x1F
#define RegPreambleMsb                  0x20
#define RegPreambleLsb                  0x21
#define RegHopPeriod                    0x24
#define RegFifoRxByteAddr               0x25
#define RegDioMapping1                  0x40
#define RegDioMapping2                  0x41

#define LORA_MSG_LENGTH                 0x20 //32
#define LORA_MSG_PAYLOAD_LENGTH         (LORA_MSG_LENGTH - 1)

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
  BW125,
  BW250,
  BW500,
} Bandwidth;

typedef enum {
  CR5 = 1,
  CR6,
  CR7,
  CR8,
} CodingRate;

typedef enum {
  SF6 = 6,
  SF7,
  SF8,
  SF9,
  SF10,
  SF11,
  SF12,
} SpreadingFactor;

typedef enum {
  SLEEP,
  STDBY,
  FSTX,
  TX,
  FSRX,
  RXCONTINUOUS,
  RXSINGLE,
  CAD
} Mode;

typedef struct {
  uint8_t id;                            //!< Packet header ID - hence minus 1
  uint8_t data[LORA_MSG_PAYLOAD_LENGTH]; //!< Packet payload
} LoRa_Packet;

/** @extends SPI */
typedef struct LoRa {
  SPI base;                                   //!< Parent SPI interface
  void (*transmit)(struct LoRa *, uint8_t *); //!< LoRa transmit method. @see LoRa_transmit
} LoRa;

void LoRa_init(LoRa *, char[DEVICE_NAME_LENGTH], GPIO_TypeDef *, unsigned long, Bandwidth, SpreadingFactor, CodingRate);
void LoRa_transmit(LoRa *, uint8_t *);
void LoRa_receive(LoRa *, uint8_t *);
void LoRa_writeRegister(LoRa *, uint8_t, uint8_t);
uint8_t LoRa_readRegister(LoRa *, uint8_t);
uint8_t CheckRX(LoRa *);

LoRa_Packet LoRa_AVData(
    uint8_t,
    uint8_t,
    uint8_t *,
    uint8_t *,
    uint8_t,
    uint8_t *,
    uint8_t,
    float,
    float
);
LoRa_Packet LoRa_GPSData(uint8_t, char *, char *, uint8_t);
LoRa_Packet LoRa_PayloadData(uint8_t, uint8_t, uint8_t *, uint8_t);
LoRa_Packet LoRa_GSEData_1(
		uint8_t,
		ADC124S021 *,
		MCP96RL00_EMX_1 *,
		MCP96RL00_EMX_1 *,
		MCP96RL00_EMX_1 *,
		MCP96RL00_EMX_1 *,
		uint16_t
		);
LoRa_Packet LoRa_GSEData_2	(
		uint8_t,
		ADC124S021 *,
		ADT75ARMZ *,
		uint16_t
		);

LoRa_Packet LoRa_Command (uint8_t[LORA_MSG_LENGTH]);

//for the packet that will receive information, to a memcpy operation to move FIFO data to the custom struct! for RX data
void _LoRa_setMode(LoRa *, Mode);
void LoRa_TimerInt();

#endif /* SRC_LORA_H_ */
