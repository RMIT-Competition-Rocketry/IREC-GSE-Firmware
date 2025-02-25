/*
 * lora.c
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */


/***********************************************************************************
 * @file        lora.c                                                             *
 * @author      Matt Ricci                                                         *
 * @addtogroup  LoRa                                                               *
 * @brief       Brief description of the file's purpose.                           *
 *                                                                                 *
 * @todo Implement adjustable packet size                                          *
 * @{                                                                              *
 ***********************************************************************************/
#include "sensors.h"
#include "lora.h"
#include "stm32f4xx_hal.h"



/* SPI3 LORA
 * ------------------------------------
 * Flash Pin  | MCU GPIO Pin  | SIGNAL TYPE
 * -----------|---------------|------------
 * SDI        | PC12          | DATA
 * SDO        | PC11          | DATA
 * SCLK       | PC10          | DATA
 * RST        | PD7           | CONTROL
 * DI0        | PD1           | DATA
 * CS         | PD0           | CONTROL      */


 /**
 * =============================================================================== */


void LoRa_init(
    LoRa *lora,
    char name[DEVICE_NAME_LENGTH],
    GPIO_TypeDef *port,
    unsigned long cs,
    Bandwidth bw,
    SpreadingFactor sf,
    CodingRate cr
) {
  SPI_init(&lora->base, COMM_LORA, SPI3, MODE16, port, cs);
  lora->transmit = LoRa_transmit;

  _LoRa_setMode(lora, SLEEP); // Set mode to sleep

  // Set interrupt pin
  LoRa_writeRegister(lora, RegDioMapping1, (0x00));
  //change interrupt pin to 0x00 for RX interrupt

  /* clang-format off */
  LoRa_writeRegister(lora, LORA_REG_OP_MODE,
     0x01 << LORA_REG_OP_MODE_LONG_RANGE_Pos  // Enable LoRa
  );

  LoRa_writeRegister(lora, LORA_REG_MODEM_CONFIG1,
    bw   << LORA_REG_MODEM_CONFIG1_BW_Pos     // Set bandwidth
  | cr   << LORA_REG_MODEM_CONFIG1_CR_Pos     // Set coding rate
  | 0x00 << LORA_REG_MODEM_CONFIG1_CRC_Pos    // Enable CRC
  );
  /* clang-format on */

  /** @todo set spreading factor */
  LoRa_writeRegister(lora, LORA_REG_MODEM_CONFIG2, 0x94);

  // Set payload length
  LoRa_writeRegister(lora, LORA_REG_PAYLOAD_LENGTH, LORA_MSG_LENGTH);
  LoRa_writeRegister(lora, LORA_REG_MAX_PAYLOAD_LENGTH, LORA_MSG_LENGTH);

  _LoRa_setMode(lora, RXCONTINUOUS); // Set mode to RXCONTINUOUS mode!
}

/********************************** PRIVATE METHODS ********************************/

#ifndef DOXYGEN_PRIVATE

/* =============================================================================== */
/**
 * @brief Sets the operational mode of the LoRa module.
 *
 * @param *lora        Pointer to LoRa struct.
 * @param mode         Desired operational mode to be set.
 * @return @c NULL.
 **
 * =============================================================================== */
void _LoRa_setMode(LoRa *lora, Mode mode) {
  uint8_t regOpMode = LoRa_readRegister(lora, LORA_REG_OP_MODE);
  regOpMode &= ~0x07; // Mask to mode bits
  regOpMode |= mode;  // Set mode
  LoRa_writeRegister(lora, LORA_REG_OP_MODE, regOpMode);
}

#endif

/********************************** STATIC METHODS *********************************/

/* =============================================================================== */
/**
 * @brief Constructs a LoRa packet with accelerometer and gyroscope data, altitude,
 *        and velocity for transmission.
 *
 * @param id           Identifier for the packet.
 * @param currentState Current state to be included in the packet.
 * @param *lAccelData  Pointer to low byte accelerometer data.
 * @param *hAccelData  Pointer to high byte accelerometer data.
 * @param lenAccel     Length of the accelerometer data.
 * @param *gyroData    Pointer to gyroscope data.
 * @param lenGyro      Length of the gyroscope data.
 * @param altitude     Altitude value to be included in the packet.
 * @param velocity     Velocity value to be included in the packet.
 * @return             Constructed LoRa packet containing the provided data.
 **
 * =============================================================================== */
LoRa_Packet LoRa_AVData(
    uint8_t id,
    uint8_t currentState,
    uint8_t *lAccelData,
    uint8_t *hAccelData,
    uint8_t lenAccel,
    uint8_t *gyroData,
    uint8_t lenGyro,
    float altitude,
    float velocity
) {
  LoRa_Packet msg;

  // Convert altitude float to byte array
  union {
    float f;
    uint8_t b[4];
  } a;
  a.f = altitude;

  // Convert velocity float to byte array
  union {
    float f;
    uint8_t b[4];
  } v;
  v.f     = velocity;

  int idx = 0;
  // Append to struct data array
  msg.id          = id;
  msg.data[idx++] = currentState;
  memcpy(&msg.data[idx], lAccelData, lenAccel); //lenAccel -> sizeof(uint16_t)
  memcpy(&msg.data[idx += lenAccel], hAccelData, lenAccel);
  memcpy(&msg.data[idx += lenAccel], gyroData, lenGyro);
  memcpy(&msg.data[idx += lenGyro], a.b, sizeof(float));
  memcpy(&msg.data[idx += sizeof(float)], v.b, sizeof(float));

  return msg;
}

LoRa_Packet LoRa_GPSData(
    uint8_t id,
    char *latitude,
    char *longitude,
    uint8_t flags
) {
  LoRa_Packet msg;

  int idx = 0;
  // Append to struct data array
  msg.id = id;
  memcpy(&msg.data[idx], latitude, 15); //!< @todo Move magic number to definition/parameter
  memcpy(&msg.data[idx += 15], longitude, 15);
  msg.data[idx += 15] = flags;

  return msg;
}

LoRa_Packet LoRa_PayloadData(
    uint8_t id,
		uint8_t state,
		uint8_t *accelData,
		uint8_t lenAccelData
) {
  LoRa_Packet msg;

  int idx = 0;
  // Append to struct data array
  msg.id = id;
	msg.data[idx++] = state;
  memcpy(&msg.data[idx+lenAccelData], accelData, lenAccelData);

  return msg;
}


LoRa_Packet LoRa_GSEData_1(
		uint8_t id,
		ADC124S021 *trans,
		MCP96RL00_EMX_1 *thermo1,
		MCP96RL00_EMX_1 *thermo2,
		MCP96RL00_EMX_1 *thermo3,
		MCP96RL00_EMX_1 *thermo4,
		uint16_t error_code)
{
		LoRa_Packet msg;

		union{
			float *f;
			uint8_t b[4];
		}TR1;
		TR1.f = &trans->Converted_Value_Transducer[0];


		union{
			float *f;
			uint8_t b[4];
		}TR2;
		TR2.f = &trans->Converted_Value_Transducer[1];

		union{
			float *f;
			uint8_t b[4];
		}TR3;
		TR3.f = &trans->Converted_Value_Transducer[2];

		union{
			float *f;
			uint8_t b[4];
		}TC1;
		TC1.f = &thermo1->temperature;

		union{
				float *f;
				uint8_t b[4];
			}TC2;
			TC2.f = &thermo2->temperature;

			union{
				float *f;
				uint8_t b[4];
			}TC3;
			TC3.f = &thermo3->temperature;

			union{
				float *f;
				uint8_t b[4];
			}TC4;
			TC4.f = &thermo4->temperature;

		uint8_t idx = 0;
		msg.id = id;
		memcpy(&msg.data[idx +=sizeof(float)], TR1.b, sizeof(float)); //sizeof(b[4]) == sizeof(float)
		memcpy(&msg.data[idx +=sizeof(float)], TR2.b, sizeof(float));
		memcpy(&msg.data[idx +=sizeof(float)], TR3.b, sizeof(float));
		memcpy(&msg.data[idx +=sizeof(float)], TC1.b, sizeof(float));
		memcpy(&msg.data[idx +=sizeof(float)], TC2.b, sizeof(float));
		memcpy(&msg.data[idx +=sizeof(float)], TC3.b, sizeof(float));
		memcpy(&msg.data[idx +=sizeof(float)], TC4.b, sizeof(float));
		memcpy(&msg.data[idx +=sizeof(uint16_t)], error_code, sizeof(uint16_t));

		return msg;
}



LoRa_Packet LoRa_GSEData_2	(
		uint8_t id,
		ADC124S021 *loadcell,
		ADT75ARMZ *temp_sense,
		uint16_t error_code
		)
{
	LoRa_Packet msg;

	uint8_t idx = 0;

	union{
		float *f;
		uint8_t b[4];
	}LC1;
	LC1.f = &loadcell->Converted_Value_LoadCell;

	union{
		float *f;
		uint8_t b[4];
	}LC2;
	LC2.f = &loadcell->Converted_Value_LoadCell;

	union{
		float *f;
		uint8_t b[4];
	}TS;
	TS.f = &temp_sense->temperature;


		msg.id = id;
		memcpy(msg.data[idx += sizeof(float)], LC1.b, sizeof(float));
		memcpy(msg.data[idx += sizeof(float)], LC2.b, sizeof(float));
		memcpy(msg.data[idx += sizeof(float)], TS.b, sizeof(float));
		memcpy(msg.data[idx += sizeof(uint16_t)], &error_code, sizeof(uint16_t));

		return msg;

}

//rx not tx
LoRa_Packet LoRa_Command (uint8_t payload[LORA_MSG_LENGTH])
{
	LoRa_Packet msg;
	msg.id = payload[0]; //ID
	msg.data[0] = payload[1];
	msg.data[1] = payload[2];
	//msg.data[2] = payload[3]; -> future expansion
	return msg;
}
/********************************** DEVICE METHODS *********************************/

/* =============================================================================== */
/**
 * @brief Transmits data using the LoRa module.
 *
 * @param lora         Pointer to LoRa struct.
 * @param pointerdata  Pointer to the data to be transmitted.
 **
 * =============================================================================== */
void LoRa_transmit(LoRa *lora, uint8_t *pointerdata) {
  LoRa_writeRegister(lora, LORA_REG_IRQ_FLAGS, 0x08);     // clears the status flags
  LoRa_writeRegister(lora, LORA_REG_FIFO_ADDR_PTR, 0x80); // set pointer adddress to TX

  // Load data into transmit FIFO
  for (int i = 0; i < 32; i++) {
    LoRa_writeRegister(lora, LORA_REG_FIFO, pointerdata[i]);
  }

  // Set device to transmit
  _LoRa_setMode(lora, TX);
  // Clear the status flags
  LoRa_writeRegister(lora, LORA_REG_IRQ_FLAGS, 0x08);
}

void LoRa_receive(LoRa *lora, uint8_t *pointerdata)
{
	  LoRa_writeRegister(lora, LORA_REG_IRQ_FLAGS, 0x08);     // clears the status flags
	  LoRa_writeRegister(lora, LORA_REG_FIFO_ADDR_PTR, 0x00); // set pointer address to RX
	  // -> addr pointer check
	  _LoRa_setMode(lora, RXCONTINUOUS); //might need to modify init

	  if(CheckRX(lora) == 0)
	  {
		  //state = LoRa can not receive -> introduce new variable!
	  }
	  else{
		  for(uint8_t i = 0; i<32; i++){
			  pointerdata[i] = LoRa_readRegister(lora, LORA_REG_FIFO);
		  }
		  	  LoRa_writeRegister(lora, LORA_REG_IRQ_FLAGS, 0x08);     // clears the status flags
	  }
	  LoRa_writeRegister(lora, LORA_REG_IRQ_FLAGS, (0x01<<6));
}


uint8_t CheckRX(LoRa *lora)
{
	uint8_t irqFlag;

	irqFlag = LoRa_readRegister(&lora,LORA_REG_IRQ_FLAGS);
		if((irqFlag & RxDone) == RxDone)
		{
			LoRa_writeRegister(&lora, LORA_REG_IRQ_FLAGS, RxDone);
			return 0x01;
		}
	return 0x00;
}




/******************************** INTERFACE METHODS ********************************/

void LoRa_writeRegister(LoRa *lora, uint8_t address, uint8_t data) {
  SPI spi          = lora->base;

  uint16_t payload = (address << 0x08) | (1 << 0x0F); // Load payload with address and write command
  payload |= data;                                    // Append data to payload
  spi.port->ODR &= ~spi.cs;                           // Lower chip select
  spi.transmit(&spi, payload);                        // Send payload over SPI
  spi.port->ODR |= spi.cs;                            // Raise chip select
}

uint8_t LoRa_readRegister(LoRa *lora, uint8_t address) {
  SPI spi = lora->base;
  uint16_t response;

  uint16_t payload = (address << 0x08);   // Load payload with address and read command
  spi.port->ODR &= ~spi.cs;               // Lower chip select
  response = spi.transmit(&spi, payload); // Receive payload over SPI
  spi.port->ODR |= spi.cs;                // Raise chip select
  return (uint8_t)response;
}
