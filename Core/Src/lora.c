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


void SX1272_init(
    SX1272_t *lora,
    char name[DEVICE_NAME_LENGTH],
    GPIO_TypeDef *port,
    unsigned long cs,
    SX1272_Bandwidth bw,
    SX1272_SpreadingFactor sf,
    SX1272_CodingRate cr
) {
  SPI_init(&lora->base, COMM_LORA, SPI6, MODE8, port, cs);
  lora->standby      = SX1272_standby;
  lora->enableBoost  = SX1272_enableBoost;
  lora->transmit     = SX1272_transmit;
  lora->startReceive = SX1272_startReceive;
  lora->readReceive  = SX1272_readReceive;
  lora->clearIRQ     = SX1272_clearIRQ;


  _SX1272_setMode(lora, SX1272_MODE_SLEEP); // Set mode to sleep


  /*Set carrier frequency to 915.6MHz*/
  //MSB remains the same as default value of E4
 // SX1272_writeRegister(lora, SX1272_REG_FR_MIB, (0xE6)); //sets middle byte
  //SX1272_writeRegister(lora, SX1272_REG_FR_LSB, (0x67)); //sets last byte

  /* clang-format off */
  SX1272_writeRegister(lora, SX1272_REG_OP_MODE,
     0x01 << SX1272_OP_MODE_LONG_RANGE_Pos  // Enable LoRa
  );

  SX1272_writeRegister(lora, SX1272_REG_MODEM_CONFIG1,
    bw   << SX1272_REG_MODEM_CONFIG1_BW_Pos     // Set bandwidth
  | cr   << SX1272_REG_MODEM_CONFIG1_CR_Pos     // Set coding rate
  | 0x00 << SX1272_REG_MODEM_CONFIG1_CRC_Pos    // Enable CRC
  );
  /* clang-format on */

  /** @todo set spreading factor */
  SX1272_writeRegister(lora, SX1272_REG_MODEM_CONFIG2, 0x94);

  //Set both Tx and Rx FIFO base addresses to 0x00
  SX1272_writeRegister(lora, SX1272_REG_FIFO_TX_BASE_ADDR, 0x00); // Tx starts at 0x00
  SX1272_writeRegister(lora, SX1272_REG_FIFO_RX_BASE_ADDR, 0x00); // Rx starts at 0x00


  // Set payload length
  SX1272_writeRegister(lora, SX1272_REG_PAYLOAD_LENGTH, LORA_MSG_LENGTH);
  SX1272_writeRegister(lora, SX1272_REG_MAX_PAYLOAD_LENGTH, LORA_MSG_LENGTH);

  _SX1272_setMode(lora, SX1272_MODE_STDBY); // Set mode to Standby mode!


}

//Config LoRa carrier frequency - 915.6MHz


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
void _LoRa_setMode(SX1272_t *lora, SX1272_Mode mode) {
  uint8_t regOpMode = LoRa_readRegister(lora, SX1272_REG_OP_MODE);
  regOpMode &= ~0x07; // Mask to mode bits
  regOpMode |= mode;  // Set mode
  LoRa_writeRegister(lora, SX1272_REG_OP_MODE, regOpMode);
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

LoRa_Packet Dummy_Transmit()
{
	LoRa_Packet msg;
	uint8_t id = 0x06;
	uint8_t SIZE = 32;
	uint8_t data[SIZE];
	msg.id = id;
	msg.data[0]=0x06;
	for(uint8_t i = 1; i <SIZE; i++)
	{
		msg.data[i] = i;
	}
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
void _SX1272_setMode(SX1272_t *lora, SX1272_Mode mode) {
  uint8_t regOpMode = SX1272_readRegister(lora, SX1272_REG_OP_MODE);
  regOpMode &= ~0x07; // Mask to mode bits
  regOpMode |= mode;  // Set mode
  SX1272_writeRegister(lora, SX1272_REG_OP_MODE, regOpMode);
}


/***************************************** PUBLIC METHODS *****************************************/

/* ============================================================================================== */
/**
 * @brief  Enables/disables power amplifier boost
 *
 * @param  *lora  Pointer to LoRa struct.
 * @param  enable Boolean value for the enable toggle.
 *
 * @return @c NULL.
 **
 * ============================================================================================== */
void SX1272_enableBoost(SX1272_t *lora, bool enable) {
  uint8_t regPaConfig  = SX1272_readRegister(lora, SX1272_REG_PA_CONFIG); // Read current config
  regPaConfig         |= SX1272_PA_SELECT;								//Set PA Boost Select
  regPaConfig 		  |= 0x0F;											//Set Gain to MAX
  SX1272_writeRegister(lora, SX1272_REG_PA_CONFIG, regPaConfig);
  regPaConfig  = SX1272_readRegister(lora, SX1272_REG_PA_CONFIG);		 //DEBUGGING -  Read back in current config and ensure its as expected
  uint8_t regPaDac = SX1272_readRegister(lora, 0x5A);
  regPaDac |= 0x07;														//Set PaDac to +20dbm mode
  SX1272_writeRegister(lora, 0x5A, regPaDac);
  uint8_t regOpC = SX1272_readRegister(lora, 0x0B);
  regOpC |= 0b00100000;														//Disable Overcurrent protection
  SX1272_writeRegister(lora, 0x0B, regOpC);
  regPaDac = SX1272_readRegister(lora, 0x0B);							//DEBUGGING - Read back in current config and ensure its as expected
}

/* ============================================================================================== */
/**
 * @brief  Sets the operational mode of the LoRa module to standby.
 *
 * @param  *lora Pointer to LoRa struct.
 *
 * @return @c NULL.
 **
 * ============================================================================================== */
void SX1272_standby(SX1272_t *lora) {
  _SX1272_setMode(lora, SX1272_MODE_STDBY);
}

/* ============================================================================================== */
/**
 * @brief Transmits data using the SX1272.
 *
 * @param lora         Pointer to SX1272 struct.
 * @param pointerdata  Pointer to the data to be transmitted.
 **
 * ============================================================================================== */
void SX1272_transmit(SX1272_t *lora, uint8_t *pointerdata) {
  // Set device to standby
  _SX1272_setMode(lora, SX1272_MODE_STDBY);
  SX1272_enableBoost(lora, true);

  // TODO: add in proper read-mask-write operation for setting DIO mapping
  //
  // Set DIO interrupt pin to TxDone
  SX1272_writeRegister(lora, SX1272_REG_DIO_MAPPING1, SX1272_LORA_DIO_TXDONE);

  // Since the device will only ever be transmitting or receiving at any given time
  // and each packet should be handled immediately by the implementation (no waiting
  // on buffering), we don't need to be concerned about the buffer being overwritten.
  //
  // ...for now.

  // TODO:
  // Think of a more elegant solution for applications that might use this
  // driver that want buffered data
  //
  // Clear IRQ flags and set FIFO address pointer.
  SX1272_writeRegister(lora, SX1272_REG_IRQ_FLAGS, SX1272_LORA_IRQ_TXDONE); // clears the IRQ flag
  SX1272_writeRegister(lora, SX1272_REG_FIFO_ADDR_PTR, 0x00);               // set pointer adddress to start

  // Load data into transmit FIFO
  for (int i = 0; i < 32; i++) {
    SX1272_writeRegister(lora, SX1272_REG_FIFO, pointerdata[i]);
  }

  // Update the current operating mode
  lora->currentMode = SX1272_MODE_TX;       // Set local mode setting
  _SX1272_setMode(lora, lora->currentMode); // Start transmitting
}

/* ============================================================================================== */
/**
 * @brief Begins continuous receive on the SX1272.
 *
 * @param lora Pointer to SX1272 struct.
 **
 * ============================================================================================== */
void SX1272_startReceive(SX1272_t *lora) {
  // Set device to standby
  _SX1272_setMode(lora, SX1272_MODE_STDBY);

  // TODO: add in proper read-mask-write operation for setting DIO mapping
  //
  // Set DIO interrupt pin to RxDone
  SX1272_writeRegister(lora, SX1272_REG_DIO_MAPPING1, SX1272_LORA_DIO_RXDONE);

  // Since the device will only ever be transmitting or receiving at any given time
  // and each packet should be handled immediately by the implementation (no waiting
  // on buffering), we don't need to be concerned about the buffer being overwritten.
  //
  // ...for now.

  // TODO:
  // Think of a more elegant solution for applications that might use this
  // driver that want buffered data
  //
  // Clear IRQ flags and set FIFO address pointer.
  SX1272_writeRegister(lora, SX1272_REG_IRQ_FLAGS, SX1272_LORA_IRQ_RXDONE); // Clear the IRQ flag
  SX1272_writeRegister(lora, SX1272_REG_FIFO_ADDR_PTR, 0x00);               // Set pointer adddress to start

  // Update the current operating mode
  lora->currentMode = SX1272_MODE_RXCONTINUOUS; // Set local mode setting
  _SX1272_setMode(lora, lora->currentMode);     // Start receiving
}

/* ============================================================================================== */
/**
 * @brief  Reads contents of received packet to local buffer from the SX1272.
 *
 * @param  lora     Pointer to SX1272 struct.
 * @param  buffer   Pointer to the buffer to store received data.
 * @param  buffSize Integer representing the size of the buffer to to fill.
 *
 * @return Boolean value indicating if a packet was successfully received and
 *         returned in buffer.
 **
 * ============================================================================================== */
bool SX1272_readReceive(SX1272_t *lora, uint8_t *buffer, uint8_t buffSize) {

  // TODO: Error handling for IRQ flags
  //
  // Currently the readReceive() method clears the SX1272 RxDone IRQ flag before
  // starting the read. This is fine for cases where the user code carefully
  // manages the DIO interrupts, however ideally the method should check for
  // errors in the IRQ register and appropriately discard received packets.

  // Clear the IRQ flag
  SX1272_writeRegister(lora, SX1272_REG_IRQ_FLAGS, SX1272_LORA_IRQ_RXDONE);

  // Read address and packet width information of received data
  uint8_t bytesReceived = SX1272_readRegister(lora, SX1272_REG_RX_BYTES);          // Number of bytes received
  uint8_t rxCurrentAddr = SX1272_readRegister(lora, SX1272_REG_FIFO_RX_CURR_ADDR); // Address of last packet

  // Return error if buffer is smaller than the received data
  if (bytesReceived > buffSize)
    return false;

  // Otherwise, set the address pointer and read each byte into buffer
  SX1272_writeRegister(lora, SX1272_REG_FIFO_ADDR_PTR, rxCurrentAddr);
  for (int i = 0; i < bytesReceived; i++) {
    buffer[i] = SX1272_readRegister(lora, SX1272_REG_FIFO);
  }

  return true;
}

/* ============================================================================================== */
/**
 * @brief  Sets the value of RegIrqFlags in the SX1272 to the provided argument value.
 *         Writing a 1 to a bit in the register will clear the associated flag.
 *
 * @param  lora  Pointer to SX1272 struct.
 * @param  flags 8-bit value representing flag bits to be set.
 *
 * @return @c NULL
 **
 * ============================================================================================== */
void SX1272_clearIRQ(SX1272_t *lora, uint8_t flags) {
  SX1272_writeRegister(lora, SX1272_REG_IRQ_FLAGS, flags);
}

/*************************************** INTERFACE METHODS ****************************************/

void SX1272_writeRegister(SX1272_t *lora, uint8_t address, uint8_t data) {
  SPI spi = lora->base;

  //Pull CS low
  spi.port->ODR &= ~spi.cs;

  //Send write data and address
  uint8_t payload = address | 0x80; // Load payload with address and write command
  spi.transmit(&spi, payload);      // Transmit payload
  spi.transmit(&spi, data);         // Transmit write data

  // Set CS high
  spi.port->ODR |= spi.cs;
}

uint8_t SX1272_readRegister(SX1272_t *lora, uint8_t address) {
  uint8_t response = 0;
  SPI spi         = lora->base;

  // Pull CS low
  spi.port->ODR &= ~spi.cs;

  // Send write data and address
  uint8_t payload = address & 0x7F;              // Load payload with address and read command
  response        = spi.transmit(&spi, payload); // Transmit payload
  response        = spi.transmit(&spi, 0xFF);    // Transmit dummy data and reasd response

  // Set CS high
  spi.port->ODR |= spi.cs;

  return response;
}



