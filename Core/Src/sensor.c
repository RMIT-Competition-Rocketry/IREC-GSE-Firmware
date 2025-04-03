/*
 * sensor.c
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */


#include "sensors.h"
#include "spi.h"
#include "stm32f4xx_hal.h"

void ADC124S021_init(ADC124S021 *adc, SensorType SensorType, GPIO_TypeDef *port, unsigned long cs)
{
	SPI_init(&adc->base , SensorType, SPI1, MODE16, port,cs);
	adc->base;
	adc->device = SensorType;
	adc->extract = ADC124S021_extract;
	adc->process = ADC124S021_process;
}

void ADC124S021_extract(ADC124S021 * adc)
{
switch(adc->device)
{
uint16_t word = 0;
case 0:
	//loadcell

	//read all 4 channels from the ADC
	for(uint8_t channel = 0; channel <NUM_MUX; channel ++)
	{
		word = (channel << 14) | (0x00 << 12);; //correct! (8 bit type casting!)
		GPIOG->ODR &= ~(GPIO_ODR_OD4);
		delay_software_us(1);
		uint16_t result = SPI_transmit(&adc->base, word);
		GPIOG->ODR |= GPIO_ODR_OD4;
		delay_software_us(1);
		adc->data_raw[channel] = result & 0xFFF;
	}

case 1://transducer
		//read all 4 channels from the ADC
	for(uint8_t channel = 0; channel <NUM_MUX; channel ++)
		{
			word = (channel << 14) | (0x00 << 12); //correct!
			//control register -> MSB doesnt matter, channel does!
			GPIOG->ODR &= ~(GPIO_ODR_OD4);
			delay_software_us(1);
			uint16_t result = SPI_transmit(&adc->base, word);
			GPIOG->ODR |= GPIO_ODR_OD4;
			delay_software_us(1);
			adc->data_raw[channel] = result & 0x0FFF; //masks for first 12 bits
		}
			//[0] = IN1 -> LOADCELL1
			//[1] = IN2 -> LOADCELL2
			//[2] = IN3 -> LOADCELL3
			//[3] = IN4 -> LOADCELL4
	break;
	}
}

void ADC124S021_process(ADC124S021 * adc)
{
	//throw this into a LPF
	for(int i = 0; i<4; i++){
		adc->data_processed[i] = 0.15f*adc->data_processed[i] + (1-0.15f)*adc->data_raw[i];
		//data_processed[0] = input 1
		//data_processed[1] = input 2
		//data_processed[2] = input 3
		//data_processed[3] = input 4
		//conversion for each here ->
		switch(adc->device)
		{
			case 0: //loadcell
				typedef struct{float mass; uint16_t post_op; int mass_int;}Converter_load;
				Converter_load load = {};
				for(int i = 0; i<NUM_MUX; i++)
				{
					load.post_op  = adc->data_processed[i];
					load.mass_int = load.post_op;
					adc->Converted_Value_LoadCell[i] = (load.mass_int/(50.0f/4096.0f));
					//output for each channel will be in KG
				}
				break;

			case 1: //transducer
				typedef struct{float mass; uint16_t post_op; int mass_int;}Converter_trans;
				Converter_trans trans = {};
				for(int i = 0; i<NUM_MUX; i++)
								{
									trans.post_op  = adc->data_processed[i];
									trans.mass_int = trans.post_op;
									adc->Converted_Value_Transducer[i] = (trans.mass_int/(300.0f/4096.0f));
									//output for each channel will be in bar of pressure
								}
				break;
		}

	}
}

void MCP96RL00_EMX_1_init(MCP96RL00_EMX_1 *sensor, I2C_TypeDef *interface,GPIO_TypeDef *port, Type THERMOCOUPLE, uint8_t sample, uint8_t resolution, uint8_t address)
{
	I2C_init(&sensor->base, interface, port, THERMOCOUPLE, address, 0);//change address later
	sensor->address = address;
	sensor->device = THERMOCOUPLE;
	sensor->resolution = resolution;
	sensor->sample = sample;
	sensor->extract = MCP96RL00_EMX_1_extract;
	sensor->process = MCP96RL00_EMX_1_process;
//	sensor->write = MCP96RL00_EMX_1_write;

	//parse data into temporary variable -> then do this with all thermocouples
	//read = bit0 = 1
	//write = bit0 = 1
	//address = external GPIO -> should not be set in software -> will change later

	//get the device ID first
	uint8_t data_thermo_conf = 0x00 << 4; 	//configure the thermocouple type (CHANGE THIS) does nothing
	uint8_t data_status_conf = COLD_JUNCTION_RES_LOW << 7 | RESOLUTION_HIGH << 5 | THERMO_SAMPLE_8 << 2 | 0x00; //sample is only in burst mode -> not implemented
	//temporarily T type thermocouple!
	MCP96RL00_EMX_1_write(&sensor->base, address, data_thermo_conf, THERMOCOUPLE_SENSOR_CONFIG_REG);
	MCP96RL00_EMX_1_write(&sensor->base, address, data_status_conf, DEVICE_CONFIGURATION_REG);
	//device config register
//*************I2C Addresses for each device*****************************
	/*Thermocouple 1: 0b1100000
	 *Thermocouple 2: 0b1100001
	 *Thermocouple 3: 0b1100010
	 *Thermocouple 4: 0b1100011
	 */


}


void MCP96RL00_EMX_1_extract(MCP96RL00_EMX_1 *mcp, uint8_t address, volatile uint8_t *data)
{
	//command is fixed -> no need to change this
	I2C_send(&mcp->base, address, HOT_JUNCTION_TEMP_REG);
	I2C_MultiReceive(&mcp->base, *data, address, 2);

	mcp->data_raw[0] = data[0]; //MSB
	mcp->data_raw[1] = data[1]; //LSB
	//each sensor will have its own struct like this

}
void MCP96RL00_EMX_1_process(MCP96RL00_EMX_1 *mcp)
{
	for(uint8_t i = 0; i<2; i++){
	//a = 0.15
		mcp->data_processed[i] = 0.15*(mcp->data_processed[i]) + (1-0.15)*mcp->data_raw[i];

	//y[n] = a*y[n-1] + (1-a)x[n];
	}
	//do temperature conversion
	//K type thermocouple -> temperature range is-> -200C to 1372C within a 0-5V range
	mcp->converted_bin[0] = (mcp->data_processed[0] << 8) | (mcp->data_processed[1]);

	union{float temperature; uint16_t raw_bin[2]; int temperature_int;}Converter;
		for(uint8_t i = 0; i<2; i++)
		{
			Converter.raw_bin[i] = mcp->converted_bin[i];
		}
		if(Converter.temperature_int & 0x01<<31)//checking for sign
		{
			Converter.temperature = (((Converter.temperature_int & 0xFF000000)*16) + ((Converter.temperature_int & 0x00FF0000)/16) - 4096);
			mcp->temperature = Converter.temperature;
		}
		else{ //if sign is 0 -> temperature is positive below!
			Converter.temperature = ((Converter.temperature_int & 0xFF000000)*16) + ((Converter.temperature_int & 0x00FF0000)/16);
			mcp->temperature = Converter.temperature;
		}
	//too access temperature information post processing-> use struct element->temperature
}
void MCP96RL00_EMX_1_write(I2C *mcp, uint8_t address, uint8_t data, uint8_t command)
{
	uint8_t *payload_first[2] ={0,0};
	uint8_t *payload[3] = {0,0,0}; //address, MSB byte, LSB byte
	payload_first[0] = &command; //pointer register
	payload_first[1] = &data;
	I2C_send(&mcp, &payload_first[0], payload_first[1]); //Struct, Address byte, Pointer address,
	I2C_sendBurst(&mcp, &payload, 3-1, address);
}


void ADT75ARMZ_init(ADT75ARMZ *i2c, I2C_TypeDef *interface, GPIO_TypeDef *port, Type TEMPERATURE_SENSOR, uint8_t address)
{
	I2C_init(&i2c->base, interface, port, TEMPERATURE_SENSOR, address, 0); //error is 0 as to presume no errors initially
	i2c->address = address;
	i2c->device = TEMPERATURE_SENSOR;
	i2c->extract = ADT75ARMZ_extract;
	i2c->process = ADT75ARMZ_process;
	i2c->write = ADT75ARMZ_write;
	//reading (R) = 0 at LSb, writing (W) = 1 at LSB
	address = (address<<1)|0x01; //bit shift left and bitwise or the result post bitshift
	uint8_t data[2];
	data[0] = 0x01; //configuration register pointer
	data[1] = (INTERRUPT_MODE_BIT | OS_ALERT_HIGH | TEMPERATURE_QUEUE_1 | NORMAL_MODE | DIS_SMBUS_ALERT);
	I2C_sendBurst(&i2c, data, 2, address); //2 bytes

}
//@param data is a temporary variable -> doesn't have to be declared globally!
void ADT75ARMZ_extract(ADT75ARMZ *i2c, volatile uint8_t *data, uint8_t address)	//data is read in 2 byte chunks! (every transaction must include pointer register)
{
	address = address<<1; //6bit address fitting the entire byte to include the r/w command

	 I2C_TempExtract(&i2c, *data, address, 0x00);
	 i2c->data_raw[0] = data[0];
	 i2c->data_raw[1] = data[1];
}

void ADT75ARMZ_process(ADT75ARMZ *i2c)
{
	for(int i = 0; i<2;i++){

	i2c->data_processed[i] = 0.10*i2c->data_processed[i] + (1-0.1)*i2c->data_raw[i];
		//y[n] = a*y[n-1] + (1-a)x[n];
			//a = 0.1
	}
		union{float temperature;  int data_int;}Converter;
			Converter.data_int = i2c->data_processed[0]<<24 | i2c->data_processed[1]<<16;
		if(Converter.data_int & 0x01<<31){//sign checking
			Converter.temperature = (((Converter.data_int & 0xFFF00000)-4096)/16);
			//for negative temperatures
		}
		else{
			Converter.temperature = ((Converter.data_int & 0xFFF00000)/16);
			//for positive temperatures
		}
}
void ADT75ARMZ_write(ADT75ARMZ *i2c, uint8_t data, uint8_t address, uint8_t pointer_address)
{
	address = (address<<1)|0x01; //bit shift left and bitwise or the result post bitshift
	uint8_t payload[2];

		payload[0] = pointer_address;
		payload[1] = data;

	I2C_sendBurst(&i2c, data, sizeof(data), address);
}

uint16_t ADC124S021_ReadChannel(uint8_t channel) {
    uint16_t adcValue = 0;
    uint16_t command = (channel << 14); // Channel select (bits D15-D14)

	GPIOG->ODR &= ~GPIO_ODR_OD4; //bring dow CS of PG

    // Transmit command (16-bit)
    while (!(SPI1->SR & SPI_SR_TXE)); // Wait for TX buffer empty
    SPI1->DR = command;

    // Wait for RX data
    while (!(SPI1->SR & SPI_SR_RXNE));
    adcValue = SPI1->DR; // Dummy read (first 16 bits are zeros)

    // Read actual ADC value
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = 0x0000; // Send dummy data to clock out ADC result

    while (!(SPI1->SR & SPI_SR_RXNE));
    adcValue = SPI1->DR & 0x0FFF; // Mask 12-bit result

	GPIOG->ODR |= GPIO_ODR_OD4; //raise up CS of PG
    return adcValue;
}

void delay_software_us( uint32_t usec )
{
    // To avoid delaying for less than usec, always round up.;
		uint32_t i = 0;
    for(i = 0; i < (usec * 21); i++);

}

void delay_software_ms( uint32_t msec )
{
    // To avoid delaying for less than usec, always round up.;
		uint32_t i = 0;
    for(i = 0; i < (msec * 21000); i++);

}
