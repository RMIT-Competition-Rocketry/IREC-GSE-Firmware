/*
 * sensors.h
 *
 *  Created on: Feb 12, 2025
 *      Author: lucas
 */

#ifndef SRC_SENSORS_H_
#define SRC_SENSORS_H_

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "I2C.h"

#define INPUT_CHANNEL_1 0
#define INPUT_CHANNEL_2 1
#define INPUT_CHANNEL_3 2
#define INPUT_CHANNEL_4 3
#define NUM_MUX 4


//*******************MCP960X**************
#define THERMO_SAMPLE_1 0
#define THERMO_SAMPLE_2 1
#define THERMO_SAMPLE_4 2
#define THERMO_SAMPLE_8 3
#define THERMO_SAMPLE_16 4
#define THERMO_SAMPLE_32 5
#define THERMO_SAMPLE_64 6
#define THERMO_SAMPLE_128 7

#define THERMO_MODE_NORMAL 0
#define THERMO_MODE_SHUTDOWN 1
#define THERMO_MODE_BURST 2

#define COLD_JUNCTION_RES_HIGH 0
#define COLD_JUNCTION_RES_LOW 1

#define RESOLUTION_VERY_HIGH 0
#define RESOLUTION_HIGH 1
#define RESOLUTION_MEDIUM 2
#define RESOLUTION_LOW 3

//MCP96RL00_EMX_1 Registers
#define HOT_JUNCTION_TEMP_REG 0X00
#define JUNCTION_TEMP_DELTA_REG 0x01
#define COLD_JUNCTION_TEMP_REG 0x02
#define RAW_DATA_ADC_REG 0x03
#define STATUS_REG 0x04
#define THERMOCOUPLE_SENSOR_CONFIG_REG 0x05
#define DEVICE_CONFIGURATION_REG 0x06
#define ALERT_1_REG 0x08
#define ALERT_2_REG 0x09
#define ALERT_3_REG 0x0A
#define ALERT_4_REG 0x0B
#define ALERT_1_HYS_REG 0x0C
#define ALERT_2_HYS_REG 0x0D
#define ALERT_3_HYS_REG 0x0E
#define ALERT_4_HYS_REG 0x0F
#define DEVICE_ID_REG 0b00100000




//ADDR2 is not needed

typedef enum{
	Load_Cell, //0
	Transducer //1
}SensorType; // will be useful for process function
/*
addresses for sensors here:
	Thermocouple1: 0b11000000
	Thermocouple2: 0b11000001
	Thermocouple3: 0b11000010
	Thermocouple4: 0b11000011
	Temperature Sensor: 0x48

*/
typedef enum{
	Very_High, //18-bit
	High, //16-bit
	Medium,//14-bit
	Low //12-bit
}MCP96RL00_RES;


//SPI ADC converters
typedef struct{
void(*extract)(struct ADC124S021 *); //raw
SPI base;
SensorType device;
uint8_t channel;
uint16_t data_raw[NUM_MUX];
uint16_t data_processed[NUM_MUX];
float Converted_Value_LoadCell[NUM_MUX];
float Converted_Value_Transducer[NUM_MUX];
void(*process)(struct ADC124S021 *);
}ADC124S021;

void ADC124S021_init(ADC124S021 *, SensorType, GPIO_TypeDef *, unsigned long);
void ADC124S021_extract(ADC124S021 *);
void ADC124S021_process(ADC124S021 *); //LPF and convert into deg

//I2C Thermocouple ADC transducer
typedef struct{
	I2C base;
	Type device;
	uint8_t ID;
	uint8_t resolution;
	uint8_t sample;
	uint8_t address;
	uint8_t data_raw[2];//temporary size
	uint8_t data_processed[2];
	uint16_t converted_bin[2];
	float temperature;
	void(*extract)(struct MCP96RL00_EMX_1 *, uint8_t, volatile uint8_t *);
	void(*process)(struct MCP96RL00_EMX_1 *); //all data should be stored in the struct
	void(*write)(struct MCP96RL00_EMX_1 *, uint8_t, uint8_t, uint8_t);
}MCP96RL00_EMX_1;

void MCP96RL00_EMX_1_extract(MCP96RL00_EMX_1 *, uint8_t, volatile uint8_t *);
void MCP96RL00_EMX_1_process(MCP96RL00_EMX_1 *); //all data should be stored in the struct
void MCP96RL00_EMX_1_write(MCP96RL00_EMX_1 *, uint8_t, uint8_t, uint8_t);
void MCP96RL00_EMX_1_init(MCP96RL00_EMX_1 *,I2C_TypeDef *,GPIO_TypeDef *,Type, uint8_t, uint8_t, uint8_t);


#define COMPARATOR_MODE_BIT 0x00 <<1
#define INTERRUPT_MODE_BIT 0x01 << 1

#define OS_ALERT_LOW 0x00 <<2
#define OS_ALERT_HIGH 0x01 <<2

#define TEMPERATURE_QUEUE_1 0x00 <<3
#define TEMPERATURE_QUEUE_2 0x01 <<3
#define TEMPERATURE_QUEUE_4 0x02 <<3
#define TEMPERATURE_QUEUE_6 0x03 <<3

#define NORMAL_MODE 0x00 <<5
#define ONE_SHOT_MODE 0x01 <<5

#define DIS_SMBUS_ALERT 0x00 <<7
#define EN_SMBUS_ALERT 0x01 <<7


//temperature sensor
typedef struct{
	I2C base;
	Type device;
	uint8_t address;
	volatile int8_t data_raw[2];
	uint8_t data_processed[2];
	float temperature;
	void(*extract)(struct ADT75ARMZ *, volatile uint8_t *, uint8_t);
	void(*process)(struct ADT75ARMZ *);
	void(*write)(struct ADT75ARMZ *, uint8_t, uint8_t, uint8_t); //config register is only 8 bits || data, device address, pointer address
}ADT75ARMZ;

void ADT75ARMZ_init(ADT75ARMZ *, I2C_TypeDef *, GPIO_TypeDef *, Type, uint8_t); //last one is device address
void ADT75ARMZ_extract(ADT75ARMZ *, volatile uint8_t *, uint8_t);
void ADT75ARMZ_process(ADT75ARMZ *);
void ADT75ARMZ_write(ADT75ARMZ *, uint8_t, uint8_t, uint8_t);

void delay_software_us( uint32_t);
void delay_software_ms( uint32_t);
#endif /* SRC_SENSORS_H_ */
