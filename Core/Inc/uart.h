/*
 * uart.h
 *
 *  Created on: Feb 25, 2025
 *      Author: lucas
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#define MAX_DEVICE_NAME 0x20
#include "stm32f4xx_hal.h"


typedef enum {
  OVER8,
  OVER16
} OversampleMode;

typedef struct UART_Pins {
	uint8_t TX;
	uint8_t RX;
} UART_Pins;

typedef struct USART{
	USART_TypeDef *interface;
	GPIO_TypeDef *port;
	UART_Pins pins;
	uint32_t baud;
	OversampleMode over8;
	void(*setBaud)(struct USART *, uint32_t);
	void(*send)(struct USART *, uint8_t);
	void(*sendBytes)(struct USART *, uint8_t *, int);
	void(*print)(struct USART *, char *);
	uint8_t(*receive)(struct USART *);
}USART;

void USART_init(USART *, char[MAX_DEVICE_NAME], USART_TypeDef *, GPIO_TypeDef *, UART_Pins, uint32_t, OversampleMode);
void USART_setBaud(USART *, uint32_t);
void USART_send(USART *, uint8_t);
void USART_sendBytes(USART *, uint8_t *, int);
void USART_print(USART *, char *);
uint8_t USART_receive(USART *);


#endif /* INC_UART_H_ */
