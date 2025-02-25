/*
 * uart.c
 *
 *  Created on: Feb 25, 2025
 *      Author: lucas
 */


#include "uart.h"


void USART_init(USART *usart, char[MAX_DEVICE_NAME], USART_TypeDef * interface, GPIO_TypeDef *port, UART_Pins pins, uint32_t baud, OversampleMode sample)
{
	usart->interface = interface;
	usart->port = port;
	usart->baud = baud;
	usart->pins = pins;
	usart->over8 = sample;

	usart->print = USART_print;
	usart->receive = USART_receive;
	usart->send = USART_send;
	usart->sendBytes = USART_sendBytes;
	usart->setBaud = USART_setBaud;


}

