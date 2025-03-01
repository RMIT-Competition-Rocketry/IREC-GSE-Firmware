/*
 * driver.h
 *
 *  Created on: Feb 11, 2025
 *      Author: lucas
 */

#ifndef SRC_DRIVER_H_
#define SRC_DRIVER_H_

void configureI2CBus1(void);
void configureSPIBus1(void);
//void configureSPIBus2(void);
void configureSPIBus6(void); //for both additional 5V channels and LoRa
void configureSPIBus4(void); //for flash memory storage
void configureRCC_APB1(void);
void configureRCC_APB2(void);
void configureRCC_AHB1(void);
void configure_TIM1(void);
void configure_USART6(void);


//configure USART and timers once that is done
//AHB2:3 do not need to be configured as those functions are not needed




#endif /* SRC_DRIVER_H_ */
