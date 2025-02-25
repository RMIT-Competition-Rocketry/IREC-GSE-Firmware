/*
 * gpio.c
 *
 *  Created on: Feb 11, 2025
 *      Author: lucas
 */


#include "gpio.h"
#include "stm32f4xx_hal.h"


void GPIO_init(GPIO *gpio, GPIO_TypeDef *port, uint8_t MODER, uint8_t OTYPER, uint8_t OSPEEDR, uint8_t PUPDR, uint8_t POS)
{
	gpio->port = port;
	gpio->MODER = MODER;
	gpio->OTYPER = OTYPER;
	gpio->OSPEEDR = OSPEEDR;
	gpio->PUPDR = PUPDR;
	gpio->POS = POS;

	gpio->port->MODER &= ~(MODER<<(2*POS)); //2* bc of 32 bit register
	gpio->port->MODER |= (MODER<<(2*POS));
	gpio->port->OTYPER &= ~(OTYPER<<POS);
	gpio->port->OTYPER |= (OTYPER<<POS);
	gpio->port->OSPEEDR &= ~(OSPEEDR<<(2*POS));
	gpio->port->OSPEEDR |= (OSPEEDR<<(2*POS));
	gpio->port->PUPDR &= ~(PUPDR<<(2*POS)); //clears the bit that we want to edit
	gpio->port->PUPDR |= (PUPDR<<(2*POS));

}

void GPIO_init_interrupt(GPIO *gpio, GPIO_TypeDef *port, uint8_t MODER, uint8_t OTYPER, uint8_t OSPEEDR, uint8_t PUPDR, uint8_t POS)
{
	gpio->port = port;
	gpio->MODER = MODER;
	gpio->OTYPER = OTYPER;
	gpio->OSPEEDR = OSPEEDR;
	gpio->PUPDR = PUPDR;
	gpio->POS = POS;


	gpio->port->MODER &= ~(MODER<<(2*POS)); //2* bc of 32 bit register
	gpio->port->MODER |= (MODER<<(2*POS));
	gpio->port->OTYPER &= ~(OTYPER<<POS);
	gpio->port->OTYPER |= (OTYPER<<POS);
	gpio->port->OSPEEDR &= ~(OSPEEDR<<(2*POS));
	gpio->port->OSPEEDR |= (OSPEEDR<<(2*POS));
	gpio->port->PUPDR &= ~(PUPDR<<(2*POS));
	gpio->port->PUPDR |= (PUPDR<<(2*POS));

	//so if I write 0x00:0x0F->PA[x]->PA[
	switch(POS){
		case 0:
		//this is for PA/B/C...0:3
			if(&port == (GPIO_TypeDef*)GPIOA)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PA;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
				EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
				EXTI->RTSR |= EXTI_RTSR_TR0;
				EXTI->IMR &= ~EXTI_IMR_IM0;
				EXTI->IMR |= EXTI_IMR_IM0;
			NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupt channel
			}
			else if(&port == (GPIO_TypeDef*)GPIOB)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PB;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;
				EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR0;
							EXTI->IMR &= ~EXTI_IMR_IM0;
							EXTI->IMR |= EXTI_IMR_IM0;
							NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOC)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PC;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;
				EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR0;
							EXTI->IMR &= ~EXTI_IMR_IM0;
							EXTI->IMR |= EXTI_IMR_IM0;
							NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupt channel
							//priority is given once a specific pin has been init
			}
			else if(&port == (GPIO_TypeDef*)GPIOD)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PD;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PD;
				EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR0;
							EXTI->IMR &= ~EXTI_IMR_IM0;
							EXTI->IMR |= EXTI_IMR_IM0;
							NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOE)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PE;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PE;
				EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR0;
							EXTI->IMR &= ~EXTI_IMR_IM0;
							EXTI->IMR |= EXTI_IMR_IM0;
							NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOF)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PF;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PF;
				EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR0;
							EXTI->IMR &= ~EXTI_IMR_IM0;
							EXTI->IMR |= EXTI_IMR_IM0;
							NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupt channel

			}
			else{
				//for GPIOG
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PG;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PG;
				EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR0;
							EXTI->IMR &= ~EXTI_IMR_IM0;
							EXTI->IMR |= EXTI_IMR_IM0;
							NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupt channel
			}
		break;
		case 1:
		//this is for PA/B/C...0:3
			if(&port == (GPIO_TypeDef*)GPIOA)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PA;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
				EXTI->RTSR &= ~EXTI_RTSR_TR1_Msk;
				EXTI->RTSR |= EXTI_RTSR_TR1;
				EXTI->IMR &= ~EXTI_IMR_IM1;
				EXTI->IMR |= EXTI_IMR_IM1;
				NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOB)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PB;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB;
				EXTI->FTSR &= ~EXTI_RTSR_TR1_Msk;
							EXTI->FTSR |= EXTI_RTSR_TR1;
							EXTI->IMR &= ~EXTI_IMR_IM1;
							EXTI->IMR |= EXTI_IMR_IM1;
							NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupt channel


			}
			else if(&port == (GPIO_TypeDef*)GPIOC)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PC;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;
				EXTI->RTSR &= ~EXTI_RTSR_TR1_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR1;
							EXTI->IMR &= ~EXTI_IMR_IM1;
							EXTI->IMR |= EXTI_IMR_IM1;
							NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOD)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PD;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PD;
				EXTI->RTSR &= ~EXTI_RTSR_TR1_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR1;
							EXTI->IMR &= ~EXTI_IMR_IM1;
							EXTI->IMR |= EXTI_IMR_IM1;
							NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOE)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PE;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PE;
				EXTI->RTSR &= ~EXTI_RTSR_TR1_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR1;
							EXTI->IMR &= ~EXTI_IMR_IM1;
							EXTI->IMR |= EXTI_IMR_IM1;
							NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOF)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PF;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PF;
				EXTI->RTSR &= ~EXTI_RTSR_TR1_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR1;
							EXTI->IMR &= ~EXTI_IMR_IM1;
							EXTI->IMR |= EXTI_IMR_IM1;
							NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupt channel

			}
			else{
				//for GPIOG
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1_PG;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PG;
				EXTI->RTSR &= ~EXTI_RTSR_TR1_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR1;
							EXTI->IMR &= ~EXTI_IMR_IM1;
							EXTI->IMR |= EXTI_IMR_IM1;
							NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupt channel

			}
		break;
		case 2:
		//this is for PA/B/C...0:3
			if(&port == (GPIO_TypeDef*)GPIOA)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PA;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA;
				EXTI->RTSR &= ~EXTI_RTSR_TR2_Msk;
				EXTI->RTSR |= EXTI_RTSR_TR2;
				EXTI->IMR &= ~EXTI_IMR_IM2;
				EXTI->IMR |= EXTI_IMR_IM2;
				NVIC_EnableIRQ(EXTI2_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOB)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PB;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;
				EXTI->RTSR &= ~EXTI_RTSR_TR2_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR2;
							EXTI->IMR &= ~EXTI_IMR_IM2;
							EXTI->IMR |= EXTI_IMR_IM2;
							NVIC_EnableIRQ(EXTI2_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOC)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PC;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;
				EXTI->RTSR &= ~EXTI_RTSR_TR2_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR2;
							EXTI->IMR &= ~EXTI_IMR_IM2;
							EXTI->IMR |= EXTI_IMR_IM2;
							NVIC_EnableIRQ(EXTI2_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOD)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PD;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PD;
				EXTI->RTSR &= ~EXTI_RTSR_TR2_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR2;
							EXTI->IMR &= ~EXTI_IMR_IM2;
							EXTI->IMR |= EXTI_IMR_IM2;
							NVIC_EnableIRQ(EXTI2_IRQn); //enable interrupt channel

			}
			else if(&port ==(GPIO_TypeDef*) GPIOE)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PE;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PE;
				EXTI->RTSR &= ~EXTI_RTSR_TR2_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR2;
							EXTI->IMR &= ~EXTI_IMR_IM2;
							EXTI->IMR |= EXTI_IMR_IM2;
							NVIC_EnableIRQ(EXTI2_IRQn); //enable interrupt channel

			}
			else if(&port == (GPIO_TypeDef*)GPIOF)
			{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PF;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PF;
				EXTI->RTSR &= ~EXTI_RTSR_TR2_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR2;
							EXTI->IMR &= ~EXTI_IMR_IM2;
							EXTI->IMR |= EXTI_IMR_IM2;
							NVIC_EnableIRQ(EXTI2_IRQn); //enable interrupt channel

			}
			else{
				//for GPIOG
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI2_PG;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PG;
				EXTI->RTSR &= ~EXTI_RTSR_TR2_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR2;
							EXTI->IMR &= ~EXTI_IMR_IM2;
							EXTI->IMR |= EXTI_IMR_IM2;
							NVIC_EnableIRQ(EXTI2_IRQn); //enable interrupt channel

			}
		break;
		case 3:
			//this is for PA/B/C...0:3
				if(&port == (GPIO_TypeDef*)GPIOA)
				{
				SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PA;
				SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA;
				EXTI->RTSR &= ~EXTI_RTSR_TR3_Msk;
				EXTI->RTSR |= EXTI_RTSR_TR3;
				EXTI->IMR &= ~EXTI_IMR_IM3;
				EXTI->IMR |= EXTI_IMR_IM3;
				NVIC_EnableIRQ(EXTI3_IRQn); //enable interrupt channel

				}
				else if(&port ==(GPIO_TypeDef*) GPIOB)
				{
					SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PB;
					SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
					EXTI->RTSR &= ~EXTI_RTSR_TR3_Msk;
								EXTI->RTSR |= EXTI_RTSR_TR3;
								EXTI->IMR &= ~EXTI_IMR_IM3;
								EXTI->IMR |= EXTI_IMR_IM3;
								NVIC_EnableIRQ(EXTI3_IRQn); //enable interrupt channel


				}
				else if(&port == (GPIO_TypeDef*)GPIOC)
				{
					SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PC;
					SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
					EXTI->RTSR &= ~EXTI_RTSR_TR3_Msk;
								EXTI->RTSR |= EXTI_RTSR_TR3;
								EXTI->IMR &= ~EXTI_IMR_IM3;
								EXTI->IMR |= EXTI_IMR_IM3;
								NVIC_EnableIRQ(EXTI3_IRQn); //enable interrupt channel

				}
				else if(&port ==(GPIO_TypeDef*) GPIOD)
				{
					SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PD;
					SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PD;
					EXTI->RTSR &= ~EXTI_RTSR_TR3_Msk;
								EXTI->RTSR |= EXTI_RTSR_TR3;
								EXTI->IMR &= ~EXTI_IMR_IM3;
								EXTI->IMR |= EXTI_IMR_IM3;
								NVIC_EnableIRQ(EXTI3_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOE)
				{
					SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PE;
					SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PE;
					EXTI->RTSR &= ~EXTI_RTSR_TR3_Msk;
								EXTI->RTSR |= EXTI_RTSR_TR3;
								EXTI->IMR &= ~EXTI_IMR_IM3;
								EXTI->IMR |= EXTI_IMR_IM3;
								NVIC_EnableIRQ(EXTI3_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOF)
				{
					SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PF;
					SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PF;
					EXTI->RTSR &= ~EXTI_RTSR_TR3_Msk;
								EXTI->RTSR |= EXTI_RTSR_TR3;
								EXTI->IMR &= ~EXTI_IMR_IM3;
								EXTI->IMR |= EXTI_IMR_IM3;
								NVIC_EnableIRQ(EXTI3_IRQn); //enable interrupt channel

				}
				else{
						//for GPIOG
					SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PG;
					SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PG;
					EXTI->RTSR &= ~EXTI_RTSR_TR3_Msk;
								EXTI->RTSR |= EXTI_RTSR_TR3;
								EXTI->IMR &= ~EXTI_IMR_IM3;
								EXTI->IMR |= EXTI_IMR_IM3;
								NVIC_EnableIRQ(EXTI3_IRQn); //enable interrupt channel

				}
			break;



		case 4:
			//this is for PA/B/C...0:3
				if(&port == (GPIO_TypeDef*)GPIOA)
				{
				SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PA;
				SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;
				EXTI->RTSR &= ~EXTI_RTSR_TR4_Msk;
				EXTI->RTSR |= EXTI_RTSR_TR4;
				EXTI->IMR &= ~EXTI_IMR_IM4;
				EXTI->IMR |= EXTI_IMR_IM4;
				NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOB)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PB;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PB;
					EXTI->RTSR &= ~EXTI_RTSR_TR4_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR4;
									EXTI->IMR &= ~EXTI_IMR_IM4;
									EXTI->IMR |= EXTI_IMR_IM4;
									NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel


				}
				else if(&port == (GPIO_TypeDef*)GPIOC)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PC;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PC;
					EXTI->RTSR &= ~EXTI_RTSR_TR4_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR4;
									EXTI->IMR &= ~EXTI_IMR_IM4;
									EXTI->IMR |= EXTI_IMR_IM4;
									NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOD)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PD;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PD;
					EXTI->RTSR &= ~EXTI_RTSR_TR4_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR4;
									EXTI->IMR &= ~EXTI_IMR_IM4;
									EXTI->IMR |= EXTI_IMR_IM4;
									NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOE)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PE;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PE;
					EXTI->RTSR &= ~EXTI_RTSR_TR4_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR4;
									EXTI->IMR &= ~EXTI_IMR_IM4;
									EXTI->IMR |= EXTI_IMR_IM4;
									NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOF)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PF;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PF;
					EXTI->RTSR &= ~EXTI_RTSR_TR4_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR4;
									EXTI->IMR &= ~EXTI_IMR_IM4;
									EXTI->IMR |= EXTI_IMR_IM4;
									NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel

				}
				else{
					//for GPIOG
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI0_PG;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PG;
					EXTI->RTSR &= ~EXTI_RTSR_TR4_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR4;
									EXTI->IMR &= ~EXTI_IMR_IM4;
									EXTI->IMR |= EXTI_IMR_IM4;
									NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel

				}
			break;
			case 5:
			//this is for PA/B/C...0:3
				if(&port == (GPIO_TypeDef*)GPIOA)
				{
				SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_PA;
				SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PA;
				EXTI->RTSR &= ~EXTI_RTSR_TR5_Msk;
				EXTI->RTSR |= EXTI_RTSR_TR5;
				EXTI->IMR &= ~EXTI_IMR_IM5;
				EXTI->IMR |= EXTI_IMR_IM5;
				NVIC_EnableIRQ(EXTI4_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOB)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_PB;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PB;
					EXTI->RTSR &= ~EXTI_RTSR_TR5_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR5;
									EXTI->IMR &= ~EXTI_IMR_IM5;
									EXTI->IMR |= EXTI_IMR_IM5;
									NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOC)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_PC;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PC;
					EXTI->RTSR &= ~EXTI_RTSR_TR5_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR5;
									EXTI->IMR &= ~EXTI_IMR_IM5;
									EXTI->IMR |= EXTI_IMR_IM5;
									NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOD)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_PD;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PD;
					EXTI->RTSR &= ~EXTI_RTSR_TR5_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR5;
									EXTI->IMR &= ~EXTI_IMR_IM5;
									EXTI->IMR |= EXTI_IMR_IM5;
									NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOE)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_PE;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PE;
					EXTI->RTSR &= ~EXTI_RTSR_TR5_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR5;
									EXTI->IMR &= ~EXTI_IMR_IM5;
									EXTI->IMR |= EXTI_IMR_IM5;
									NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOF)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_PF;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PF;
					EXTI->RTSR &= ~EXTI_RTSR_TR5_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR5;
									EXTI->IMR &= ~EXTI_IMR_IM5;
									EXTI->IMR |= EXTI_IMR_IM5;
									NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else{
					//for GPIOG
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_PG;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PG;
					EXTI->RTSR &= ~EXTI_RTSR_TR5_Msk;
									EXTI->RTSR |= EXTI_RTSR_TR5;
									EXTI->IMR &= ~EXTI_IMR_IM5;
									EXTI->IMR |= EXTI_IMR_IM5;
									NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
			break;
			case 6:
			//this is for PA/B/C...0:3
				if(&port == (GPIO_TypeDef*)GPIOA)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI2_PA;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI2_PA;
					EXTI->RTSR &= ~EXTI_RTSR_TR6_Msk;
					EXTI->RTSR |= EXTI_RTSR_TR6;
					EXTI->IMR &= ~EXTI_IMR_IM6;
					EXTI->IMR |= EXTI_IMR_IM6;
					NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOB)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI2_PB;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI2_PB;
					EXTI->RTSR &= ~EXTI_RTSR_TR6_Msk;
										EXTI->RTSR |= EXTI_RTSR_TR6;
										EXTI->IMR &= ~EXTI_IMR_IM6;
										EXTI->IMR |= EXTI_IMR_IM6;
										NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel
				}
				else if(&port == (GPIO_TypeDef*)GPIOC)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI2_PC;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI2_PC;
					EXTI->RTSR &= ~EXTI_RTSR_TR6_Msk;
										EXTI->RTSR |= EXTI_RTSR_TR6;
										EXTI->IMR &= ~EXTI_IMR_IM6;
										EXTI->IMR |= EXTI_IMR_IM6;
										NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel
				}
				else if(&port == (GPIO_TypeDef*)GPIOD)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI2_PD;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI2_PD;
					EXTI->RTSR &= ~EXTI_RTSR_TR6_Msk;
										EXTI->RTSR |= EXTI_RTSR_TR6;
										EXTI->IMR &= ~EXTI_IMR_IM6;
										EXTI->IMR |= EXTI_IMR_IM6;
										NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOE)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI2_PE;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI2_PE;
					EXTI->RTSR &= ~EXTI_RTSR_TR6_Msk;
										EXTI->RTSR |= EXTI_RTSR_TR6;
										EXTI->IMR &= ~EXTI_IMR_IM6;
										EXTI->IMR |= EXTI_IMR_IM6;
										NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else if(&port == (GPIO_TypeDef*)GPIOF)
				{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI2_PF;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI2_PF;
					EXTI->RTSR &= ~EXTI_RTSR_TR6_Msk;
										EXTI->RTSR |= EXTI_RTSR_TR6;
										EXTI->IMR &= ~EXTI_IMR_IM6;
										EXTI->IMR |= EXTI_IMR_IM6;
										NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
				else{
					//for GPIOG
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI2_PG;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI2_PG;
					EXTI->RTSR &= ~EXTI_RTSR_TR6_Msk;
										EXTI->RTSR |= EXTI_RTSR_TR6;
										EXTI->IMR &= ~EXTI_IMR_IM6;
										EXTI->IMR |= EXTI_IMR_IM6;
										NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

				}
			break;
			case 7:
				//this is for PA/B/C...0:3
					if(&port == (GPIO_TypeDef*)GPIOA)
					{
					SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PA;
					SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PA;
					EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
					EXTI->RTSR |= EXTI_RTSR_TR7;
					EXTI->IMR &= ~EXTI_IMR_IM7;
					EXTI->IMR |= EXTI_IMR_IM7;
					NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOB)
					{
						SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PB;
						SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PB;
						EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR7;
											EXTI->IMR &= ~EXTI_IMR_IM7;
											EXTI->IMR |= EXTI_IMR_IM7;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOC)
					{
						SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PC;
						SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PC;
						EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR7;
											EXTI->IMR &= ~EXTI_IMR_IM7;
											EXTI->IMR |= EXTI_IMR_IM7;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOD)
					{
						SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PD;
						SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PD;
						EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR7;
											EXTI->IMR &= ~EXTI_IMR_IM7;
											EXTI->IMR |= EXTI_IMR_IM7;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOE)
					{
						SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PE;
						SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PE;
						EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR7;
											EXTI->IMR &= ~EXTI_IMR_IM7;
											EXTI->IMR |= EXTI_IMR_IM7;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOF)
					{
						SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PF;
						SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PF;
						EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR7;
											EXTI->IMR &= ~EXTI_IMR_IM7;
											EXTI->IMR |= EXTI_IMR_IM7;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else{
							//for GPIOG
						SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PG;
						SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PG;
						EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR7;
											EXTI->IMR &= ~EXTI_IMR_IM7;
											EXTI->IMR |= EXTI_IMR_IM7;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel
					}
				break;




			case 8:
				//this is for PA/B/C...0:3
					if(&port == (GPIO_TypeDef*)GPIOA)
					{
					SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI0_PA;
					SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PA;
					EXTI->RTSR &= ~EXTI_RTSR_TR8_Msk;
					EXTI->RTSR |= EXTI_RTSR_TR8;
					EXTI->IMR &= ~EXTI_IMR_IM8;
					EXTI->IMR |= EXTI_IMR_IM8;
					NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOB)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI0_PB;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PB;
						EXTI->RTSR &= ~EXTI_RTSR_TR8_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR8;
											EXTI->IMR &= ~EXTI_IMR_IM8;
											EXTI->IMR |= EXTI_IMR_IM8;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOC)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI0_PC;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PC;
						EXTI->RTSR &= ~EXTI_RTSR_TR8_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR8;
											EXTI->IMR &= ~EXTI_IMR_IM8;
											EXTI->IMR |= EXTI_IMR_IM8;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOD)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI0_PD;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PD;
						EXTI->RTSR &= ~EXTI_RTSR_TR8_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR8;
											EXTI->IMR &= ~EXTI_IMR_IM8;
											EXTI->IMR |= EXTI_IMR_IM8;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOE)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI0_PE;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PE;
						EXTI->RTSR &= ~EXTI_RTSR_TR8_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR8;
											EXTI->IMR &= ~EXTI_IMR_IM8;
											EXTI->IMR |= EXTI_IMR_IM8;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOF)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI0_PF;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PF;
						EXTI->RTSR &= ~EXTI_RTSR_TR8_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR8;
											EXTI->IMR &= ~EXTI_IMR_IM8;
											EXTI->IMR |= EXTI_IMR_IM8;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else{
						//for GPIOG
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI0_PG;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI0_PG;
						EXTI->RTSR &= ~EXTI_RTSR_TR8_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR8;
											EXTI->IMR &= ~EXTI_IMR_IM8;
											EXTI->IMR |= EXTI_IMR_IM8;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
				break;
				case 9:
				//this is for PA/B/C...0:3
					if(&port == (GPIO_TypeDef*)GPIOA)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI1_PA;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI1_PA;
						EXTI->RTSR &= ~EXTI_RTSR_TR9_Msk;
						EXTI->RTSR |= EXTI_RTSR_TR9;
						EXTI->IMR &= ~EXTI_IMR_IM9;
						EXTI->IMR |= EXTI_IMR_IM9;
						NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOB)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI1_PB;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI1_PB;
						EXTI->RTSR &= ~EXTI_RTSR_TR9_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR9;
											EXTI->IMR &= ~EXTI_IMR_IM9;
											EXTI->IMR |= EXTI_IMR_IM9;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOC)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI1_PC;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI1_PC;
						EXTI->RTSR &= ~EXTI_RTSR_TR9_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR9;
											EXTI->IMR &= ~EXTI_IMR_IM9;
											EXTI->IMR |= EXTI_IMR_IM9;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOD)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI1_PD;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI1_PD;
						EXTI->RTSR &= ~EXTI_RTSR_TR9_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR9;
											EXTI->IMR &= ~EXTI_IMR_IM9;
											EXTI->IMR |= EXTI_IMR_IM9;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOE)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI1_PE;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI1_PE;
						EXTI->RTSR &= ~EXTI_RTSR_TR9_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR9;
											EXTI->IMR &= ~EXTI_IMR_IM9;
											EXTI->IMR |= EXTI_IMR_IM9;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOF)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI1_PF;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI1_PF;
						EXTI->RTSR &= ~EXTI_RTSR_TR9_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR9;
											EXTI->IMR &= ~EXTI_IMR_IM9;
											EXTI->IMR |= EXTI_IMR_IM9;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
					else{
						//for GPIOG
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI1_PG;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI1_PG;
						EXTI->RTSR &= ~EXTI_RTSR_TR9_Msk;
											EXTI->RTSR |= EXTI_RTSR_TR9;
											EXTI->IMR &= ~EXTI_IMR_IM9;
											EXTI->IMR |= EXTI_IMR_IM9;
											NVIC_EnableIRQ(EXTI9_5_IRQn); //enable interrupt channel

					}
				break;
				case 10:
				//this is for PA/B/C...0:3
					if(&port == (GPIO_TypeDef*)GPIOA)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PA;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PA;
						EXTI->RTSR &= ~EXTI_RTSR_TR10_Msk;
						EXTI->RTSR |= EXTI_RTSR_TR10;
						EXTI->IMR &= ~EXTI_IMR_IM10;
						EXTI->IMR |= EXTI_IMR_IM10;
						NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOB)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PB;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PB;
						EXTI->RTSR &= ~EXTI_RTSR_TR10_Msk;
												EXTI->RTSR |= EXTI_RTSR_TR10;
												EXTI->IMR &= ~EXTI_IMR_IM10;
												EXTI->IMR |= EXTI_IMR_IM10;
												NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOC)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PC;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PC;
						EXTI->RTSR &= ~EXTI_RTSR_TR10_Msk;
												EXTI->RTSR |= EXTI_RTSR_TR10;
												EXTI->IMR &= ~EXTI_IMR_IM10;
												EXTI->IMR |= EXTI_IMR_IM10;
												NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOD)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PD;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PD;
						EXTI->RTSR &= ~EXTI_RTSR_TR10_Msk;
												EXTI->RTSR |= EXTI_RTSR_TR10;
												EXTI->IMR &= ~EXTI_IMR_IM10;
												EXTI->IMR |= EXTI_IMR_IM10;
												NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOE)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PE;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PE;
						EXTI->RTSR &= ~EXTI_RTSR_TR10_Msk;
												EXTI->RTSR |= EXTI_RTSR_TR10;
												EXTI->IMR &= ~EXTI_IMR_IM10;
												EXTI->IMR |= EXTI_IMR_IM10;
												NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

					}
					else if(&port == (GPIO_TypeDef*)GPIOF)
					{
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PF;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PF;
						EXTI->RTSR &= ~EXTI_RTSR_TR10_Msk;
												EXTI->RTSR |= EXTI_RTSR_TR10;
												EXTI->IMR &= ~EXTI_IMR_IM10;
												EXTI->IMR |= EXTI_IMR_IM10;
												NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

					}
					else{
						//for GPIOG
						SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI2_PG;
						SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI2_PG;
						EXTI->RTSR &= ~EXTI_RTSR_TR10_Msk;
												EXTI->RTSR |= EXTI_RTSR_TR10;
												EXTI->IMR &= ~EXTI_IMR_IM10;
												EXTI->IMR |= EXTI_IMR_IM10;
												NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

					}
				break;
				case 11:
					//this is for PA/B/C...0:3
						if(&port == (GPIO_TypeDef*)GPIOA)
						{
							SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI3_PA;
							SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI3_PA;
							EXTI->RTSR &= ~EXTI_RTSR_TR11_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR11;
							EXTI->IMR &= ~EXTI_IMR_IM11;
							EXTI->IMR |= EXTI_IMR_IM11;
							NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOB)
						{
							SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI3_PB;
							SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI3_PB;
							EXTI->RTSR &= ~EXTI_RTSR_TR11_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR11;
														EXTI->IMR &= ~EXTI_IMR_IM11;
														EXTI->IMR |= EXTI_IMR_IM11;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOC)
						{
							SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI3_PC;
							SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI3_PC;
							EXTI->RTSR &= ~EXTI_RTSR_TR11_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR11;
														EXTI->IMR &= ~EXTI_IMR_IM11;
														EXTI->IMR |= EXTI_IMR_IM11;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOD)
						{
							SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI3_PD;
							SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI3_PD;
							EXTI->RTSR &= ~EXTI_RTSR_TR11_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR11;
														EXTI->IMR &= ~EXTI_IMR_IM11;
														EXTI->IMR |= EXTI_IMR_IM11;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOE)
						{
							SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI3_PE;
							SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI3_PE;
							EXTI->RTSR &= ~EXTI_RTSR_TR11_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR11;
														EXTI->IMR &= ~EXTI_IMR_IM11;
														EXTI->IMR |= EXTI_IMR_IM11;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOF)
						{
							SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI3_PF;
							SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI3_PF;
							EXTI->RTSR &= ~EXTI_RTSR_TR11_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR11;
														EXTI->IMR &= ~EXTI_IMR_IM11;
														EXTI->IMR |= EXTI_IMR_IM11;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else{
								//for GPIOG
							SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR1_EXTI3_PG;
							SYSCFG->EXTICR[2] |= SYSCFG_EXTICR1_EXTI3_PG;
							EXTI->RTSR &= ~EXTI_RTSR_TR11_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR11;
														EXTI->IMR &= ~EXTI_IMR_IM11;
														EXTI->IMR |= EXTI_IMR_IM11;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
					break;




				case 12:
					//this is for PA/B/C...0:3
						if(&port == (GPIO_TypeDef*)GPIOA)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI0_PA;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PA;
							EXTI->RTSR &= ~EXTI_RTSR_TR12_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR12;
							EXTI->IMR &= ~EXTI_IMR_IM12;
							EXTI->IMR |= EXTI_IMR_IM12;
							NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOB)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI0_PB;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PB;
							EXTI->RTSR &= ~EXTI_RTSR_TR12_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR12;
														EXTI->IMR &= ~EXTI_IMR_IM12;
														EXTI->IMR |= EXTI_IMR_IM12;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOC)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI0_PC;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PC;
							EXTI->RTSR &= ~EXTI_RTSR_TR12_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR12;
														EXTI->IMR &= ~EXTI_IMR_IM12;
														EXTI->IMR |= EXTI_IMR_IM12;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOD)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI0_PD;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PD;
							EXTI->RTSR &= ~EXTI_RTSR_TR12_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR12;
														EXTI->IMR &= ~EXTI_IMR_IM12;
														EXTI->IMR |= EXTI_IMR_IM12;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOE)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI0_PE;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PE;
							EXTI->RTSR &= ~EXTI_RTSR_TR12_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR12;
														EXTI->IMR &= ~EXTI_IMR_IM12;
														EXTI->IMR |= EXTI_IMR_IM12;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOF)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI0_PF;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PF;
							EXTI->RTSR &= ~EXTI_RTSR_TR12_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR12;
														EXTI->IMR &= ~EXTI_IMR_IM12;
														EXTI->IMR |= EXTI_IMR_IM12;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else{
							//for GPIOG
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI0_PG;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI0_PG;
							EXTI->RTSR &= ~EXTI_RTSR_TR12_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR12;
														EXTI->IMR &= ~EXTI_IMR_IM12;
														EXTI->IMR |= EXTI_IMR_IM12;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
					break;
					case 13:
					//this is for PA/B/C...0:3
						if(&port == (GPIO_TypeDef*)GPIOA)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI1_PA;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI1_PA;
							EXTI->RTSR &= ~EXTI_RTSR_TR13_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR13;
							EXTI->IMR &= ~EXTI_IMR_IM13;
							EXTI->IMR |= EXTI_IMR_IM13;
							NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOB)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI1_PB;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI1_PB;
							EXTI->RTSR &= ~EXTI_RTSR_TR13_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR13;
														EXTI->IMR &= ~EXTI_IMR_IM13;
														EXTI->IMR |= EXTI_IMR_IM13;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOC)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI1_PC;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI1_PC;
							EXTI->RTSR &= ~EXTI_RTSR_TR13_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR13;
														EXTI->IMR &= ~EXTI_IMR_IM13;
														EXTI->IMR |= EXTI_IMR_IM13;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOD)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI1_PD;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI1_PD;
							EXTI->RTSR &= ~EXTI_RTSR_TR13_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR13;
														EXTI->IMR &= ~EXTI_IMR_IM13;
														EXTI->IMR |= EXTI_IMR_IM13;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOE)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI1_PE;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI1_PE;
							EXTI->RTSR &= ~EXTI_RTSR_TR13_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR13;
														EXTI->IMR &= ~EXTI_IMR_IM13;
														EXTI->IMR |= EXTI_IMR_IM13;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOF)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI1_PF;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI1_PF;
							EXTI->RTSR &= ~EXTI_RTSR_TR13_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR13;
														EXTI->IMR &= ~EXTI_IMR_IM13;
														EXTI->IMR |= EXTI_IMR_IM13;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else{
							//for GPIOG
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI1_PG;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI1_PG;
							EXTI->RTSR &= ~EXTI_RTSR_TR13_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR13;
														EXTI->IMR &= ~EXTI_IMR_IM13;
														EXTI->IMR |= EXTI_IMR_IM13;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
					break;
					case 14:
					//this is for PA/B/C...0:3
						if(&port == (GPIO_TypeDef*)GPIOA)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2_PA;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI2_PA;
							EXTI->RTSR &= ~EXTI_RTSR_TR14_Msk;
							EXTI->RTSR |= EXTI_RTSR_TR14;
							EXTI->IMR &= ~EXTI_IMR_IM14;
							EXTI->IMR |= EXTI_IMR_IM14;
							NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOB)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2_PB;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI2_PB;
							EXTI->RTSR &= ~EXTI_RTSR_TR14_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR14;
														EXTI->IMR &= ~EXTI_IMR_IM14;
														EXTI->IMR |= EXTI_IMR_IM14;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOC)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2_PC;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI2_PC;
							EXTI->RTSR &= ~EXTI_RTSR_TR14_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR14;
														EXTI->IMR &= ~EXTI_IMR_IM14;
														EXTI->IMR |= EXTI_IMR_IM14;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOD)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2_PD;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI2_PD;
							EXTI->RTSR &= ~EXTI_RTSR_TR14_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR14;
														EXTI->IMR &= ~EXTI_IMR_IM14;
														EXTI->IMR |= EXTI_IMR_IM14;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOE)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2_PE;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI2_PE;
							EXTI->RTSR &= ~EXTI_RTSR_TR14_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR14;
														EXTI->IMR &= ~EXTI_IMR_IM14;
														EXTI->IMR |= EXTI_IMR_IM14;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else if(&port == (GPIO_TypeDef*)GPIOF)
						{
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2_PF;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI2_PF;
							EXTI->RTSR &= ~EXTI_RTSR_TR14_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR14;
														EXTI->IMR &= ~EXTI_IMR_IM14;
														EXTI->IMR |= EXTI_IMR_IM14;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
						else{
							//for GPIOG
							SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI2_PG;
							SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI2_PG;
							EXTI->RTSR &= ~EXTI_RTSR_TR14_Msk;
														EXTI->RTSR |= EXTI_RTSR_TR14;
														EXTI->IMR &= ~EXTI_IMR_IM14;
														EXTI->IMR |= EXTI_IMR_IM14;
														NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

						}
					break;
					case 15:
						//this is for PA/B/C...0:3
							if(&port == (GPIO_TypeDef*)GPIOA)
							{
								SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI3_PA;
								SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI3_PA;
								EXTI->RTSR &= ~EXTI_RTSR_TR15_Msk;
								EXTI->RTSR |= EXTI_RTSR_TR15;
								EXTI->IMR &= ~EXTI_IMR_IM15;
								EXTI->IMR |= EXTI_IMR_IM15;
								NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

							}
							else if(&port == (GPIO_TypeDef*)GPIOB)
							{
								SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI3_PB;
								SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI3_PB;
								EXTI->RTSR &= ~EXTI_RTSR_TR15_Msk;
																EXTI->RTSR |= EXTI_RTSR_TR15;
																EXTI->IMR &= ~EXTI_IMR_IM15;
																EXTI->IMR |= EXTI_IMR_IM15;
																NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

							}
							else if(&port == (GPIO_TypeDef*)GPIOC)
							{
								SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI3_PC;
								SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI3_PC;
								EXTI->RTSR &= ~EXTI_RTSR_TR15_Msk;
																EXTI->RTSR |= EXTI_RTSR_TR15;
																EXTI->IMR &= ~EXTI_IMR_IM15;
																EXTI->IMR |= EXTI_IMR_IM15;
																NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

							}
							else if(&port == (GPIO_TypeDef*)GPIOD)
							{
								SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI3_PD;
								SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI3_PD;
								EXTI->RTSR &= ~EXTI_RTSR_TR15_Msk;
																EXTI->RTSR |= EXTI_RTSR_TR15;
																EXTI->IMR &= ~EXTI_IMR_IM15;
																EXTI->IMR |= EXTI_IMR_IM15;
																NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

							}
							else if(&port == (GPIO_TypeDef*)GPIOE)
							{
								SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI3_PE;
								SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI3_PE;
								EXTI->RTSR &= ~EXTI_RTSR_TR15_Msk;
																EXTI->RTSR |= EXTI_RTSR_TR15;
																EXTI->IMR &= ~EXTI_IMR_IM15;
																EXTI->IMR |= EXTI_IMR_IM15;
																NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

							}
							else if(&port == (GPIO_TypeDef*)GPIOF)
							{
								SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI3_PF;
								SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI3_PF;
								EXTI->RTSR &= ~EXTI_RTSR_TR15_Msk;
																EXTI->RTSR |= EXTI_RTSR_TR15;
																EXTI->IMR &= ~EXTI_IMR_IM15;
																EXTI->IMR |= EXTI_IMR_IM15;
																NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

							}
							else{
									//for GPIOG
								SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR1_EXTI3_PG;
								SYSCFG->EXTICR[3] |= SYSCFG_EXTICR1_EXTI3_PG;
								EXTI->RTSR &= ~EXTI_RTSR_TR15_Msk;
																EXTI->RTSR |= EXTI_RTSR_TR15;
																EXTI->IMR &= ~EXTI_IMR_IM15;
																EXTI->IMR |= EXTI_IMR_IM15;
																NVIC_EnableIRQ(EXTI15_10_IRQn); //enable interrupt channel

							}
						break;
	}

}










//will need to list ALL interrupt pins on GPIO -> check whether they ALL are from same GPIO type!
uint8_t GPIO_read(GPIO *gpio)
{
	uint8_t returnVal;
	if(gpio->MODER != 0x00){
		return 0; //mode check to ensure the GPIO mode is set correctly
	}
	else{
		returnVal = (gpio->port->IDR<<(gpio->POS)); //read out the value at the GPIO port specified
		return returnVal;
	}
}

void GPIO_write(GPIO *gpio)
{
	uint8_t state = gpio->port->ODR<<(gpio->POS);
	if(gpio->MODER !=0x01){
		return 0;
	}
	else{
		if(state == 0){ //checking whether the GPIO is on or off

			gpio->port->ODR |= 0x01<<gpio->POS;//toggle GPIO on
		}
		else{
			gpio->port->ODR |= 0x00<<gpio->POS; //toggle the GPIO off

		}
	}

}
