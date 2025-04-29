/*
 * driver.c
 *
 *  Created on: Feb 11, 2025
 *      Author: lucas
 */

#include "stm32f4xx_hal.h"
#include "driver.h"
	void configureI2CBus1(void)
{ //for BUS, mode is alternate function
	GPIOF->MODER &= (~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk)); //everything except for
	GPIOF->MODER |= (0x03<<GPIO_MODER_MODE0_Pos | 0x03 << GPIO_MODER_MODE1_Pos);
	GPIOF->OTYPER &= (~(GPIO_OTYPER_OT0_Msk | GPIO_OTYPER_OT1_Msk));
	GPIOF->OTYPER |= (0x01 << GPIO_OTYPER_OT0_Pos | 0x01<<GPIO_OTYPER_OT1_Pos); //open drain
	GPIOF->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED0_Msk | GPIO_OSPEEDR_OSPEED1_Msk));
	GPIOF->OSPEEDR |= 0x01 << GPIO_OSPEEDR_OSPEED0_Pos | 0x01 << GPIO_OSPEEDR_OSPEED1_Pos;

	GPIOF->PUPDR &= (~(GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD1_Msk));
	GPIOF->PUPDR |= 0x01<<GPIO_PUPDR_PUPD0_Pos | 0x01<<GPIO_PUPDR_PUPD1_Pos; //enabling internal pull ups on I2C lines

	//turn on alternate function of I2C
	GPIOF->AFR[0] |= 0x04; //alt function 3 //PF0
	GPIOF->AFR[0] |= (0x04 << (4*1)); //PF1

	I2C2->CR2 &= (~(I2C_CR2_FREQ_Msk));
	I2C2->CR2 = 45; //for 45mhz = PCLK1 for AHB1 -> remember use decimal 45!
	I2C2->CCR &= (~(0xFFF)); // clears bits 11:0
	I2C2->CCR |= 0x28; //for 100KHz
	I2C2->TRISE &= ~(I2C_TRISE_TRISE_Msk);
	I2C2->TRISE |= 0x46; //
	I2C2->CR1 = I2C_CR1_PE; //starts the protocol
}



void configureSPIBus1(void) //for ADC transducers
{
	GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODE5_Pos | 0x02 << GPIO_MODER_MODE6_Pos | 0x02 << GPIO_MODER_MODE7_Pos);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk | GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk); //if needing to change internal pull up/downs
	GPIOA->PUPDR |= (0x01 << GPIO_PUPDR_PUPD5_Pos | 0x01 << GPIO_PUPDR_PUPD6_Pos | 0x01 << GPIO_PUPDR_PUPD7_Pos); //internal pull ups on SCK, MOSI and MISO
	GPIOA->OTYPER &= (uint16_t)~(GPIO_OTYPER_OT5_Msk | GPIO_OTYPER_OT6_Msk | GPIO_OTYPER_OT7_Msk ); //push pull de
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED6_Msk | GPIO_OSPEEDR_OSPEED7_Msk);
	GPIOA->OSPEEDR |= (0x03<<GPIO_OSPEEDR_OSPEED5_Pos | 0x03<<GPIO_OSPEEDR_OSPEED6_Pos | 0x03<<GPIO_OSPEEDR_OSPEED7_Pos); //fast mode
    GPIOA->AFR[0] |= (5 << (4 * 5)) | (5 << (4 * 6)) | (5 << (4 * 7));

	//Chip Select for Transducer: PG4
	GPIOG->MODER |= 0x01 << GPIO_MODER_MODE4_Pos;
	GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD4_Msk);
	GPIOG->OTYPER &= ~(0x01 << GPIO_OTYPER_OT4_Pos); //Push Pull
	GPIOG->OSPEEDR |= 0x01<<GPIO_OSPEEDR_OSPEED4_Pos;
	GPIOG->ODR |= GPIO_ODR_OD4; //raise up CS of PG

	//Chip select for Loadcell: PA2
	GPIOA->MODER |= 0x01 << GPIO_MODER_MODE2_Pos;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2_Msk);
	GPIOA->OTYPER |= 0x00 <<GPIO_OTYPER_OT2_Pos;
	GPIOA->OSPEEDR |= 0x01<<GPIO_OSPEEDR_OSPEED2_Pos;
	GPIOA->ODR |= GPIO_ODR_OD2; //raise up CS of PG

	//timer config for SPI1 -> remember to change RCC to allow for TIM7
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);
	TIM7->PSC &= ~(TIM_PSC_PSC_Msk);

	SPI1->CR1 &= (~(SPI_CR1_BR_Msk));
	SPI1->CR1 |= (0x04 <<SPI_CR1_BR_Pos); //SPIclk/32 //~1MHZ

	SPI1->CR1 |= (SPI_CR1_CPHA); //
	//SPI1->CR1 |= SPI_CR1_CPHA; // CPHA mode 1 //comment for mode 0
	SPI1->CR1 |= (SPI_CR1_CPOL);
	//SPI1->CR1 |= SPI_CR1_CPOL; //CPOL mode 1 //comment for mode 0
	//Clock is IDLE high, and polarity is on the falling edge!

	SPI1->CR1 |= SPI_CR1_MSTR; //sets SPI to master mode
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; //set both bits;

	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST); //MSB
	SPI1->CR1 |= SPI_CR1_DFF; //16 bit mode has been selected!
	SPI1->CR1 &= ~(SPI_CR1_RXONLY | SPI_CR1_BIDIMODE);
	SPI1->CR1 |= SPI_CR1_SPE; //enables the protocol
}


void configureSPIBus5(void) // For ADC or external SPI device via SPI5
{
    // Config PF7 (Clock), PF8 (MISO), PF9 (MOSI) for alternate function mode
    GPIOF->MODER &= ~(GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk);
    GPIOF->MODER |= (0x2 << GPIO_MODER_MODE7_Pos) | (0x2 << GPIO_MODER_MODE8_Pos) | (0x2 << GPIO_MODER_MODE9_Pos);

    // PUPDs
    GPIOF->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk | GPIO_PUPDR_PUPD8_Msk | GPIO_PUPDR_PUPD9_Msk);
    GPIOF->PUPDR |= (0x1 << GPIO_PUPDR_PUPD7_Pos) | (0x1 << GPIO_PUPDR_PUPD8_Pos) | (0x1 << GPIO_PUPDR_PUPD9_Pos);

    // Push Pull
    GPIOF->OTYPER &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);

    // Speed - fast
    GPIOF->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7_Msk | GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED9_Msk);
    GPIOF->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED7_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED8_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED9_Pos);

    // Alternate Function 5 for SPI5 on PF7, PF8, PF9
    GPIOF->AFR[0] &= ~(0xF << (4 * 7));                      // PF7 (SCK)
    GPIOF->AFR[1] &= ~((0xF << (4 * (8 - 8))) | (0xF << (4 * (9 - 8)))); // PF8, PF9
    GPIOF->AFR[0] |= (5 << (4 * 7));
    GPIOF->AFR[1] |= (5 << (4 * (8 - 8))) | (5 << (4 * (9 - 8)));
}


void configureSPIBus6(void)// 	//for both additional 5V channels and LoRa
{
	GPIOG->MODER &= ~(GPIO_MODER_MODE12_Msk | GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk);
	GPIOG->MODER |= (0x02 <<GPIO_MODER_MODE12_Pos | 0x02 << GPIO_MODER_MODE13_Pos | 0x02 << GPIO_MODER_MODE14_Pos);
	GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD12_Msk | GPIO_PUPDR_PUPD13_Msk | GPIO_PUPDR_PUPD14_Msk); //if needing to change internal pull up/downs
	GPIOG->PUPDR |= (0x01 << GPIO_PUPDR_PUPD12_Pos | 0x01 << GPIO_PUPDR_PUPD13_Pos | 0x01 << GPIO_PUPDR_PUPD14_Pos); //internal pull ups on SCK, MOSI and MISO
		//@var PG12 = SDO
		//		PG13 = SCLK
		// 		 PG14 = SDI
	//GPIO for LoRa, RF Switch & Chip select
	GPIOG->MODER &= ~(GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE11_Msk);
	GPIOG->MODER |= (0x01 <<GPIO_MODER_MODE9_Pos | 0x01 <<GPIO_MODER_MODE10_Pos | 0x01 <<GPIO_MODER_MODE11_Pos);
	//@var PG9 = SX_RESET
	//		PG10 = RF_SWITCH
	//		 PG11 = CS

	//timer config for LoRa
		//Use the same config as Australis board
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);
	TIM6->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM6->ARR |= 20000;
	TIM6->PSC |= 251; //same as australis to remain consistent!

	GPIOG->ODR |= (GPIO_ODR_OD9); //hold reset high for SX1272
	TIM6->CR1 |= TIM_CR1_CEN; //enable TIM6
//	while((TIM6->CR1 & TIM_SR_UIF)==0); //wait for hardware registers to be updated
	GPIOG->ODR &= ~(GPIO_ODR_OD9); //resets reset on GPIO
	TIM6->CR1 &= ~(TIM_SR_UIF); //clears UIF register

	//
	GPIOG->OTYPER &= (uint16_t)~(GPIO_OTYPER_OT12_Msk | GPIO_OTYPER_OT13_Msk | GPIO_OTYPER_OT14_Msk); //push pull
	GPIOG->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12_Msk | GPIO_OSPEEDR_OSPEED13_Msk | GPIO_OSPEEDR_OSPEED14_Msk);
	GPIOG->OSPEEDR |= (0x02<<GPIO_OSPEEDR_OSPEED12_Pos | 0x02<<GPIO_OSPEEDR_OSPEED13_Pos | 0x02<<GPIO_OSPEEDR_OSPEED14_Pos); //fast mode
	GPIOG->ODR |= GPIO_ODR_OD11; //raise up CS

	GPIOG->AFR[1] &= ~((GPIO_AFRH_AFRH5) | (GPIO_AFRH_AFRH6) | (GPIO_AFRH_AFRH7));// alternate functions for SPI3
	GPIOG->AFR[1] |= ((0x05 << 4*4) | (0x05 << 5*4) | (0x05 << 6*4));// alternate functions for SPI3

    SPI6->CR1 &= ~(SPI_CR1_BR_Msk); // Clear baud rate bits
    SPI6->CR1 |= (0x03 << SPI_CR1_BR_Pos); // SPIclk/8
    SPI6->CR1 &= ~(SPI_CR1_CPHA_Msk | SPI_CR1_CPOL_Msk); // CPHA = 0, CPOL = 0
    SPI6->CR1 |= SPI_CR1_MSTR; // Master mode
    SPI6->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software slave management
    SPI6->CR1 &= ~(SPI_CR1_LSBFIRST); // MSB first
   // SPI6->CR1 |= SPI_CR1_DFF; // 16-bit mode
    SPI6->CR1 &= ~(SPI_CR1_RXONLY | SPI_CR1_BIDIMODE); // Full duplex
    SPI6->CR1 |= SPI_CR1_SPE; // Enable SPI
}

void configureSPIBus4(void) //for flash memory storage
{
	//PE11: CS4
	//PE12: SCK4
	//PE13: SDO
	//PE14: SDI
	GPIOE->MODER &= ~(GPIO_MODER_MODE12_Msk | GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk);
	GPIOE->MODER |= (0x02 <<GPIO_MODER_MODE12_Pos | 0x02 << GPIO_MODER_MODE13_Pos | 0x02 << GPIO_MODER_MODE14_Pos);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD12_Msk | GPIO_PUPDR_PUPD13_Msk | GPIO_PUPDR_PUPD14_Msk); //if needing to change internal pull up/downs
	GPIOE->PUPDR |= (0x01 << GPIO_PUPDR_PUPD12_Pos | 0x01 << GPIO_PUPDR_PUPD13_Pos | 0x01 << GPIO_PUPDR_PUPD14_Pos); //internal pull ups on SCK, MOSI and MISO
//PE 11 initial config
	GPIOE->MODER &= ~(GPIO_MODER_MODE11_Msk);
	GPIOE->MODER |= 0x01 << GPIO_MODER_MODE11_Pos;
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD11_Msk);
	GPIOE->OTYPER &= (uint16_t)~(GPIO_OTYPER_OT12_Msk | GPIO_OTYPER_OT13_Msk | GPIO_OTYPER_OT14_Msk);
	GPIOE->OTYPER &= (uint16_t)~(GPIO_OTYPER_OT11_Msk);//CS
	GPIOE->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12_Msk | GPIO_OSPEEDR_OSPEED13_Msk | GPIO_OSPEEDR_OSPEED14_Msk);
	GPIOE->OSPEEDR |= (0x02 << GPIO_OSPEEDR_OSPEED12_Pos | 0x02 << GPIO_OSPEEDR_OSPEED13_Pos | 0x02 << GPIO_OSPEEDR_OSPEED14_Pos);
	GPIOE->OSPEEDR |= 0x01 << GPIO_OSPEEDR_OSPEED11_Pos;

	//all alt function 5
	GPIOE->AFR[1] &= ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6);
	GPIOE->AFR[1] |= ((0x05 << GPIO_AFRL_AFRL4) | (0x05 << GPIO_AFRL_AFRL5) | (0x05 << GPIO_AFRL_AFRL6));

	SPI4->CR1 &= (~(SPI_CR1_BR_Msk));
	SPI4->CR1 |= (0x02 <<SPI_CR1_BR_Pos); //SPIclk/8
	SPI4->CR1 &= (~(SPI_CR1_CPHA_Msk | SPI_CR1_CPOL_Msk)); //as per SX specifications

	SPI4->CR1 |= SPI_CR1_MSTR; //sets SPI to master mode
	SPI4->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; //set both bits;

	SPI4->CR1 &= ~(SPI_CR1_LSBFIRST); //MSB
	SPI4->CR1 |= SPI_CR1_DFF;
	SPI4->CR1 &= (~(SPI_CR1_RXONLY | SPI_CR1_BIDIMODE));
	SPI4->CR1 |= (SPI_CR1_SPE); //enables the protocol
	//rise and repeat for all SPI buses
}


void configureRCC_APB1(void)
{
	RCC->APB1ENR &= ~(RCC_APB1ENR_I2C2EN | RCC_APB1ENR_SPI3EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN);
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN | RCC_APB1ENR_SPI3EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_I2C2RST |  RCC_APB1RSTR_SPI3RST | RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB1RSTR &= (uint16_t)~(RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_I2C2RST | RCC_APB1RSTR_SPI3RST | RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST);
	__ASM("NOP");
	__ASM("NOP");
	//configure for more timers when necessary
}

void configureRCC_APB2(void)
{
	RCC->APB2ENR &=  ~(RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SPI4EN |RCC_APB2ENR_SPI6EN| RCC_APB2ENR_SYSCFGEN /*important for interrupts and other sys init*/| RCC_APB2ENR_USART6EN | RCC_APB2ENR_TIM1EN);
	RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SPI4EN |RCC_APB2ENR_SPI6EN| RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART6EN | RCC_APB2ENR_TIM1EN;

	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST | RCC_APB2RSTR_SPI4RST|RCC_APB2RSTR_SPI6RST|RCC_APB2RSTR_SYSCFGRST | RCC_APB2RSTR_USART6RST | RCC_APB2RSTR_TIM11RST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB2RSTR &= (uint16_t)~(RCC_APB2RSTR_SPI1RST | RCC_APB2RSTR_SPI4RST|RCC_APB2RSTR_SPI6RST| RCC_APB2RSTR_SYSCFGRST | RCC_APB2RSTR_USART6RST | RCC_APB2RSTR_TIM11RST);
	__ASM("NOP");
	__ASM("NOP");
}
void configureRCC_AHB1(void)
{
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN);
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN);
	__ASM("NOP");
	__ASM("NOP");
	RCC->AHB1RSTR &= (uint16_t)(~(RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOCRST | RCC_AHB1RSTR_GPIOCRST | RCC_AHB1RSTR_GPIODRST | RCC_AHB1RSTR_GPIOERST | RCC_AHB1RSTR_GPIOFRST | RCC_AHB1RSTR_GPIOGRST));
}

void configure_TIM1(void) //operating at 5Hz or 200ms
{
	TIM1->ARR &= ~(TIM_ARR_ARR_Msk);
	TIM1->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM1->ARR |= (20001-1);
	TIM1->PSC |= (1801-1);

	//when the timer overflows, an interrupt will trigger!
	TIM1->DIER &= !(TIM_DIER_UIE_Msk);
	TIM1->DIER |= (TIM_DIER_UIE); //enable update event interrupt

	TIM1->CR1 |= TIM_CR1_CEN; //enable TIM6
	while((TIM1->SR & TIM_SR_UIF)==0); //wait for hardware registers to be updated
	TIM1->SR &= ~(TIM_SR_UIF); //clears UIF register

	NVIC_SetPriority(TIM1_UP_TIM10_IRQn,1);
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/*
void configure_USART6(void) { //change values here
  GPIOD->MODER &= (~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE13_Msk));
  GPIOD->MODER |= ((0x2 << GPIO_MODER_MODE8_Pos) | (0x2 << GPIO_MODER_MODE9_Pos) | (0x1 << GPIO_MODER_MODE13_Pos));
  GPIOD->AFR[1] &= (uint32_t)(~(0x000000FF)); // clears AFRL 6 and 7
  GPIOD->AFR[1] |= (0x00000077);              // sets PD 8, 9 and 13 to AF7

  GPIOD->PUPDR |= (0X1 << GPIO_PUPDR_PUPD9_Pos);

  GPIOD->OSPEEDR &= (~(GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED13));
  GPIOD->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED8_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED9_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED13_Pos);

  // need over sampling = 1
  USART3->BRR &= (unsigned int)(0xFFFF0000); //  clear mantissa and div in baud rate reg
  USART3->BRR |= (0x0002227);                // set mantissa and div in baud rate reg to 9600

  USART3->CR1 &= (unsigned int)(~(0x400));   // disable parity
  USART3->CR2 &= (unsigned int)(~(0xE00));   // disable synchrnous mode
  USART3->CR3 &= (unsigned int)(~(0x300));   // disable flow control
  USART3->CR1 |= (unsigned int)(0x200C);     // enable usart, enable receive and transmitt
  USART3->CR1 |= USART_CR1_OVER8;

  // turn reset pin high
  GPIOD->ODR |= GPIO_ODR_OD13;
}*/


