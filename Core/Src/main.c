/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
//These are here instead of main.h as I don't wanna go into different documents to remember definitions
uint8_t switch_case_state = 0; //global variable for state machine!
uint8_t state = 0;
uint8_t state_local = 0;
uint8_t local_dump_flag = 0;
uint16_t error = 0;
uint8_t hardware_timer_count = 0;
uint8_t dump_flag = 0;
uint8_t TX_Packet_Flag = 0; //ID 7
volatile uint8_t data_thermo[2];
volatile uint8_t test_rx_interrupt =0;
uint8_t lora_error = 0; //mainly for lora comms error state
bool triggerRX = false;
uint8_t pointerdata[LORA_MSG_LENGTH];

 	 //Error Code definitions here;
/*
 * ->


*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

 static SX1272_t lora;
 static LoRa_Packet GSE_GCS1;
 static LoRa_Packet GSE_GCS2;
 static LoRa_Packet GSE_Command;


 static Flash flash;
 //sensors*******************************************
 //do it for each sensor/converter
 //I2C
 static ADT75ARMZ temp_sensor;
 static MCP96RL00_EMX_1 thermocouple_1;
 static MCP96RL00_EMX_1 thermocouple_2;
 static MCP96RL00_EMX_1 thermocouple_3;
 static MCP96RL00_EMX_1 thermocouple_4;
 //SPI
 static ADC124S021 LoadCells;
 static ADC124S021 Transducers;

 //interrupt driven GPIO
 static GPIO LoRa_Rx_int;
 static GPIO temperature_alert;

 //normal GPIO*********************************************

 //relay actuations
 static GPIO CH1_OP;
 static GPIO CH1_ARM;
 static GPIO CH1_MON;
 static GPIO CH2_OP;
 static GPIO CH2_ARM;
 static GPIO CH2_MON;
 static GPIO CH3_OP;
 static GPIO CH3_ARM;
 static GPIO CH3_MON;
 /* For Future Expansion
 static GPIO CH4_OP;
 static GPIO CH4_ARM;
 static GPIO CH4_MON;*/

 //Ignition coil GPIO
 static GPIO Ignition1_ARM;
 static GPIO Ignition1_OP;
 static GPIO Ignition2_ARM;
 static GPIO Ignition2_OP;

 //LEDs for status visualisation
 static GPIO led_power;
 static GPIO led_local;
 static GPIO led_remote;
 static GPIO led_transducer;
 static GPIO led_n2o;
 static GPIO led_O2;

 //State activation/etc
 static GPIO activate_SW;
 static GPIO local_control_SW;
 static GPIO DUM_SW;  //turns out DUMP cannot be used as a variable name (DUMP_SW)
 static GPIO N2O_SW;
 static GPIO O2_IS_SW; //IS = ISO
 static GPIO N2O_IS_SW;
 static GPIO IGNITION_IS_SW;
 static GPIO N2O_DEAD_SW;
 static GPIO O2_DEAD_SW;
 static GPIO IGNITE_DEAD_SW;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


//if deadman switches are activated (or set)! -> TRIGGER PURGE
//is ISO switches are selected -> figure out what to do from there!
//if Other switch
//If local control switch is asserted, then the inputs from the other switches SHALL matter
	//If not remote control will take priority!


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  configureRCC_APB1();
  configureRCC_APB2();
  configureRCC_AHB1();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  configureSPIBus1();
  configureSPIBus6(); //SPI6
 // configureSPIBus4();
  configureI2CBus1();
//sensor configuration**********************************************
  //void ADT75ARMZ_init(ADT75ARMZ * i2c, I2C_TypeDef *interface, GPIO_TypeDef *port, Type TEMPERATURE_SENSOR, uint8_t address)



  //interrupt driven GPIO*******************************************

//Interrupt Mapped PD7 -> SX DIO0
 //This will be enabled by default Upon the reading of the local switch state
/*
  GPIO_init_interrupt(&temperature_alert, GPIOE, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_NO, 0x03);
  NVIC_SetPriority(EXTI4_IRQn,11); //setting temperature warning alert as highest priority!
  //NVIC_DisableIRQ(EXTI4_IRQn);
//Interrupt Mapped to PE4 ->Ambient temperature alert

  GPIO_init_interrupt(&DUM_SW, GPIOB, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x01);
  DUM_SW.port->ODR |= DUMP_SW;//IMMEDIATELY setting high
  //this GPIO interrupt was specifically made to operate on the falling edge!
  NVIC_SetPriority(EXTI1_IRQn,8);
  	  //PB1-> DUMP_SW (purge??)
*/
  //Normal GPIO initialisations*********************************************************
  //LEDs first
  	  	  //make the pullup/down selection be internal pulldowns to match external circuit
  GPIO_init(&led_power, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x00);
  	  //PG0 -> Power LED
  GPIO_init(&led_local, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x01);
  	  //PG1 -> local LED
  GPIO_init(&led_remote, GPIOE, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x07);
  	  //PE7 -> remote LED
  GPIO_init(&led_transducer, GPIOE, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x08);
	  //PE8 -> transducer LED
  GPIO_init(&led_n2o, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x05);
	  //PG5 -> nitrous LED
  GPIO_init(&led_O2, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x06);
  	 //PG6 -> nitrous LED
//THESE LEDS WILL BE SET ONCE AN ERROR OCCURS WITH THEIR RESPECTIVE FUNCTION

  //Control GPIO
  GPIO_init(&activate_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0F);
  	  //PF15-> Activate_sys SW

  GPIO_init(&local_control_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0E);
	  //PF14-> Local_control SW
  GPIO_init(&N2O_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0D);
  	  //PF13-> N20_ISO SW
  GPIO_init(&O2_IS_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0C);
   	  //PF12-> O2_ISO SW
  GPIO_init(&IGNITION_IS_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0B);
	  //PF11-> IGNITION_ISO SW
  GPIO_init(&N2O_DEAD_SW, GPIOB, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x00);
	  //PB0-> N2O_DEADMAN_SW
  GPIO_init(&O2_DEAD_SW, GPIOC, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x05);
  	  //PC5-> O2_DEADMAN_SW
  GPIO_init(&IGNITE_DEAD_SW, GPIOC, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x04);
	  //PC4-> IGNITE_DEADMAN_SW

  //Ignite actuation
  GPIO_init(&Ignition1_ARM, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0F);
  GPIO_init(&Ignition1_OP ,GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0E);
  GPIO_init(&Ignition2_ARM, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x03);
  GPIO_init(&Ignition2_OP ,GPIOG, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x02);

  //Relay Actuation
  //Relay 1 ->PURGE Relay
  GPIO_init(&CH1_OP, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x09);
  GPIO_init(&CH1_ARM, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0C);
  GPIO_init(&CH1_MON, GPIOB, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0F);
  //Relay 2 -> O2 Relay
  GPIO_init(&CH2_OP, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0D);
  GPIO_init(&CH2_ARM, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0E);
  GPIO_init(&CH2_MON, GPIOB, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0B);
  //Relay 3 -> N20 Relay
  GPIO_init(&CH3_OP, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x09);
  GPIO_init(&CH3_ARM, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0A);
  GPIO_init(&CH3_MON, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0D);
  //Relay 4 //for future expansion
  /*GPIO_init(&CH4_OP, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0B);
  GPIO_init(&CH4_ARM, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0C);
  GPIO_init(&CH4_MON, GPIOD, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x08);*/




  //sensor configuration
  ADT75ARMZ_init(&temp_sensor, I2C1, GPIOF,TEMPERATURE_SENSOR, 0x48);
  MCP96RL00_EMX_1_init(&thermocouple_1,I2C1, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, 0x60);
  MCP96RL00_EMX_1_init(&thermocouple_2,I2C1, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, 0x61);
  MCP96RL00_EMX_1_init(&thermocouple_3,I2C1, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, 0x62);
  MCP96RL00_EMX_1_init(&thermocouple_4,I2C1, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, 0x63);
  //conversion rate is 63ms for thermocouple conversion. Conversion will be 63ms + (time it takes to do 4 I2C read operations)
  ADC124S021_init(&LoadCells,Load_Cell, LOAD_CELL_PORT, LOAD_CELL_CS);
  ADC124S021_init(&Transducers,Transducer, TRANSDUCER_PORT, TRANSDUCER_CS);

  configure_TIM1(); //start LoRa timer -> as late as possible!
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/*	@Param:Error
 * 	B15: Ignition Failure
 * 	B14: Relay Error #3
 * 	B13: Relay Error #2
 *  B12: Relay Error #1
 *  B11: Thermocouple Error #4
 *  B10: Thermocouple Error #3
 *  B9: Thermocouple Error #2
 *  B8: Thermocouple Error #1
 *  B7: Load Cell Error #4
 *  B6: Load Cell Error #3
 *  B5: Load Cell Error #2
 *  B4: Load Cell Error #1
 *  B3: Transducer Error #4
 *  B2: Transducer Error #3
 *  B1: Transducer Error #2
 *  B0: Transducer Error #1
 *
 *  @Param: State
 *  B7: Manual Purge
 *  B6: O2 Fill Activate
 *  B5: Selector Switch Neutral Position
 *  B4: N20 Fill Activate
 *  B3: Ignition Fire
 *  B2: Ignition Selected
 *  B1: Gas Filled
 *  B0: System Activate -> nothing can be done unless this bit is set
 */
uint8_t LoRa_test_RX_Packets_received = 0;
uint8_t LoRa_Frequency_MSB = 0;
uint8_t LoRa_Frequency_MiB = 0;
uint8_t LoRa_Frequency_LSB = 0;

uint8_t RXDone_CHK = 0;

uint8_t DIOmapping = 0;


//INT_pin_input = (GPIOD->IDR & GPIO_IDR_IDR_7); //should be a non 0 value here!

//Make sure interrupts are configured BEFORE interupts
GPIO_init(&LoRa_Rx_int, GPIOD, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_NO, 0x07);
//NVIC_DisableIRQ(EXTI9_5_IRQn); //easier than changing the function GPIO_init_interrupt
SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI3_PD;
SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI3_PD;
EXTI->FTSR &= ~EXTI_FTSR_TR7_Msk;
EXTI->FTSR |= EXTI_FTSR_TR7;
EXTI->RTSR &= ~EXTI_RTSR_TR7_Msk;
EXTI->RTSR |= EXTI_RTSR_TR7;
EXTI->IMR &= ~EXTI_IMR_IM7;
EXTI->IMR |= EXTI_IMR_IM7;
					//here is channel for loRa PD7
//NVIC_EnableIRQ(EXTI9_5_IRQn);
//NVIC_SetPriority(EXTI9_5_IRQn,9);
//re-enable to turn on LoRa RX interrupt!

SX1272_init(&lora,"GSE_LORA", LORA_PORT, LORA_CS, SX1272_BW500, SX1272_SF9, SX1272_CR5);
SX1272_startReceive(&lora);
SX1272_clearIRQ(&lora, SX1272_LORA_IRQ_RXDONE);
//uint8_t DATAA = 0; -> for testing loRa
  while (1)
  {

	switch(switch_case_state){
	case 0:
		//This state is the default state -> reading sensor information and initial error checks!
		//test function
/*
		LoRa_test_RX_Packets_received = SX1272_readRegister(&lora, 0x13);
		LoRa_Frequency_MSB = SX1272_readRegister(&lora, 0x06);
		LoRa_Frequency_MiB = SX1272_readRegister(&lora, 0x07);
		LoRa_Frequency_LSB = SX1272_readRegister(&lora, 0x08);

		DIOmapping = SX1272_readRegister(&lora, 0x40);

		while (DATAA<10)
		{
			if(triggerInterrupt)
			{
				RXDone_CHK = SX1272_readRegister(&lora, 0x11);
				triggerInterrupt = false;
				DATAA = 0;
			}
			DATAA++;
		}
		*/


		if(triggerRX){RX_Receive();}else{__asm("NOP");}

		MCP96RL00_EMX_1_extract(&thermocouple_1, 0x60, data_thermo);
		MCP96RL00_EMX_1_process(&thermocouple_1);
		MCP96RL00_EMX_1_extract(&thermocouple_2, 0x61, data_thermo);//if variable data_thermo needs to be changed, make separate variables for each
		MCP96RL00_EMX_1_process(&thermocouple_2);
		MCP96RL00_EMX_1_extract(&thermocouple_3, 0x62, data_thermo);
		MCP96RL00_EMX_1_process(&thermocouple_3);
		MCP96RL00_EMX_1_extract(&thermocouple_4, 0x63, data_thermo);
		MCP96RL00_EMX_1_process(&thermocouple_4);
			//go into purge state IMMEDIATELY
			if(thermocouple_1.temperature >38){switch_case_state = 10; error |=(0x01<<11);}//error flag SPECIFICALLY for thermo couple 1
				else if(thermocouple_2.temperature >38){switch_case_state = 10; error |=(0x01<<10);}//error flag SPECIFICALLY for thermo couple 2
				else if(thermocouple_3.temperature >38){switch_case_state = 10; error |=(0x01<<9);}//error flag SPECIFICALLY for thermo couple 3
				else if(thermocouple_4.temperature >38){switch_case_state = 10; error |=(0x01<<8);}//error flag SPECIFICALLY for thermo couple 4
			else{
					if(thermocouple_1.temperature >=35){error |=(0x01<<11);}//set just error flag/no purge state
						else if(thermocouple_2.temperature >=35){error |=(0x01<<10);}
						else if(thermocouple_3.temperature >=35){error |=(0x01<<9);}
						else if(thermocouple_4.temperature >=35){error |=(0x01<<8);}
					else{} //make it so nothing happens here -> proceed
				}

//	ADC124S021_extract(&Transducers);
//	ADC124S021_process(&Transducers);
			if(triggerRX){RX_Receive();}else{}
	if(Transducers.Converted_Value_Transducer[0] >=68){switch_case_state = 10; error |=(0x01<<7);}//error flag SPECIFICALLY for Transducer1..etc
				else if(Transducers.Converted_Value_Transducer[1] >=68){switch_case_state = 10; error |=(0x01<<6); }//error flag SPECIFICALLY for Transducer2 check
				else if(Transducers.Converted_Value_Transducer[2] >=68){switch_case_state = 10; error |=(0x01<<5);}//error flag SPECIFICALLY for Transducer3 check
				else if(Transducers.Converted_Value_Transducer[3] >=68){switch_case_state = 10; error |=(0x01<<4);}//error flag SPECIFICALLY for Transducer4 check
			else{
					if(Transducers.Converted_Value_Transducer[0] >=65){error |=(0x01<<3);}//set just error flag/no purge state
						else if(Transducers.Converted_Value_Transducer[1] >=65){error |=(0x01<<2);}
						else if(Transducers.Converted_Value_Transducer[2] >=65){error |=(0x01<<1);}
						else if(Transducers.Converted_Value_Transducer[3] >=65){error |=0x01;}
					else{} //make it so nothing happens here -> proceed
				}
			if(triggerRX){RX_Receive();}else{__asm("NOP");}

//	ADC124S021_extract(&LoadCells);
	//ADC124S021_process(&LoadCells);
	if(LoadCells.Converted_Value_LoadCell[0] <4.1){error |=(0x01<<7);}//error flag SPECIFICALLY for Load1..etc
				else if(LoadCells.Converted_Value_LoadCell[1] <4.1){error |=(0x01<<6);}//error flag SPECIFICALLY for Load2 check
				else if(LoadCells.Converted_Value_LoadCell[2] <4.1){error |=(0x01<<5);}//error flag SPECIFICALLY for Load3 check
				else if(LoadCells.Converted_Value_LoadCell[3] <4.1){error |=(0x01<<4);}//error flag SPECIFICALLY for Load4 check
			else{}//nothing should happen here!}
	ADT75ARMZ_extract(&temp_sensor, data_thermo, 0x48);
	ADT75ARMZ_process(&temp_sensor);
	//no error checking is needed with internal temperature

if(switch_case_state == 10)
{
	break; //enter PURGE state
}
else
{
	//switch_case_state = 1; //input selector state
	break;
}

	/*
	 *State input selection -> Whether System inputs will be controlled via remote LoRa comms or local switch inputs
	 */

		case 1:
			if(triggerRX){RX_Receive();}else{__asm("NOP");}
				if((local_control_SW.port->IDR & LOCAL_CONTROL_SW) == 0) //meaning local control is needed
				{
					led_local.port->ODR |= (0x00 & LOCAL_CONTROL_SW)<<14;
					switch_case_state = 3; //remote control
					break;
				}
				else //If Local control is ON!
				{
					led_local.port->ODR |= LOCAL_CONTROL_SW;
					switch_case_state = 2; break; //local control
				}
		case 2:

			//@Param: local_state_input
			/*
			 * B7: Dump_SW
			 * B6: N20_ISO_SW
			 * B5: O2_ISO_SW
			 * B4: IGNITION_ISO_SW
			 * B3: N20_DEADMAN_SW
			 * B2: O2_DEADMAN_SW
			 * B1: IGNITION_DEAD_SW
			 * B0: Activate_SYS_SW (MUST BE 1) if changed at this point, sys must abort
			 */
			//Turn off GPIO and Timer interrupts

			TIM1->CR1 |= (0x00 & TIM_CR1_CEN); //enable TIM6
				while((TIM6->SR & TIM_SR_UIF)==0); //wait for hardware registers to be updated
				TIM1->SR &= ~(TIM_SR_UIF); //clears UIF register
			NVIC_DisableIRQ(EXTI9_5_IRQn);
			NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);

			state_local =
			((DUM_SW.port->IDR & DUMP_SW) << 7) +
			((N2O_SW.port->IDR & N2O_ISO_SW) << 6) +
			((O2_IS_SW.port->IDR & O2_ISO_SW) << 5) +
			((IGNITION_IS_SW.port->IDR & IGNITION_SW) << 4) +
			((N2O_DEAD_SW.port->IDR & N2O_DEADMAN_SW) << 3) +
			((O2_DEAD_SW.port->IDR & O2_DEADMAN_SW) << 2) + //to trigger -> high to
			((IGNITE_DEAD_SW.port->IDR & IGNITE_DEADMAN_SW) << 1) +
			(activate_SW.port->IDR & ACTIVATE_SW);

			switch_case_state = 0x0F; //change this to allow for local inputs
			break;
			//remote Access
		case 3:
			if(triggerRX){RX_Receive();}else{__asm("NOP");} //
			NVIC_EnableIRQ(EXTI9_5_IRQn);
			NVIC_SetPriority(EXTI9_5_IRQn,9);
			NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
			NVIC_SetPriority(TIM1_UP_TIM10_IRQn,10);
			//re-enable interrupt callbacks for LoRa
			switch_case_state = 4;
			/*@param: state
			 * B7: Manual Purge
			 * B6: O2 Fill
			 * B5: Switch Selector (redundant)
			 * B4: N2O fill
			 * B3: Ignition Fire
			 * B2: Ignition Selected
			 * B1: Gas Filled selected
			 * B0: System Activated
			 */

			break;
//**************************REMOTE ACCESS*************************************************************
		case 4:
			//Is the system on bit on at this point in TIME!
			if((state & SYSTEM_ACTIVATED) != SYSTEM_ACTIVATED)
						{
							switch_case_state =10; break;
						}
						else if((state & MANUAL_PURGE) == MANUAL_PURGE)
							{
									switch_case_state =10; break;	//When system is off, leave in PURGE state!
							}
//**********************Pass this point, the system is active, and manual purge has not been selected! *************************************

						else if((state & GAS_FILLED_SELECTED) == 0) //0 to indicate a successful -> skip this when bit 1 is a 1
											{
												if((state & IGNITION_SELECTED) == 0) //IGNITE SELECTED -> BIT2
												{
													switch_case_state = 9; //go into neutral state!
													break;
												}
												else //if bit 2 is a 1
												{
													//Close relays here!
													CH3_ARM.port->ODR &= ~(CH3_Arm);
													CH3_OP.port->ODR &= ~(CH3_Operate);
													led_n2o.port->ODR &= ~(N2O_LED);

													led_O2.port->ODR&= ~(O2_LED);
													CH2_ARM.port->ODR &= ~(CH2_Arm);
													CH2_OP.port->ODR &= ~(CH2_Operate);


													Ignition1_ARM.port->ODR |= IGNITION1_ARM;
													//Ignition2_ARM.port->ODR |= IGNITION2_ARM;
													//ARM both pins
													if((state & IGNITION_FIRE) != IGNITION_FIRE) //IGNITE OP -> BIT 3
													{ //if bit 3 is a 0
														switch_case_state = 9;
														break;
													}
													else
													{ //if bit 3 is a 1

														switch_case_state = 0x80; //to ensure accidential bit flips do not trigger fire stage
														break;
													}

												}

											}

//***********Pass this point, ignite is NOT selected, and gas fill state has been selected ************************************

						else if((state & SWITCH_SELECTOR) == SWITCH_SELECTOR)
										{
											//redundant so far but allow for future expansion
											__asm("NOP"); //do nothing so far!
										}
						//only O2 can be on and NOT N20
						else if((state & O2_FILL_ACTIVATE) == O2_FILL_ACTIVATE && (state & N2O_FILL_ACTIVATE) == 0x00)
						{
							if(state & (0x01<<1))//gas fill state needs to be on!
							{
								switch_case_state = 8;
								break;
							}
							else
							{
								switch_case_state = 11; //exclusive break to turn off O2 and N2O relays (NOT TOTAL neutral state)
								break;

							}
						}
			//N20 FILL
						else if((state & N2O_FILL_ACTIVATE) == N2O_FILL_ACTIVATE && (state & O2_FILL_ACTIVATE))
						{
							if(state &(0x01<<1))//gas fill state needs to be on!
								{
									switch_case_state = 7;
									break;
								}
							else
							{
								switch_case_state = 11; //exclusive break to turn off O2 and N2O relays (NOT TOTAL neutral state)
								break;
							}
						}
			//GAS SELECTED -> BIT 1
				else
				{
					//get error flag in here as well!
					switch_case_state = 0;
					break;
				}


//***************************END OF REMOTE ACCESS*****************************************************************************

//*******************************LOCAL ACCESS*************************************************************
		case 0x0F:
			if((state_local & ACTIVATE_SW) != ACTIVATE_SW) //if the system is not active
			{
				led_power.port->ODR &= ~(PWR_LED);
				switch_case_state = 10;
				break;
			}
			//Adding 'dump' flag to detect the change in interrupt state in key areas!
			//periodic 'dump' flag checks are needed along certain areas as to ensure state is checked regardless
				else if((state_local & IGNITION_SW) == IGNITION_SW && //is ignition ISO on
					(state_local & N2O_ISO_SW) != N2O_ISO_SW  && //is N2O off
					(state_local & O2_ISO_SW) != O2_ISO_SW && 	//is O2 ISO off
					(state_local & N2O_DEADMAN_SW) != N2O_DEADMAN_SW &&  //is N2O 'deadman' off
					(state_local & O2_DEADMAN_SW) != O2_DEADMAN_SW) //is O2 deadman off
				{
					if(dump_flag == 1){switch_case_state = 0; dump_flag = 0; break;}
					else{__asm("NOP");}
					//does nothing when false condition is set - ie when dump button has not been pressed!

					led_power.port->ODR |= PWR_LED;
					CH3_ARM.port->ODR &= ~(CH3_Arm);
					CH3_OP.port->ODR &= ~(CH3_Operate);
					led_n2o.port->ODR &= ~(N2O_LED);

					led_O2.port->ODR &= ~(O2_LED);
					CH2_ARM.port->ODR &= ~(CH2_Arm);
					CH2_OP.port->ODR &= ~(CH2_Operate);


					Ignition1_ARM.port->ODR |= IGNITION1_ARM;
					Ignition2_ARM.port->ODR |= IGNITION2_ARM;
					if((state_local & IGNITE_DEADMAN_SW) == IGNITE_DEADMAN_SW) //is IGNITE button pressed!
					{

						switch_case_state = 0x80;  //IGNITE state
						break;
					}
					else
					{
						switch_case_state = 9; //neutral state
						break;
					}
				}
			else if((state_local & N2O_ISO_SW) == N2O_ISO_SW &&
					(state_local & O2_ISO_SW) == O2_ISO_SW) //error check if both ISO switches are triggered for N20 and O2
			{
				if(dump_flag == 1){switch_case_state = 0; dump_flag = 0; break;}
				else{__asm("NOP");}

				//if local dump flag has been triggered REGARDLESS of input control state!

				led_O2.port->ODR &= ~(O2_LED);
				led_n2o.port->ODR &= ~(N2O_LED);
				switch_case_state = 10;
				break;
			}

			else if((state_local & N2O_ISO_SW) == N2O_ISO_SW &&
					(state_local & O2_ISO_SW) != O2_ISO_SW)
				{
				if(dump_flag == 1){switch_case_state = 0; dump_flag = 0; break;}
				else{__asm("NOP");}

					led_n2o.port->ODR |=N2O_LED;
					if((state_local& N2O_DEADMAN_SW) == N2O_DEADMAN_SW)
					{
						switch_case_state = 7;
						break;
					}
					else
					{
						led_n2o.port->ODR &= ~(N2O_LED);
					}
				}
			else if((state_local & O2_ISO_SW) == O2_ISO_SW &&
					(state_local & N2O_ISO_SW) != N2O_ISO_SW)
				{
				if(dump_flag == 1){switch_case_state = 0; dump_flag = 0; break;}
				else{__asm("NOP");}
					led_O2.port->ODR |= O2_LED;
					if((state_local & O2_DEADMAN_SW) == O2_DEADMAN_SW)
					{
						switch_case_state = 8;
						break;
					}
					else
					{
						led_O2.port->ODR &= ~(O2_LED);
					}
				}
		else
			{
				//If power is ON but other bits are off!
				switch_case_state = 0;
				break;
			}

//********************VARIOUS STATES/STAGES************************************************************************************

		case 7:
			//N20 Fill State
			if(CH3_MON.port->IDR != CH3_Cont)
			{
				led_n2o.port->ODR &= ~(N2O_LED);
				error = 0x01 << 14;
				switch_case_state = 0;
				break;
			}
			else{
				error = 0x00 << 14;
				led_n2o.port->ODR|=N2O_LED;
				CH3_ARM.port->ODR |= CH3_Arm;
				CH3_OP.port->ODR |= CH3_Operate;

				switch_case_state = 0;
				break;
			}

		case 8:
			//O2 FILL
			if(CH2_MON.port->IDR != CH2_Cont)
			{
				led_O2.port->ODR &= ~(O2_LED);
				error = 0x01 << 13;
				switch_case_state = 0;
				break;
			}
			else{
				error = 0x00 << 13; //unasserts error flag
				led_O2.port->ODR|=O2_LED;
				CH2_ARM.port->ODR |= CH2_Arm;
				CH2_OP.port->ODR |= CH2_Operate;

				switch_case_state = 0;
				break;
			}

		case 9:
			//neutral state
			//Turn OFF ignition coil relays
			Ignition1_ARM.port->ODR &= ~(IGNITION1_ARM);
		//	Ignition2_ARM.port->ODR &= ~(IGNITION2_ARM);
			Ignition1_OP.port->ODR &= ~(IGNITION1_OP);
		//	Ignition2_OP.port->ODR &= ~(IGNITION2_OP);


			//turn off relays purging gas! (gas can be filled)
			CH1_OP.port->ODR |= (CH1_Operate_OFF);
			CH1_ARM.port->ODR |= (CH1_Arm_OFF);


			switch_case_state = 0;
			break;

		case 10: //this will be the PURGE state
			//Presuming relay 1 is the DUMP relay!
			//PURGE is NO -> output a low to purge
			//output a high to stop purging!
			//__disable_irq(); //during purge state, no other interference can disturb the purge state!


			CH3_ARM.port->ODR &= ~(CH3_Arm);
			CH3_OP.port->ODR &= ~(CH3_Operate);
			led_n2o.port->ODR &= ~(N2O_LED);

			led_O2.port->ODR &= ~(O2_LED);
			CH2_ARM.port->ODR &= ~(CH2_Arm);
			CH2_OP.port->ODR &= ~(CH2_Operate);


			CH1_ARM.port->ODR &= CH1_Arm;
			CH1_OP.port->ODR &= CH1_Operate;
			//__enable_irq();
			//PURGE state will be left on UNTIL further notice as indicated by state var!
			switch_case_state = 0;
			break;

		case 11: //O2 -> N20 Shutoff state (when O2 and N2O state are triggered at the same time)
			CH3_ARM.port->ODR &= ~(CH3_Arm);
			CH3_OP.port->ODR &= ~(CH3_Operate);
			led_n2o.port->ODR &= ~(N2O_LED);

			led_O2.port->ODR &= ~(O2_LED);
			CH2_ARM.port->ODR &= ~(CH2_Arm);
			CH2_OP.port->ODR &= ~(CH2_Operate);

			switch_case_state = 0;
			break;

		case 0x80:
			__disable_irq();
			Ignition1_OP.port->ODR |= IGNITION1_OP;
		//	Ignition2_OP.port->ODR |= IGNITION2_OP;
			switch_case_state = 0;
			delay_software_ms(30); //provide a delay to ensure fire state has been activated for a long enough time
			Ignition1_OP.port->ODR &= ~(IGNITION1_OP);
			state &= ~(0x02 <<2); //this if more so for remote control 0bxxxx11xx become 0
			//turns of the ignite state once done!
			//state cannot be triggered more than once sequentially!
			__enable_irq();
			break;

		//FIRE STATE!!!!!

	}//switch case statement
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  } ///while(1)
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

   void TIM1_UP_TIM10_IRQHandler(void)
   {
		hardware_timer_count++;
		if(hardware_timer_count<5)
		{
			   //Hardware Timer interrupt callback for LoRa RX
				TIM1->SR &= ~(TIM_SR_UIF); //clears UIF register
		}
		else
		{
			CH3_ARM.port->ODR &= ~(CH3_Arm);
			CH3_OP.port->ODR &= ~(CH3_Operate);
			led_n2o.port->ODR &= ~(N2O_LED);

			led_O2.port->ODR &= ~(O2_LED);
			CH2_ARM.port->ODR &= ~(CH2_Arm);
			CH2_OP.port->ODR &= ~(CH2_Operate);


			CH1_ARM.port->ODR |= CH1_Arm;
			CH1_OP.port->ODR |= CH1_Operate;
			//PURGE state

			state &= ~0xFE; //0b11111110: all bits are bit-masked 0 except for system on
			hardware_timer_count = 0;
			TIM1->SR &= ~(TIM_SR_UIF); //clears UIF register
		}
   }

   void EXTI1_IRQHandler(void)
   {
	delay_software_us(200); //200us delay to prevent debouncing
   	if(EXTI->PR & EXTI_PR_PR1) //if the rising edge has been detected by pin 2
   	{
   		EXTI->PR &= ~EXTI_PR_PR1; //resets the flag
   		//Dump flag for local access!
   		//do something here
   		//equivalent of Purge Mode
   		if((local_control_SW.port->IDR & LOCAL_CONTROL_SW) == LOCAL_CONTROL_SW)
   		{
   			EXTI->PR &= ~EXTI_PR_PR1; //resets the flag
			CH3_ARM.port->ODR &= ~(CH3_Arm);
			CH3_OP.port->ODR &= ~(CH3_Operate);
			led_n2o.port->ODR &= ~(N2O_LED);

			led_O2.port->ODR &= ~(O2_LED);
			CH2_ARM.port->ODR &= ~(CH2_Arm);
			CH2_OP.port->ODR &= ~(CH2_Operate);

			CH1_ARM.port->ODR &= CH1_Arm;
			CH1_OP.port->ODR &= CH1_Operate;
			//__enable_irq();
			//PURGE state will be left on UNTIL further notice as indicated by state var!
			switch_case_state = 0; //will reset state machine if interrupt is triggered
			//might need to trigger a dump flag that restarts the main loop!
			dump_flag = 1; //tells program manual dump flag has been set!
   		}
   		else
   		{
   			EXTI->PR &= ~EXTI_PR_PR1; //resets the flag
   		 }
    	}
   }


  void EXTI3_IRQHandler(void)
  {
  	if(EXTI->PR & EXTI_PR_PR3) //if the rising edge has been detected by pin 2
  	{
  		EXTI->PR &= ~EXTI_PR_PR3; //resets the flag
  		//ambient temperature alert!
  		//do something here
  	}
  }

  void EXTI9_5_IRQHandler(void)
  {
//When this interrupt triggers -> this will indicate an RX packet has been received by the SX1272!
	  //Since data needs to be SENT from the GSE, that logic will need to be introduced via the state machine! not via interrupt callbacks!
	  /*What needs to be done during this interrupt handler/callback! -
	   *  1) Read the incoming FIFO buffer that has now appeared in the SX1272
	   *  2) Determine which packet has been received by the GSE itself (byte 1)
	   *  3) Store the FIFO data via
	   *  4) Change state variable
	   *  5) Proceed
	   */
	//  test_rx_interrupt++;
	 // uint8_t transmit_state = 0;
 	// SX1272_clearIRQ(&lora, SX1272_LORA_IRQ_RXDONE);
	 EXTI->PR &= ~0x1F0; //resets the flag
	 	triggerRX= true;


  }

void RX_Receive(void)
{
//	__disable_irq(); //uncomment after testing!!
	__NVIC_DisableIRQ(EXTI9_5_IRQn); //uncomment after testing!!
	bool RX_result = false;

	RX_result = SX1272_readReceive(&lora, pointerdata, LORA_MSG_LENGTH);
		triggerRX = false;
		GSE_Command.id= pointerdata[0];
		GSE_Command.data[0]= pointerdata[1];
		GSE_Command.data[1]= pointerdata[2];
		if(GSE_Command.id != 0x02)
		{
			lora_error = ERROR_INVALID_PACKET_ID;
			hardware_timer_count++;
			__asm("NOP");
			__NVIC_EnableIRQ(EXTI9_5_IRQn);

		}
		else if((GSE_Command.data[0] & 0x01) != 1) //ID is correct
	{
			//IF SYSTEM ON bit is 0
			state = 0; //make sure everything is OFF
			//make sure LED is OFF as well!
			led_power.port->ODR |= (PWR_LED & 0x00);
		}
		else //ID is correct AND state bit0 == 1
		{
			led_power.port->ODR |= PWR_LED; //Turn ON LED
			state = GSE_Command.data[0];
			hardware_timer_count = 0;
			uint8_t transmit_state = 0;
	        // Transmit response based on TX_Packet_Flag
			switch(TX_Packet_Flag)
			{
			case 0:
				LoRa_Packet packet_0 = LoRa_GSEData_1(0x06,
						&Transducers,
					&thermocouple_1,
					&thermocouple_2,
					&thermocouple_3,
					&thermocouple_4,
					error);

				SX1272_transmit(&lora, packet_0.data);
		  		do
		  		{
		  			transmit_state = SX1272_readRegister(&lora, SX1272_REG_IRQ_FLAGS);
		  		}while((transmit_state & 0x08) == 0x00); //will continue if transmit state and 0x08 are the same or 0
		  		//wait for Tx complete!


		  		  SX1272_writeRegister(&lora, SX1272_REG_IRQ_FLAGS, 0x08); //clears IRQ reg
		  		  TX_Packet_Flag = 1;
		  		_SX1272_setMode(&lora, SX1272_MODE_RXCONTINUOUS); //resetting flag back to RXCONTINUOUS mode after flag has been set!
				__NVIC_EnableIRQ(EXTI9_5_IRQn);


		  		break;


			case 1:

				LoRa_Packet packet_1 = LoRa_GSEData_2(0x07,
						&LoadCells,
					&temp_sensor,
					error);
  			SX1272_transmit(&lora, packet_1.data);

		  		do
		  		{ //continuously poll status register for TX complete!
		  			transmit_state = SX1272_readRegister(&lora, SX1272_REG_IRQ_FLAGS);
		  		}while((transmit_state & 0x08) == 0x00);

		  			SX1272_writeRegister(&lora, SX1272_REG_IRQ_FLAGS, 0x08); //clears IRQ reg
		  			TX_Packet_Flag = 0;
		  		_SX1272_setMode(&lora, SX1272_MODE_RXCONTINUOUS);
				__NVIC_EnableIRQ(EXTI9_5_IRQn);
		  		break;
		  		//the program should NEVER end up here
			default:
				lora_error = ERROR_SYSTEM_STATE_FAILED;
			}
		}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
