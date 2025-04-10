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
TIM_HandleTypeDef htim6;
I2C_HandleTypeDef hi2c2;
/* USER CODE BEGIN PV */

//These are here instead of main.h as I don't wanna go into different documents to remember definitions
uint8_t switch_case_state = 0; //global variable for state machine!
uint8_t state = 0;
uint8_t state_local = 0;
uint8_t local_dump_flag = 0;
static uint16_t error = 0;
uint8_t hardware_timer_count = 0;
uint8_t dump_flag = 0;
uint8_t TX_Packet_Flag = 0; //ID 7
volatile uint8_t data_thermo[2];
volatile uint8_t test_rx_interrupt =0;
uint8_t lora_error = 0; //mainly for lora comms error state
bool triggerRX = false;
uint8_t pointerdata[LORA_MSG_LENGTH];
uint8_t lora_error_test = 0;



uint8_t lora_spam_transmit_state = 0;


//Failure Mode Variables
int max_temp_failure_mode = 38.0;
int max_temp_error_mode = 35.0;

int max_pressure_failure_mode = 68.0;
int max_pressure_error_mode = 65.0;

int min_weight_error_mode = 4.1;



 	 //Error Code definitions here;
/*
 * ->


*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
//static void MX_TIM6_Init(void); -- NEED TO IMPLEMENT LORA PACKET TIMELOUT
/* USER CODE BEGIN PFP */

 static SX1272_t lora;
 static LoRa_Packet packet;
 static LoRa_Packet GSE_Command;

// For future expansion - flash storage
 //static Flash flash;

 //************************* sensors*******************************************
 //Currently commented out - internal functions to main.c and HAL libs used to date - JC 07-04-2025

 //do it for each sensor/converter
 //I2C
// static ADT75ARMZ temp_sensor;
 static MCP96RL00_EMX_1 thermocouple_1;
 static MCP96RL00_EMX_1 thermocouple_2;
 static MCP96RL00_EMX_1 thermocouple_3;
 static MCP96RL00_EMX_1 thermocouple_4;
 //SPI
 static ADC124S021 LoadCells;
 static ADC124S021 Transducers;


 //interrupt driven GPIO
 static GPIO LoRa_Rx_int;
 //For furture expansion - temp alerts by GPIO interupt
 //static GPIO temperature_alert;

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
 static GPIO CH4_OP;
 static GPIO CH4_ARM;
 static GPIO CH4_MON;
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
 static GPIO led_o2;


 //SMD LEDs
 static GPIO LED_1;
 static GPIO LED_2;
 static GPIO LED_3;
 static GPIO LED_4;
 static GPIO LED_5;
 static GPIO LED_6;

 //State activation/etc
 static GPIO activate_SW;
 static GPIO local_control_SW;
 static GPIO DUM_SW;  //turns out DUMP cannot be used as a variable name (DUMP_SW)
 static GPIO N2O_SW;
 static GPIO O2_IS_SW; //IS = ISO
 //static GPIO N2O_IS_SW;
 static GPIO IGNITION_IS_SW;
 static GPIO N2O_DEAD_SW;
 static GPIO O2_DEAD_SW;
 static GPIO IGNITE_DEAD_SW;





 static GPIO RF_SW;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//HAL LIB I2C COMMS SETUP - Added JC 07-04-2025
 typedef struct{
	 uint8_t ADDR;
	 uint8_t ID[1];
	 uint8_t resolution;
	 char thermocouple_type;
	 float temp;
 }TEMP_SENSE;

 typedef struct{
	 bool comms_ok;
	 uint8_t return_length;
	 uint8_t return_value[16];
 }i2c_comms_result;


 TEMP_SENSE SMD_TEMP_SENSE = {.ADDR = 0x48 << 1, .resolution = 0xC, .thermocouple_type = 0x00}; //Use 8-bit address;
 TEMP_SENSE THERMOCOUPLE_1 = {.ADDR = 0b11000000, .resolution = 0x12, .thermocouple_type = 'K'};
 TEMP_SENSE THERMOCOUPLE_2 = {.ADDR = 0b11000010, .resolution = 0x12, .thermocouple_type = 'J'};
 TEMP_SENSE THERMOCOUPLE_3 = {.ADDR = 0b11000100, .resolution = 0x12, .thermocouple_type = 'J'};
 TEMP_SENSE THERMOCOUPLE_4 = {.ADDR = 0b11000110, .resolution = 0x12, .thermocouple_type = 'J'};

 i2c_comms_result config_thermocouple();
 i2c_comms_result get_temp(TEMP_SENSE *temp_sense);

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
	delay_software_ms(100); //important!!

	MX_GPIO_Init();
	MX_I2C2_Init();
	//MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	configureSPIBus1();
	configureSPIBus6(); //SPI6
 // configureSPIBus4();
	//configureI2CBus1();

//*******************************NORMAL GPIO INITALISATIONS*************************************************************

	//-------------------- SMD LED GPIO --------------------------
  	//make the pullup/down selection be internal pulldowns to match external circuit
	//THESE LEDS WILL BE SET ONCE AN ERROR OCCURS WITH THEIR RESPECTIVE FUNCTION
	//PG0 -> Power LED
	GPIO_init(&LED_1, GPIOA, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_UP, 0x0C);
	//PG1 -> local LED
	GPIO_init(&LED_2, GPIOA, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_UP, 0x0B);
	//PE7 -> remote LED
	GPIO_init(&LED_3, GPIOA, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_UP, 0x0A);
	//PE8 -> transducer LED
	GPIO_init(&LED_4, GPIOA, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_UP, 0x09);
	//PG5 -> N2O LED
	GPIO_init(&LED_5, GPIOA, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_UP, 0x08);
	//PG6 -> O2 LED
	GPIO_init(&LED_6, GPIOC, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_UP, 0x09);

	//-------------------- LED GPIO --------------------------
  	//make the pullup/down selection be internal pulldowns to match external circuit
	//THESE LEDS WILL BE SET ONCE AN ERROR OCCURS WITH THEIR RESPECTIVE FUNCTION

	//PG0 -> Power LED
	GPIO_init(&led_power, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x00);
	//PG1 -> local LED
	GPIO_init(&led_local, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x01);
	//PE7 -> remote LED
	GPIO_init(&led_remote, GPIOE, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x07);
	//PE8 -> transducer LED
	GPIO_init(&led_transducer, GPIOE, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x08);
	//PG5 -> N2O LED
	GPIO_init(&led_n2o, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x05);
	//PG6 -> O2 LED
	GPIO_init(&led_o2, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x06);

	//-------------------- Control GPIO --------------------------

	//PF15-> Activate_sys SW
	GPIO_init(&activate_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0F);
	//PF14-> Local_control SW
	GPIO_init(&local_control_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0E);
	//PF13-> N20_ISO SW
	GPIO_init(&N2O_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0D);
	//PF12-> O2_ISO SW
	GPIO_init(&O2_IS_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0C);
	//PF11-> IGNITION_ISO SW
	GPIO_init(&IGNITION_IS_SW, GPIOF, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0B);
	//PB0-> N2O_DEADMAN_SW
	GPIO_init(&N2O_DEAD_SW, GPIOB, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x00);
	//PC5-> O2_DEADMAN_SW
	GPIO_init(&O2_DEAD_SW, GPIOC, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x05);
	//PC4-> IGNITE_DEADMAN_SW
	GPIO_init(&IGNITE_DEAD_SW, GPIOC, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x04);

	//-------------------- IGNITE ACTUATION GPIO --------------------------
	GPIO_init(&Ignition1_ARM, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0F);
	GPIO_init(&Ignition1_OP ,GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0E);



	GPIO_init(&Ignition2_ARM, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x03);
	GPIO_init(&Ignition2_OP ,GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x02);
	//Disarm Ignition circuit
	Ignition2_ARM.port->ODR |= (IGNITION2_ARM);
	Ignition2_OP.port->ODR |= (IGNITION2_OP);


	//-------------------- RELAY ACTUATION GPIO --------------------------
	//Relay 1 ->PURGE Relay
	GPIO_init(&CH1_OP, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0D);
	GPIO_init(&CH1_ARM, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN,  0x0E);
	GPIO_init(&CH1_MON, GPIOB, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN,  0x0B);

	//Relay 2 -> O2 Relay
	GPIO_init(&CH2_OP, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0A);
	GPIO_init(&CH2_ARM, GPIOB, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0C);
	GPIO_init(&CH2_MON, GPIOB, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0F);

	//Relay 3 -> N20 Relay
	GPIO_init(&CH3_OP, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0B);
	GPIO_init(&CH3_ARM, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0C);
	GPIO_init(&CH3_MON, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x08);

	//Relay 4 //for future expansion
	 GPIO_init(&CH4_OP, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x09);
	 GPIO_init(&CH4_ARM, GPIOD, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0A);
	 GPIO_init(&CH4_MON, GPIOD, GPIO_MODER_INPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_DOWN, 0x0D);



	 GPIO_init(&RF_SW, GPIOG, GPIO_MODER_GENERAL_PURPOSE_OUTPUT, GPIO_OTYPER_PUSH, GPIO_OSPEEDR_MEDIUM, GPIO_PUPDRy_NO, 0x0A);

	 RF_SW.port->ODR |= (GPIO_ODR_OD10);

	 //Ensure CH1-4 is turned off, as its currently unused
	 CH1_ARM.port->ODR &= ~(CH1_Arm);
	 CH1_OP.port->ODR &= ~(CH1_Operate);
	 CH2_ARM.port->ODR &= ~(CH2_Arm);
	 CH2_OP.port->ODR &= ~(CH2_Operate);
	 CH3_ARM.port->ODR &= ~(CH3_Arm);
	 CH3_OP.port->ODR &= ~(CH3_Operate);
	 CH4_ARM.port->ODR &= ~(CH4_Arm);
	 CH4_OP.port->ODR &= ~(CH4_Operate);

	//*******************************SENSOR CONFIG INITALISATIONS*************************************************************


	 //*******************************NOTE - Updated to use HAL libraries, and funcions within main.c*************************************************************
	 // Done by JC - 07/04/2025, for first legacy launch

	//ADT75ARMZ_init(&temp_sensor, I2C2, GPIOF,TEMPERATURE_SENSOR, 0x48);

	//conversion rate is 63ms for thermocouple conversion. Conversion will be 63ms + (time it takes to do 4 I2C read operations)
	//MCP96RL00_EMX_1_init(&thermocouple_1,I2C2, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, THERMOCOUPLE_1_ADDR);
	//MCP96RL00_EMX_1_init(&thermocouple_2,I2C2, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, THERMOCOUPLE_2_ADDR);
	//MCP96RL00_EMX_1_init(&thermocouple_3,I2C2, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, THERMOCOUPLE_3_ADDR);
	//MCP96RL00_EMX_1_init(&thermocouple_4,I2C2, GPIOF, THERMOCOUPLE, THERMO_SAMPLE_8, RESOLUTION_HIGH, THERMOCOUPLE_4_ADDR);

	//ADC124S021_init(&LoadCells,Load_Cell, LOAD_CELL_PORT, LOAD_CELL_CS);
	//ADC124S021_init(&Transducers,Transducer, TRANSDUCER_PORT, TRANSDUCER_CS);

	//configure_TIM1(); //start LoRa timer -> as late as possible!




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

	//Ensure SMD LEDs are turned OFF on bootup
	LED_1.port -> ODR &= ~LED_1_PWR;
	LED_2.port -> ODR &= ~LED_2_PWR;
	LED_3.port -> ODR &= ~LED_3_PWR;
	LED_4.port -> ODR &= ~LED_4_PWR;
	LED_5.port -> ODR &= ~LED_5_PWR;
	LED_6.port -> ODR &= ~LED_6_PWR;

	CH1_ARM.port->ODR &= ~(CH1_Arm);
	CH1_OP.port->ODR &= ~(CH1_Operate);


	CH2_ARM.port->ODR &= ~(CH2_Arm);
	CH2_OP.port->ODR &= ~(CH2_Operate);

	CH3_ARM.port->ODR &= ~(CH3_Arm);
	CH3_OP.port->ODR &= ~(CH3_Operate);

	CH4_ARM.port->ODR &= ~(CH4_Arm);
	CH4_OP.port->ODR &= ~(CH4_Operate);

	state = 0x00;


	RF_SW.port->ODR |= (GPIO_ODR_OD10);
	RF_SW.port->ODR &= ~(GPIO_ODR_OD10);
	RF_SW.port->ODR |= (GPIO_ODR_OD10);
	RF_SW.port->ODR &= ~(GPIO_ODR_OD10);
	RF_SW.port->ODR |= (GPIO_ODR_OD10);
	RF_SW.port->ODR &= ~(GPIO_ODR_OD10);

	//Debugging LoRa Step - move straight into spamming packets
	/*
	while(1){
		transmit_packets_spam();
	}
	*/



	switch_case_state = 0x0;
//Resetting Indentation from the start, cos I CBF changing the entire code - JC 0503025
while (1) {

	switch(switch_case_state){
//*******************************DEFAULT STATE*************************************************************
//								Read Sensor Data and Inital Error Checks
	case 0:

		__enable_irq();


		LED_1.port -> ODR |= LED_1_PWR;
		LED_2.port -> ODR |= LED_2_PWR;
		LED_3.port -> ODR |= LED_3_PWR;
		LED_4.port -> ODR |= LED_4_PWR;
		LED_5.port -> ODR |= LED_5_PWR;
		LED_6.port -> ODR |= LED_6_PWR;

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}

		//if variable data_thermo needs to be changed, make separate variables for each
		//Extract data from Thermocouple 1
	//	MCP96RL00_EMX_1_extract(&thermocouple_1, 0x60, data_thermo);
	//	MCP96RL00_EMX_1_process(&thermocouple_1);
		//Extract data from Thermocouple 2
		//MCP96RL00_EMX_1_extract(&thermocouple_2, 0x61, data_thermo);
		//MCP96RL00_EMX_1_process(&thermocouple_2);
		//Extract data from thermocouple 3
		//MCP96RL00_EMX_1_extract(&thermocouple_3, 0x62, data_thermo);
		//MCP96RL00_EMX_1_process(&thermocouple_3);
		//Extract data from thermocouple 4
		//MCP96RL00_EMX_1_extract(&thermocouple_4, 0x63, data_thermo);
		//MCP96RL00_EMX_1_process(&thermocouple_4);

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}

		//Check Thermocouple temps, if temps too high go directly to PURGE state
		//Error flags are specifc per Thermocouple
		if(thermocouple_1.temperature >max_temp_failure_mode){switch_case_state = 10; error |=(0x01<<11);}
		else if(thermocouple_2.temperature >max_temp_failure_mode){switch_case_state = 10; error |=(0x01<<10);}
		else if(thermocouple_3.temperature >max_temp_failure_mode){switch_case_state = 10; error |=(0x01<<9);}
		else if(thermocouple_4.temperature >max_temp_failure_mode){switch_case_state = 10; error |=(0x01<<8);}
		//If Error state but not direct to PURGE
		else{
			if(thermocouple_1.temperature >=max_temp_error_mode){error |=(0x01<<11);}
			else if(thermocouple_2.temperature >=max_temp_error_mode){error |=(0x01<<10);}
			else if(thermocouple_3.temperature >=max_temp_error_mode){error |=(0x01<<9);}
			else if(thermocouple_4.temperature >=max_temp_error_mode){error |=(0x01<<8);}
		//Temps are A-O.K, so carry on without doing anything
			else{}
		}

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}

		//Extract Transducer Pressures
		//ADC124S021_extract(&Transducers);
		//uint16_t response_test = ADC124S021_ReadChannel(0);
		//ADC124S021_process(&Transducers);

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{}

		//Check Transducer pressures, if pressures too high go directly to PURGE state
		//Error flags are specifc per Transducer
		if(Transducers.Converted_Value_Transducer[0] >=max_pressure_failure_mode){switch_case_state = 10; error |=(0x01<<7);}
		else if(Transducers.Converted_Value_Transducer[1] >=max_pressure_failure_mode){switch_case_state = 10; error |=(0x01<<6); }
		else if(Transducers.Converted_Value_Transducer[2] >=max_pressure_failure_mode){switch_case_state = 10; error |=(0x01<<5);}
		else if(Transducers.Converted_Value_Transducer[3] >=max_pressure_failure_mode){switch_case_state = 10; error |=(0x01<<4);}
		//If Error state but not direct to PURGE
		else{
			if(Transducers.Converted_Value_Transducer[0] >=max_pressure_error_mode){error |=(0x01<<3);}
			else if(Transducers.Converted_Value_Transducer[1] >=max_pressure_error_mode){error |=(0x01<<2);}
			else if(Transducers.Converted_Value_Transducer[2] >=max_pressure_error_mode){error |=(0x01<<1);}
			else if(Transducers.Converted_Value_Transducer[3] >=max_pressure_error_mode){error |=0x01;}
		//Pressures are A-OK, so carry on without doing anything
			else{} //make it so nothing happens here -> proceed
		}

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}


		//Extract Load Cell Weights
		//ADC124S021_extract(&LoadCells);
		//ADC124S021_process(&LoadCells);

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}


		//Check Loadcell weights, if too low, trigger error flag
		//Error Flags specific to loadcell
		if(LoadCells.Converted_Value_LoadCell[0] <min_weight_error_mode){error |=(0x01<<7);}
		else if(LoadCells.Converted_Value_LoadCell[1] <min_weight_error_mode){error |=(0x01<<6);}
		else if(LoadCells.Converted_Value_LoadCell[2] <min_weight_error_mode){error |=(0x01<<5);}
		else if(LoadCells.Converted_Value_LoadCell[3] <min_weight_error_mode){error |=(0x01<<4);}
		//Weights are A-OK, so carry on without doing anything
		else{}

		//Extract surface mount temp sensor temp
		//no error checking currently implemented with internal temperature
	//	ADT75ARMZ_extract(&temp_sensor, data_thermo, 0x48);
	//	ADT75ARMZ_process(&temp_sensor);

		i2c_comms_result result = get_temp(&SMD_TEMP_SENSE);
		if (result.comms_ok){
			for (uint8_t i = 0; i <= result.return_length; i++) {
				uint8_t * floatPtr = (uint8_t *) &SMD_TEMP_SENSE.temp;
				floatPtr[i] = result.return_value[i];
			}
		}
		else {
			SMD_TEMP_SENSE.temp = 0x00;
		}

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}

		//Check if we need to go directly to PURGE
		if(switch_case_state == 10)
		{
			break; //enter PURGE state
		}
		else
		{
			switch_case_state = 1; //input selector state
			break;
		}


//*******************************LOCAL/REMOTE INPUT SELECTION STATE*************************************************************
//								Checks switch input and sets LED outputs

	case 1:

		//Set SMD LEDs as per current case/state
		LED_1.port->ODR &= ~LED_1_PWR;
		LED_2.port -> ODR |= LED_2_PWR;
		LED_3.port -> ODR &= ~LED_3_PWR;
		LED_4.port -> ODR &= ~LED_4_PWR;
		LED_5.port -> ODR &= ~LED_5_PWR;
		LED_6.port -> ODR &= ~LED_6_PWR;

			//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}

			//check for either remote control (== 0) or local control (== 1)
		if((local_control_SW.port->IDR & LOCAL_CONTROL_SW) == 0)
		{
				//Change LED State
			led_remote.port->ODR |= LOCAL_CONTROL_SW;			//Set Remote CTRL LED
			led_local.port->ODR &= ~(LOCAL_CONTROL_SW);			//Off Local CTRL LED
			//Change switch case state
			switch_case_state = 3; //remote control
			break;
		}
		else //Local Control (==1)
		{
			//Change LED State
			led_remote.port->ODR &= ~(LOCAL_CONTROL_SW);		//Off Remote CTRL LED
			led_local.port->ODR |= LOCAL_CONTROL_SW;			//Set Local CTRL LED
			//Change switch case state
			switch_case_state = 2; //local control
			break;
		}

//*******************************LOCAL ACCESS - TIMER AND INPUTS UPDATE *************************************************************
//								Sets Timer 1 and reads in switch inputs
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

		__disable_irq();

		//Read in switch inputs to state_local var
		state_local =
		((DUM_SW.port->IDR & DUMP_SW) << 7) +
		((N2O_SW.port->IDR & N2O_ISO_SW) << 6) +
		((O2_IS_SW.port->IDR & O2_ISO_SW) << 5) +
		((IGNITION_IS_SW.port->IDR & IGNITION_SW) << 4) +
		((N2O_DEAD_SW.port->IDR & N2O_DEADMAN_SW) << 3) +
		((O2_DEAD_SW.port->IDR & O2_DEADMAN_SW) << 2) + //to trigger -> high to
		((IGNITE_DEAD_SW.port->IDR & IGNITE_DEADMAN_SW) << 1) +
		(activate_SW.port->IDR & ACTIVATE_SW);

		//Move to local control output setting state
		switch_case_state = 0x0F;
		break;


//*******************************REMOTE ACCESS - TIMER UPDATES*************************************************************
//								Enable Interrupts
	case 3:

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}

		//Enable interrupts for LoRa
		NVIC_EnableIRQ(EXTI9_5_IRQn);
		NVIC_SetPriority(EXTI9_5_IRQn,9);
		//NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		//NVIC_SetPriority(TIM1_UP_TIM10_IRQn,10);

		//Move to remote access output setting state
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
//**************************REMOTE ACCESS - OUTPUT ACTUATION*************************************************************
//							Actuates outputs depended on state flags within last received LoRa packet
	case 4:

		LED_1.port -> ODR &= ~LED_1_PWR;
		LED_2.port -> ODR &= ~LED_2_PWR;
		LED_3.port -> ODR |= LED_3_PWR;
		LED_4.port -> ODR &= ~LED_4_PWR;
		LED_5.port -> ODR &= ~LED_5_PWR;
		LED_6.port -> ODR &= ~LED_6_PWR;

		//Check if SX1272 has recieved a packet, if not move on
		if(triggerRX){RX_Receive();}else{__asm("NOP");}

		//Check if System activated bit was set in last received packet
		if((state & SYSTEM_ACTIVATED) != SYSTEM_ACTIVATED)
		{
			//If system is not active, should go straight to PURGE state
			switch_case_state = 10;
			break;
		}

		//If system is active, ensure purge doesnt PURGE
		//if((state & SYSTEM_ACTIVATED) == SYSTEM_ACTIVATED){
		//	CH1_ARM.port->ODR |= (CH1_Arm);
		//	CH1_OP.port->ODR |= (CH1_Operate);
		//}

		//Check if manual purge bit was set in last received packet
		else if( ((state & MANUAL_PURGE) == MANUAL_PURGE) && ((state & GAS_FILLED_SELECTED) == GAS_FILLED_SELECTED) )
		{
			//If manual purge was set, should go straight to PURGE state
			switch_case_state = 10;
			break;
		}
// ------------------- Pass this point, the system is active, and manual purge has not been selected! -------------------

		//If neither Gas or Ignition selected, go to neutral state
		else if( ((state & GAS_FILLED_SELECTED) == 0) && ((state & IGNITION_SELECTED) == 0) ) //0 to indicate neither are selected
		{
			switch_case_state = 9; //go into neutral state!
			break;
		}

		//If Ignition is selected
		else if ( ((state & IGNITION_SELECTED) == IGNITION_SELECTED) && ((state & GAS_FILLED_SELECTED) != GAS_FILLED_SELECTED))
		{
			//Ensure N2O Solenoid is closed, and turn off LED
			CH3_ARM.port->ODR &= ~(CH3_Arm);
			CH3_OP.port->ODR &= ~(CH3_Operate);
			led_n2o.port->ODR &= ~(N2O_LED);


			//If Ignition FIRE bit is set, time to light this puppy
			if((state & IGNITION_FIRE) == IGNITION_FIRE)
			{
				switch_case_state = 0x80;
				break;
			}
			//If O2 Fill bit is set, enable the O2 Solenoids
			else if((state & O2_FILL_ACTIVATE) == O2_FILL_ACTIVATE){
				switch_case_state = 8;
				break;
			}
			//Otherwise go to neutral
			else{
				switch_case_state = 9;
				break;
			}
		}

// ------------------- Pass this point, ignite is NOT selected, and gas fill state has been selected -------------------

		else if ( ((state & IGNITION_SELECTED) != IGNITION_SELECTED) && ((state & GAS_FILLED_SELECTED) == GAS_FILLED_SELECTED))
		{
//N2O and PURGE have not been selected, go to neutral state
			if((state & SWITCH_SELECTOR) == SWITCH_SELECTOR)
			{
				switch_case_state = 9;
				break;
			}
//N20 Fill is selected, move to N2O actuation state
			else if( (state & N2O_FILL_ACTIVATE) == N2O_FILL_ACTIVATE)
			{
				switch_case_state = 7;
				break;
			}
//If neither are selected, some error... Go to neutral state for safety, and set error flag
			else
			{
				//get error flag in here as well!
				switch_case_state = 9; //Something has gone wrong and weird - go to neutral state and stop gas flow
				break;
			}
		}


//***************************END OF REMOTE ACCESS*****************************************************************************

//*******************************LOCAL ACCESS*************************************************************
	case 0x0F:


		LED_1.port -> ODR |= LED_1_PWR;
		LED_2.port -> ODR &= ~LED_2_PWR;
		LED_3.port -> ODR &= ~LED_3_PWR;
		LED_4.port -> ODR &= ~LED_4_PWR;
		LED_5.port -> ODR &= ~LED_5_PWR;
		LED_6.port -> ODR |= LED_6_PWR;


		if((state_local & ACTIVATE_SW) != ACTIVATE_SW) //if the system is not active
		{
			switch_case_state = 10;
			break;
		}

//Ignition Key Swtich Selected
		else if((state_local & IGNITION_SW) == IGNITION_SW && //is ignition ISO on
				(state_local & N2O_ISO_SW) != N2O_ISO_SW  && //is N2O off
				(state_local & O2_ISO_SW) != O2_ISO_SW && 	//is O2 ISO off
				(state_local & N2O_DEADMAN_SW) != N2O_DEADMAN_SW &&  //is N2O 'deadman' off
				(state_local & O2_DEADMAN_SW) != O2_DEADMAN_SW) //is O2 deadman off
		{

			//Adding 'dump' flag to detect the change in interrupt state in key areas!
			//periodic 'dump' flag checks are needed along certain areas as to ensure state is checked regardless
			if(dump_flag == 1){switch_case_state = 10; dump_flag = 0; break;}
			else{__asm("NOP");}
			//does nothing when false condition is set - ie when dump button has not been pressed!
			CH3_ARM.port->ODR &= ~(CH3_Arm);
			CH3_OP.port->ODR &= ~(CH3_Operate);
			led_n2o.port->ODR &= ~(N2O_LED);
			led_o2.port->ODR &= ~(O2_LED);
			CH2_ARM.port->ODR &= ~(CH2_Arm);
			CH2_OP.port->ODR &= ~(CH2_Operate);


			Ignition1_ARM.port->ODR |= IGNITION1_ARM;
			Ignition2_ARM.port->ODR |= IGNITION2_ARM;
//Ignition Rotary Switch Selected
			if((state_local & IGNITE_DEADMAN_SW) == IGNITE_DEADMAN_SW) //is IGNITE button pressed!
			{
				switch_case_state = 0x80;  //IGNITE state
				break;
			}
//Ignition Key Switch but NO Ignition Rotary Switch
			else
			{
				switch_case_state = 9; //neutral state
				break;
			}
		}


//Both Keyswitches for N2O and O2 are triggered
		else if((state_local & N2O_ISO_SW) == N2O_ISO_SW &&
				(state_local & O2_ISO_SW) == O2_ISO_SW) //error check if both ISO switches are triggered for N20 and O2
		{
			if(dump_flag == 1){switch_case_state = 10; dump_flag = 0; break;}
			else{__asm("NOP");}

			//if local dump flag has been triggered REGARDLESS of input control state!
			switch_case_state = 10;
			break;
		}


//N2O Keyswitch
		else if((state_local & N2O_ISO_SW) == N2O_ISO_SW &&
				(state_local & O2_ISO_SW) != O2_ISO_SW)
		{

			if(dump_flag == 1){switch_case_state = 10; dump_flag = 0; break;}
			else{__asm("NOP");}

//N2O Rotary Switch
			if((state_local& N2O_DEADMAN_SW) == N2O_DEADMAN_SW)
			{
				switch_case_state = 7; // N2O Fill State
				break;
			}
//N2O Keyswitch but NO N2O Rotary Switch
			else
			{
				switch_case_state = 9; //neutral state
				break;
			}
		}


//O2 Keyswitch
		else if((state_local & O2_ISO_SW) == O2_ISO_SW &&
				(state_local & N2O_ISO_SW) != N2O_ISO_SW)
		{

			if(dump_flag == 1){switch_case_state = 10; dump_flag = 0; break;}
			else{__asm("NOP");}

//O2 Rotary Switch
			if((state_local & O2_DEADMAN_SW) == O2_DEADMAN_SW)
			{
				switch_case_state = 8;
				break;
			}
//O2 Keyswitch but NO O2 Rotary Switch
			else
			{
				switch_case_state = 9; //neutral state
				break;
			}
		}

//If power is ON but other bits are off
		else
		{
			switch_case_state = 9; // Neutral state
			break;
		}

//********************VARIOUS STATES/STAGES************************************************************************************


//**************************N2O FILL *************************************************************
	case 7:

		//Set SMD LEDs
		LED_1.port -> ODR &= ~LED_1_PWR;
		LED_2.port -> ODR &= ~LED_2_PWR;
		LED_3.port -> ODR &= ~LED_3_PWR;
		LED_4.port -> ODR |= LED_4_PWR;
		LED_5.port -> ODR &= ~LED_5_PWR;
		LED_6.port -> ODR &= ~LED_6_PWR;

		//Clear Error
		error = 0x00 << 14;

		//Ensure Purge is not purging
		CH1_ARM.port->ODR |= (CH1_Arm);
		CH1_OP.port->ODR |= (CH1_Operate);

		//Ensure O2 Fill is not filling
		led_o2.port->ODR &= ~(O2_LED);
		CH2_ARM.port->ODR &= ~(CH2_Arm);
		CH2_OP.port->ODR &= ~(CH2_Operate);

		//Ensure igntion is not igniting
		Ignition1_ARM.port->ODR &= ~(IGNITION1_ARM);
		Ignition1_OP.port->ODR &= ~(IGNITION1_OP);

		//Enable N2O Fill
		led_n2o.port->ODR|=N2O_LED;
		CH3_ARM.port->ODR |= CH3_Arm;
		CH3_OP.port->ODR |= CH3_Operate;
		switch_case_state = 0;
		break;


		/*
		//Check feedback Opto
		if(CH3_MON.port->IDR != CH3_Cont)
		{
			//No Continuity on the feedback opto, set error flag, and go to neutral state
			led_n2o.port->ODR &= ~(N2O_LED);
			error = 0x01 << 14;
			switch_case_state = 9;
			break;
		}
		else
		{
			//Clear Error
			error = 0x00 << 14;

			//Ensure Purge is not purging
			CH1_ARM.port->ODR |= (CH1_Arm);
			CH1_OP.port->ODR |= (CH1_Operate);

			//Ensure O2 Fill is not filling
			led_o2.port->ODR &= ~(O2_LED);
			CH2_ARM.port->ODR &= ~(CH2_Arm);
			CH2_OP.port->ODR &= ~(CH2_Operate);

			//Ensure igntion is not igniting
			Ignition1_ARM.port->ODR &= ~(IGNITION1_ARM);
			Ignition1_OP.port->ODR &= ~(IGNITION1_OP);

			//Enable N2O Fill
			led_n2o.port->ODR|=N2O_LED;
			CH3_ARM.port->ODR |= CH3_Arm;
			CH3_OP.port->ODR |= CH3_Operate;

			switch_case_state = 0;
			break;
		}
		*/


//**************************O2 FILL*************************************************************
	case 8:

		//Set SMD LEDs
		LED_1.port -> ODR &= ~LED_1_PWR;
		LED_2.port -> ODR &= ~LED_2_PWR;
		LED_3.port -> ODR &= ~LED_3_PWR;
		LED_4.port -> ODR &= ~LED_4_PWR;
		LED_5.port -> ODR &= ~LED_5_PWR;
		LED_6.port -> ODR |= LED_6_PWR;

		//Clear Error
		error = 0x00 << 13;

		//Ensure Purge is not purging
		CH1_ARM.port->ODR |= (CH1_Arm);
		CH1_OP.port->ODR |= (CH1_Operate);

		//Ensure N2O Fill is not filling
		led_n2o.port->ODR &= ~(N2O_LED);
		CH3_ARM.port->ODR &= ~(CH3_Arm);
		CH3_OP.port->ODR &= ~(CH3_Operate);

		//Enable O2 to flow
		led_o2.port->ODR|=O2_LED;
		CH2_ARM.port->ODR |= CH2_Arm;
		CH2_OP.port->ODR |= CH2_Operate;


		switch_case_state = 0;
		break;

		/*
		//Check Feedback Opto
		if(CH2_MON.port->IDR != CH2_Cont)
		{
			//No Continutiy on feedback opto, set error flag, and go to neutral state
			led_o2.port->ODR &= ~(O2_LED);
			error = 0x01 << 13;
			switch_case_state = 9;
			break;
		}
		else
		{
			//Clear Error
			error = 0x00 << 13;

			//Ensure Purge is not purging
			CH1_ARM.port->ODR |= (CH1_Arm);
			CH1_OP.port->ODR |= (CH1_Operate);

			//Ensure N2O Fill is not filling
			led_n2o.port->ODR &= ~(N2O_LED);
			CH3_ARM.port->ODR &= ~(CH3_Arm);
			CH3_OP.port->ODR &= ~(CH3_Operate);

			//Enable O2 to flow
			led_o2.port->ODR|=O2_LED;
			CH2_ARM.port->ODR |= CH2_Arm;
			CH2_OP.port->ODR |= CH2_Operate;

			switch_case_state = 0;
			break;
		}
		*/


//**************************NEUTRAL STATE*************************************************************
	case 9:

		//Set SMD LEDs
		LED_1.port -> ODR |= LED_1_PWR;
		LED_2.port -> ODR |= LED_2_PWR;
		LED_3.port -> ODR |= LED_3_PWR;
		LED_4.port -> ODR &= ~LED_4_PWR;
		LED_5.port -> ODR &= ~LED_5_PWR;
		LED_6.port -> ODR &= ~LED_6_PWR;


		//Turn OFF ignition coil relays
		Ignition1_ARM.port->ODR &= ~(IGNITION1_ARM);
		Ignition2_ARM.port->ODR &= ~(IGNITION2_ARM);
		Ignition1_OP.port->ODR &= ~(IGNITION1_OP);
		Ignition2_OP.port->ODR &= ~(IGNITION2_OP);


		//Ensure PURGE is not purging
		CH1_OP.port->ODR |= (CH1_Operate);
		CH1_ARM.port->ODR |= (CH1_Arm);

		//Ensure N2O is not filling
		CH3_ARM.port->ODR &= ~(CH3_Arm);
		CH3_OP.port->ODR &= ~(CH3_Operate);
		led_n2o.port->ODR &= ~(N2O_LED);

		//Ensure O2 is not filling
		led_o2.port->ODR &= ~(O2_LED);
		CH2_ARM.port->ODR &= ~(CH2_Arm);
		CH2_OP.port->ODR &= ~(CH2_Operate);


		switch_case_state = 0;
		break;


//**************************PURGE STATE*************************************************************
	case 10:

		//Set SMD LEDs
		LED_1.port -> ODR |= LED_1_PWR;
		LED_2.port -> ODR |= LED_2_PWR;
		LED_3.port -> ODR |= LED_3_PWR;
		LED_4.port -> ODR |= LED_4_PWR;
		LED_5.port -> ODR |= LED_5_PWR;
		LED_6.port -> ODR |= LED_6_PWR;

		//Presuming relay 1 is the DUMP relay!
		//PURGE is NO -> output a low to purge
		//output a high to stop purging!

		//Ensure Ignition is not igniting
		Ignition1_ARM.port->ODR &= ~(IGNITION2_ARM);
		Ignition1_OP.port->ODR &= ~(IGNITION2_OP);

		//Turn off N2O Solenoid and turn off LED
		CH3_ARM.port->ODR &= ~(CH3_Arm);
		CH3_OP.port->ODR &= ~(CH3_Operate);
		led_n2o.port->ODR &= ~(N2O_LED);

		//Turn off O2 Solenoid and turn off LED
		led_o2.port->ODR &= ~(O2_LED);
		CH2_ARM.port->ODR &= ~(CH2_Arm);
		CH2_OP.port->ODR &= ~(CH2_Operate);

		//Power off PURGE solenoid therefore starting purge
		CH1_ARM.port->ODR &= ~(CH1_Arm);
		CH1_OP.port->ODR &= ~(CH1_Operate);

		switch_case_state = 0;
		break;


//********************IGNITION************************************************************************************

	case 0x80:

		//Disable IRQs - we are igniting, nothing can stop this....
		__disable_irq();

		//Set SMD LEDs
		LED_1.port -> ODR &= ~LED_1_PWR;
		LED_2.port -> ODR &= ~LED_2_PWR;
		LED_3.port -> ODR &= ~LED_3_PWR;
		LED_4.port -> ODR |= LED_4_PWR;
		LED_5.port -> ODR |= LED_5_PWR;
		LED_6.port -> ODR |= LED_6_PWR;

		//Check O2 Fill status - as we check for igniton first in case 4 - we need to check O2 again to ensure that we actuate correctly
		if((state & O2_FILL_ACTIVATE) == O2_FILL_ACTIVATE){
			//Turn ON O2 Solenoid and turn on LED
			led_o2.port->ODR |= (O2_LED);
			CH2_ARM.port->ODR |= (CH2_Arm);
			CH2_OP.port->ODR |= (CH2_Operate);
		}
		else {
			//Turn off O2 Solenoid and turn off LED
			led_o2.port->ODR &= ~(O2_LED);
			CH2_ARM.port->ODR &= ~(CH2_Arm);
			CH2_OP.port->ODR &= ~(CH2_Operate);
		}

		//Ensure N2O LED are OFF
		led_n2o.port->ODR &= ~(N2O_LED);

		//Ensure N2O solenoid is OFF
		CH3_ARM.port->ODR &= ~(CH3_Arm);
		CH3_OP.port->ODR &= ~(CH3_Operate);

		//Ensure the DUMP solenoid is not dumping
		CH1_ARM.port->ODR |= (CH1_Arm);
		CH1_OP.port->ODR |= (CH1_Operate);


		//Spark Generation Sequence, 5 sparks total
	//	Ignition1_ARM.port->ODR |= IGNITION1_ARM;
		//Ignition1_OP.port->ODR |= IGNITION1_OP;
		Ignition2_ARM.port->ODR |= (IGNITION2_ARM);
		Ignition2_OP.port->ODR |= (IGNITION2_OP);
		delay_software_ms(30); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR &= ~(IGNITION2_OP);
		Ignition2_ARM.port->ODR &= ~(IGNITION2_ARM);
		delay_software_ms(30); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR |= (IGNITION2_OP);
		Ignition2_ARM.port->ODR |= (IGNITION2_ARM);
		delay_software_ms(500); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR &= ~(IGNITION2_OP);
		Ignition2_ARM.port->ODR &= ~(IGNITION2_ARM);
		delay_software_ms(30); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR |= (IGNITION2_OP);
		Ignition2_ARM.port->ODR |= (IGNITION2_ARM);
		delay_software_ms(500); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR &= ~(IGNITION2_OP);
		Ignition2_ARM.port->ODR &= ~(IGNITION2_ARM);
		delay_software_ms(30); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR |= (IGNITION2_OP);
		Ignition2_ARM.port->ODR |= (IGNITION2_ARM);
		delay_software_ms(500); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR &= ~(IGNITION2_OP);
		Ignition2_ARM.port->ODR &= ~(IGNITION2_ARM);
		delay_software_ms(30); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR |= (IGNITION2_OP);
		Ignition2_ARM.port->ODR |= (IGNITION2_ARM);
		delay_software_ms(500); //provide a delay to ensure fire state has been activated for a long enough time
		Ignition2_OP.port->ODR &= ~(IGNITION2_OP);
		Ignition2_ARM.port->ODR &= ~(IGNITION2_ARM);
		delay_software_ms(30); //provide a delay to ensure fire state has been activated for a long enough time

		//Disarm Ignition circuit
		Ignition2_ARM.port->ODR |= (IGNITION2_ARM);
		Ignition2_OP.port->ODR |= (IGNITION2_OP);

		delay_software_ms(500); //provide a delay to ensure fire state has been activated for a long enough time

		//Manually removes "ignition" state bit from last read LoRa packet info
		state &= ~(0x02 <<2); //this if more so for remote control 0bxxxx11xx become 0
		switch_case_state = 0;
		//turns of the ignite state once done!
		//state cannot be triggered more than once sequentially!
		__enable_irq();
		break;


		}//switch case statement
	} ///close bracket for while(1)
} // close bracket for main()



/**
  * @brief System Clock Configuration
  * @retval None
 **/
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

  // Initializes the CPU, AHB and APB buses clocks
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
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

static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}



  /** Configure Digital filter
  */


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
		led_o2.port->ODR &= ~(O2_LED);
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
   		if((local_control_SW.port->IDR & LOCAL_CONTROL_SW) == LOCAL_CONTROL_SW)
   		{
   			EXTI->PR &= ~EXTI_PR_PR1; //resets the flag
			dump_flag = 1; //tells program manual dump flag has been set!
   		}
   		else
   		{
   			EXTI->PR &= ~EXTI_PR_PR1; //resets the flag
   		}
   	}
}


//ambient temperature alert!
void EXTI3_IRQHandler(void)
{
  	if(EXTI->PR & EXTI_PR_PR3) //if the rising edge has been detected by pin 2
  	{
  		EXTI->PR &= ~EXTI_PR_PR3; //resets the flag
  		//do something here
  	}
}



// LoRa DIO Interrupt
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
	__NVIC_DisableIRQ(EXTI9_5_IRQn); //uncomment after testing!!

}

void RX_Receive(void)
{
	__disable_irq(); //uncomment after testing!!
	__NVIC_DisableIRQ(EXTI9_5_IRQn); //uncomment after testing!!
	__NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn); //Disable IQR for LoRa Hardware Timer

	delay_software_ms(100); //important!!


	bool RX_result = SX1272_readReceive(&lora, pointerdata, LORA_MSG_LENGTH);
	triggerRX = false;
	GSE_Command.id= pointerdata[0];
	GSE_Command.data[0]= pointerdata[1];
	GSE_Command.data[1]= pointerdata[2];


	if(GSE_Command.id != 0x02)
	{
		lora_error = ERROR_INVALID_PACKET_ID;
		hardware_timer_count++;
		__asm("NOP");
		//__NVIC_EnableIRQ(EXTI9_5_IRQn);


	}
	// Double Checks Valid Data
	else if ((GSE_Command.data[0] & GSE_Command.data[1]) == 0x00)
	{
		led_power.port->ODR |= PWR_LED; //Turn ON LED
		lora_error_test++;
		//state = GSE_Command.id; //temporary value
		state = GSE_Command.data[0];
		hardware_timer_count = 0;
		uint8_t transmit_state = 0;
		//__NVIC_EnableIRQ(EXTI9_5_IRQn);

		//Transmit response based on TX_Packet_Flag
		// to test RX and state change validation with RX command
		switch(TX_Packet_Flag)
		{
		case 0:
				packet = Dummy_Transmit();
				packet.id = 0x06;

				packet.data[0] = 0x00;
				packet.data[1] = 0x00;
				packet.data[2] = 0x00;
				packet.data[3] = 0x00;
				packet.data[4] = 0x00;
				packet.data[5] = 0x00;
				packet.data[6] = 0x00;
				packet.data[7] = 0x00;
				packet.data[8] = 0x00;
				packet.data[9] = 0x00;
				packet.data[10] = 0x00;
				packet.data[11] = 0x00;
				packet.data[12] = 0x00;
				packet.data[13] = 0x00;
				packet.data[14] = 0x00;
				packet.data[15] = 0x00;
				packet.data[16] = 0x00;
				packet.data[17] = 0x00;
				packet.data[18] = 0x00;
				packet.data[19] = 0x00;
				packet.data[20] = 0x00;
				packet.data[21] = 0x00;
				packet.data[22] = 0x00;
				packet.data[23] = 0x00;
				packet.data[24] = 0x00;
				packet.data[25] = 0x00;
				packet.data[26] = 0x00;
				packet.data[27] = 0x00;
				packet.data[28] = 0x00;
				packet.data[29] = 0x00;
				packet.data[30] = 0x00;

					/*
					 LoRa_Packet packet_0 = LoRa_GSEData_1(0x06,
							&Transducers,
							&thermocouple_1,
							&thermocouple_2,
							&thermocouple_3,
							&thermocouple_4,
							error);
					*/
			  	TX_Packet_Flag = 1;

				break;
		case 1:

				packet = Dummy_Transmit();
				packet.id = 0x07;


				packet.data[0] = GSE_Command.data[0];
				uint8_t * floatPtr = (uint8_t *) &SMD_TEMP_SENSE.temp;
				packet.data[1] = floatPtr[3];
				packet.data[2] = floatPtr[2];
				packet.data[3] = floatPtr[1];
				packet.data[4] = floatPtr[0];

				packet.data[5] = 0x00;
				packet.data[6] = 0x00;
				packet.data[7] = 0x00;
				packet.data[8] = 0x00;
				packet.data[9] = 0x00;
				packet.data[10] = 0x00;
				packet.data[11] = 0x00;
				packet.data[12] = 0x00;
				packet.data[13] = 0x00;
				packet.data[14] = 0x00;
				packet.data[15] = 0x00;
				packet.data[16] = 0x00;
				packet.data[17] = 0x00;
				packet.data[18] = 0x00;
				packet.data[19] = 0x00;
				packet.data[20] = 0x00;
				packet.data[21] = 0x00;
				packet.data[22] = 0x00;
				packet.data[23] = 0x00;
				packet.data[24] = 0x00;
				packet.data[25] = 0x00;
				packet.data[26] = 0x00;
				packet.data[27] = 0x00;
				packet.data[28] = 0x00;
				packet.data[29] = 0x00;
				packet.data[30] = 0x00;
				/*
				LoRa_Packet packet_1 = LoRa_GSEData_2(0x07,
					&LoadCells,
					&temp_sensor,
					error);
				*/
			  	TX_Packet_Flag = 0;

				break;
		//the program should NEVER end up here
		default:
				lora_error = ERROR_SYSTEM_STATE_FAILED;
		} //P2 (end)

		RF_SW.port->ODR &= ~(GPIO_ODR_OD10);
		//Transmit the packet!
		SX1272_transmit(&lora, (uint8_t*) &packet);
	  	do
	  	{
	  		transmit_state = SX1272_readRegister(&lora, SX1272_REG_IRQ_FLAGS);
	  	}while((transmit_state & 0x08) == 0x00); //will continue if transmit state and 0x08 are the same or 0
	  	//wait for Tx complete!

	  	RF_SW.port->ODR |= (GPIO_ODR_OD10);

	  	SX1272_writeRegister(&lora, SX1272_REG_IRQ_FLAGS, 0x08); //clears IRQ reg
	  	_SX1272_setMode(&lora, SX1272_MODE_RXCONTINUOUS); //resetting flag back to RXCONTINUOUS mode after flag has been set!
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
		//__NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	}


	else
	{
		lora_error = ERROR_INVALID_PACKET_DATA;
		hardware_timer_count++;
		__asm("NOP");
		//__NVIC_EnableIRQ(EXTI9_5_IRQn);
	}


}

void transmit_packets_spam()
{
	uint8_t lora_spam_transmit = 0;

	static LoRa_Packet packet0;
	packet0.data[0] = 0x00;
	packet0.data[1] = 0x01;
	packet0.data[2] = 0x02;
	packet0.data[3] = 0x03;
	packet0.data[4] = 0x04;
	packet0.data[5] = 0x05;
	switch(lora_spam_transmit_state)
	{
	case 0:
		packet0.id = 0x06;
		RF_SW.port->ODR |= (GPIO_ODR_OD10);
		SX1272_transmit(&lora, (uint8_t * ) &packet0);
		do
		{
			lora_spam_transmit = SX1272_readRegister(&lora, SX1272_REG_IRQ_FLAGS);
		}while((lora_spam_transmit & 0x08) == 0x00); //will continue if transmit state and 0x08 are the same or 0
	  	SX1272_writeRegister(&lora, SX1272_REG_IRQ_FLAGS, 0x08); //clears IRQ reg
	  	TX_Packet_Flag = 0;
	  	_SX1272_setMode(&lora, SX1272_MODE_RXCONTINUOUS);
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
		//__NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

		RF_SW.port->ODR &=  ~(GPIO_ODR_OD10);
		lora_spam_transmit_state = 1;
	  	break;
	case 1:
		packet0.id = 0x07;
		RF_SW.port->ODR |= (GPIO_ODR_OD10);
		SX1272_transmit(&lora, (uint8_t * ) &packet0);
		do
		{
			lora_spam_transmit = SX1272_readRegister(&lora, SX1272_REG_IRQ_FLAGS);
		}while((lora_spam_transmit & 0x08) == 0x00); //will continue if transmit state and 0x08 are the same or 0
	  	SX1272_writeRegister(&lora, SX1272_REG_IRQ_FLAGS, 0x08); //clears IRQ reg
	  	TX_Packet_Flag = 0;
	  	_SX1272_setMode(&lora, SX1272_MODE_RXCONTINUOUS);
		__NVIC_EnableIRQ(EXTI9_5_IRQn);
	//	__NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
		lora_spam_transmit_state = 0;
		RF_SW.port->ODR &= ~(GPIO_ODR_OD10);
	  	break;
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
    {__asm("NOP");}
  /* USER CODE END Error_Handler_Debug */
}






i2c_comms_result get_temp(TEMP_SENSE *temp_sense){

	i2c_comms_result result;

	//If its not a thermocouple, its the SMD temp sense
	if(temp_sense -> thermocouple_type == 0x00){

		uint8_t buf[4];
		buf[0] = 0x00;
		uint8_t ret;
		ret = HAL_I2C_Master_Transmit(&hi2c2, temp_sense -> ADDR, buf[0], 1, HAL_MAX_DELAY);
		if (ret != HAL_OK){
			  result.comms_ok = false;
		}
		else {
			  ret = HAL_I2C_Master_Receive(&hi2c2, temp_sense -> ADDR, result.return_value, 2, HAL_MAX_DELAY);

			  if (ret != HAL_OK){
				  result.comms_ok = false;
			  }
			  else{

				  uint16_t val = ((int16_t)result.return_value[0]<<4) | (result.return_value[1] >> 4);
				  float temp = val/16;
				  uint8_t * tempPointer = (uint8_t *) &temp;
				  result.return_value[0] = tempPointer[0];
				  result.return_value[1] = tempPointer[1];
				  result.return_value[2] = tempPointer[2];
				  result.return_value[3] = tempPointer[3];
				  result.return_length = 0x04;

				  result.comms_ok = true;
			  }
		}

	}

	//Its got a thermocouple type, therefore, lets get the thermocouple hot junc temp
	else{
		uint8_t ret;
		ret = HAL_I2C_Master_Transmit(&hi2c2, (temp_sense -> ADDR | 0x00), 0x00, 1, 100); //Write to thermocouple IC, to move pointer to hot junc reg
		if (ret != HAL_OK){
			result.comms_ok = false;
		}
		else {
			ret = HAL_I2C_Master_Receive(&hi2c2, (temp_sense -> ADDR | 0x01), result.return_value, 2, 100);	//Read 2 bytes from the hot junc reg into return val
			if (ret != HAL_OK){
				result.comms_ok = false;
			}
			else{

				uint16_t val = ((int16_t)result.return_value[0] * 16) | (result.return_value[1]);
				float temp = val/16;
				if ((result.return_value[0] & 0x80) == 0x80){ //If the temp is < 0deg
					temp = temp - 4096;
				}
				uint8_t * tempPointer = (uint8_t *) &temp;
				result.return_value[0] = tempPointer[0];
				result.return_value[1] = tempPointer[1];
				result.return_value[2] = tempPointer[2];
				result.return_value[3] = tempPointer[3];
				result.return_length = 0x04;

				result.comms_ok = true;
			}
		}
	}


	return result;

};

i2c_comms_result config_thermocouple(TEMP_SENSE *temp_sense){
	i2c_comms_result result;





	return result;
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
