/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include "math.h"
#include "l6470.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef uint8_t crc_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFER_LENGTH 30
#define WIDTH  (8 * sizeof(crc_t))
#define TOPBIT (1 << (WIDTH - 1))


#define MOTOR_DIR 		((MOTOR_STATUS_RAW_DATA&(1<<4))>>4)
#define MOTOR_STATUS 	((MOTOR_STATUS_RAW_DATA&(3<<5))>>5)
#define MOTOR_BUSY_FLAG 	((MOTOR_STATUS_RAW_DATA&(1<<1))>>1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 const char start_string[] = "UART STARTED" ;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

crc_t POLYNOMIAL = 0xcb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
//static void Spi_transmit(uint8_t byte);
//static void Spi_recieve(uint8_t byte[],uint8_t size);
void send_on_rs485(uint8_t* buffer);
void recieve_on_rs485(uint8_t buffer[],size_t buffer_length);

crc_t CRC_generate(crc_t* message, crc_t polynomial, int nBytes );

uint8_t* message_packet_with_crc(crc_t* message2, crc_t polynomial, int nBytes );

crc_t CRC_CHECK_decode(crc_t* message, crc_t polynomial, int nBytes );

void UNL_freeride();

void MAN_freeride();

void MAN_Videoloop();

void UNL_Animation();

void MAN_Animation();

void Limit_setting();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t MOTOR_STATUS_RAW_DATA=0;
long int ps=0;
unsigned long read=0,read2=0;
//uint8_t rc_data[1]={0};
//uint8_t buffer[14];
//char buffer1[9]="hello01\r\n";
//uint8_t* send_buffer="h-l6470\r\n";
//uint8_t send_buffer[11] = {0x55,'-','l','6','4','7','0','\r','\n',0x00,0x00};
uint8_t send_buffer[BUFFER_LENGTH] = {0x55,0x02,0x01,0x01,0xf4,0x00,'0','\r','\n',0x00,0x55};
uint8_t Previous_buffer[BUFFER_LENGTH];
uint8_t RECIEVE_VALID_DATA[BUFFER_LENGTH];
uint8_t recieve_buffer[BUFFER_LENGTH];

uint8_t RS_485_Data_validate=0;
//int BUFFER_LENGTH=20;
uint8_t state_of_rs485=1;
uint8_t counter=0;
HAL_StatusTypeDef uart_state;
int len=0;
uint8_t DAMPING =145,PREVIOUS_DAMPING=145;
uint16_t SPEED=0;
uint16_t sp=0;
uint8_t sp1=0;
uint8_t sp2=0;
uint16_t x=0;
uint32_t y=0,z=0;
int32_t s=0;
uint8_t count = 0;
int step=0;
 long TOTAL_MICROSTEP=0;
 extern long CURRENT_POSITION;
 long LIMIT_M = 0x7FFFFFFF,check=0;
 uint32_t id=0;
 uint8_t dir=3;
 _Bool toggle=0;
 _Bool busy_flag=0;
 _Bool movement_is_pending=0;

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
#define self_test 0
#define LIMIT 6000
  //send_on_rs485(send_buffer);
  recieve_on_rs485(recieve_buffer,BUFFER_LENGTH);

  Spi_init();
//HAL_TIM_Base_Start_IT(&htim4);
  HAL_Delay(1000);
 // id=  DBGMCU->IDCODE;


  	HARD_HIZ();
    GET_STATUS();

	Set_param(L6470_ACC, 		DAMPING, L6470_ACC_LEN); // min damping 0x91 //max damping 0x0a //ffa
	Set_param(L6470_DEC, 		DAMPING, L6470_DEC_LEN);
	Set_param(L6470_MAX_SPEED, 	0x015e, L6470_MAX_SPEED_LEN);//3ff
	Set_param(L6470_MIN_SPEED, 	0x00, L6470_MIN_SPEED_LEN);
	Set_param(L6470_FS_SPD, 	0x3ff, L6470_FS_SPD_LEN);
	Set_param(L6470_KVAL_HOLD, 	0x34, L6470_KVAL_HOLD_LEN);
	Set_param(L6470_KVAL_RUN, 	0x34, L6470_KVAL_RUN_LEN);
	Set_param(L6470_KVAL_ACC, 	0x34, L6470_KVAL_ACC_LEN);
	Set_param(L6470_KVAL_DEC,	0x34, L6470_KVAL_DEC_LEN);
	Set_param(L6470_INT_SPD, 	0x0408, L6470_INT_SPD_LEN);
	Set_param(L6470_ST_SLP, 	0x19, L6470_ST_SLP_LEN);
	Set_param(L6470_FN_SLP_ACC, 0x29, L6470_FN_SLP_ACC_LEN);
	Set_param(L6470_FN_SLP_DEC, 0x29, L6470_FN_SLP_DEC_LEN);
	Set_param(L6470_K_THERM, 	0x00, L6470_K_THERM_LEN);
	Set_param(L6470_OCD_TH, 	0x08, L6470_OCD_TH_LEN);
	Set_param(L6470_STALL_TH, 	0x61, L6470_STALL_TH_LEN);
	Set_param(L6470_ALARM_EN, 	0xFF, L6470_ALARM_EN_LEN);
	Set_param(L6470_CONFIG_A, 	0x2E88, L6470_CONFIG_A_LEN);
	RESET_POS();

	//GO_UNTIL(ACT0, FORWARD_DIR,step_s_2_Speed(8000) );//15 min //1550 max
//	Move(FORWARD_DIR,200*128);
//	HAL_Delay(4000);
//
//	GO_TO(0xfffff);
#if self_test
	Set_param(L6470_MAX_SPEED, 	0x66, L6470_MAX_SPEED_LEN);// 1550 steps/sec or 0x65
	Set_param(L6470_MIN_SPEED, 	0x3f, L6470_MIN_SPEED_LEN);//15 steps/sec or 0x3f

	Run(FORWARD_DIR, step_s_2_Speed(1550));
//	GO_TO(4100000);
	//GO_TO_DIR(0, 3194000);
//	Move(FORWARD_DIR,4194303);//
#endif
	//HAL_Delay(4000);
	//Set_param(SOFT_STOP_CMD,0, 8);



	//Move(FORWARD_DIR,4194303);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);


#if !self_test
	  switch (RECIEVE_VALID_DATA[6])
	  	  {
	  		  case 'U': /*unlimited mode*/
	  			  switch (RECIEVE_VALID_DATA[7])
	  			  {
	  			  case 'C': // calibration mode


	  				  break;
	  			  case 'F':  // freeride
	  				  UNL_freeride();
	  				  break;
	  			  case 'A':   //animation
	  				 UNL_Animation();
	  				  break;
	  			  case 'T':  // timelapse

	  				  break;
	  			  case 'R':  // recording

	  				  break;

	  			  case 'P':  // playback

	  				  break;

	  			  case 'V':  // videoloop

	  				  break;
	  			  case 'G':  // general
	  				  SOFT_STOP();
	  				  LIMIT_M=0x7FFFFFFF;

	  				  break;
	  			  case 'L':  // limit setting

	  				  LIMIT_M=0x7FFFFFFF;
	  				  break;

	  			  }
	  			  break;
	  		  case 'M': /*Manual mode*/
	  			  switch (RECIEVE_VALID_DATA[7])
	  			  {
	  			  case 'C': // calibration mode
	  				  UNL_freeride();
	  				  break;
	  			  case 'F':  // freeride
	  				  MAN_freeride();
	  				  break;
	  			  case 'A':   //animation
	  				 MAN_Animation();
	  				  break;
	  			  case 'T':  // timelapse

	  				  break;
	  			  case 'R':  // recording

	  				  break;

	  			  case 'P':  // playback

	  				  break;

	  			  case 'V':  // videoloop

	  				 MAN_Videoloop();

	  				  break;
	  			  case 'G':  // general
	  				SOFT_STOP();

	  				  break;
	  			  case 'L':  // limit setting
	  				  Limit_setting();
	  				  break;

	  			  }
	  		  	  break;
	  		  default :
	  		  	  break;

	  	  }








	  DAMPING = RECIEVE_VALID_DATA[3];
//	  Set_param(L6470_ACC, 		DAMPING, L6470_ACC_LEN); // min damping 0x91 //max damping 0x0a //ffa
//	  Set_param(L6470_DEC, 		DAMPING, L6470_DEC_LEN);
	  if(DAMPING != PREVIOUS_DAMPING)
	  {
//		  SOFT_STOP();
//		  HAL_Delay(100);
		 if(MOTOR_STATUS==0){
		  Set_param(L6470_ACC, 		DAMPING, L6470_ACC_LEN); // min damping 0x91 //max damping 0x0a //ffa
		  Set_param(L6470_DEC, 		DAMPING, L6470_DEC_LEN);
		  PREVIOUS_DAMPING = DAMPING;
		 }
	  }

	  SPEED = RECIEVE_VALID_DATA[2];


//	   sp1 = RECIEVE_VALID_DATA[4];
//	  		  sp2=RECIEVE_VALID_DATA[5];
//	  		  sp=(sp1<<8)|sp2;
//	  		  sp=((RECIEVE_VALID_DATA[4]<<8)|RECIEVE_VALID_DATA[5]);
//	  HAL_Delay(20);
//
//	  		if(RECIEVE_VALID_DATA[7]== 'L')
//	  		{
//
//				CURRENT_POSITION =  RECIEVE_VALID_DATA[8] << 56;
//				CURRENT_POSITION |= RECIEVE_VALID_DATA[9] << 48;
//				CURRENT_POSITION |= RECIEVE_VALID_DATA[10] << 40;
//				CURRENT_POSITION |= RECIEVE_VALID_DATA[11] << 32;
//				CURRENT_POSITION |= RECIEVE_VALID_DATA[12] << 24;
//				CURRENT_POSITION |= RECIEVE_VALID_DATA[13] << 16;
//				CURRENT_POSITION |= RECIEVE_VALID_DATA[14] << 8;
//				CURRENT_POSITION |= RECIEVE_VALID_DATA[15];
//
//				LIMIT_M=CURRENT_POSITION;
//	  		}


//	  if(RECIEVE_VALID_DATA[1]== 'F')
//	  {
//		 // dir=1;
////		   sp = RECIEVE_VALID_DATA[4];
////		  sp=sp<<8;
////		  sp|=RECIEVE_VALID_DATA[5];
//
////		  if((TOTAL_MICROSTEP+s)<LIMIT)
////		  	{
////		Run(FORWARD_DIR, step_s_2_Speed(sp));
////		  	}
//		//  Run(FORWARD_DIR, step_s_2_Speed(15));
////		  check = TOTAL_MICROSTEP+s;
//
//		  if((TOTAL_MICROSTEP+s)<LIMIT_M){
//		 Run(FORWARD_DIR, step_s_2_Speed(sp));
//		  }
//
//		step+=256;
//	  }
//	  else if(RECIEVE_VALID_DATA[1]== 'B')
//	  {
//		//  dir=2;
////		   sp = RECIEVE_VALID_DATA[4];
////		  sp=sp<<8;
////		  sp|=RECIEVE_VALID_DATA[5];
////		  if((TOTAL_MICROSTEP+s)LIMIT)
////		  	{
//		 // Run(BACKWARD_DIR, step_s_2_Speed(15));
//		//  if((TOTAL_MICROSTEP-s)>0){
//		  Run(BACKWARD_DIR, step_s_2_Speed(sp));
//		//  }
////	  }
//		  step-=256;
//	  }
//
//	  else if(RECIEVE_VALID_DATA[1]== 'S')
//	 	  {
//		 // dir=3;
//		 // RESET_POS();
//		  	  SOFT_STOP();
////	 		  uint16_t sp = RECIEVE_VALID_DATA[2];
////	 		  sp=sp<<8;
////	 		  sp|=RECIEVE_VALID_DATA[3];
////	 		  Run(BACKWARD_DIR, step_s_2_Speed(sp));
//	 	  }
	  	  	  while(state_of_rs485!=1);
	  	  	  send_buffer[1]=0x01;
	  	  	  send_buffer[2]=(TOTAL_MICROSTEP>>56)&0xff;
			  send_buffer[3]=(TOTAL_MICROSTEP>>48)&0xff;
			  send_buffer[4]=(TOTAL_MICROSTEP>>40)&0xff;
			  send_buffer[5]=(TOTAL_MICROSTEP>>32)&0xff;
			  send_buffer[6]=(TOTAL_MICROSTEP>>24)&0xff;
			  send_buffer[7]=(TOTAL_MICROSTEP>>16)&0xff;
			  send_buffer[8]=(TOTAL_MICROSTEP>>8)&0xff;
			  send_buffer[9]=(TOTAL_MICROSTEP)&0xff;
			  send_buffer[16]=(y>>8)&0xff;
			  send_buffer[17]=(y)&0xff;
			  send_buffer[18]=(z>>8)&0xff;
			  send_buffer[19]=(z)&0xff;
#endif



//	  while(state_of_rs485!=1);
//	  if(state_of_rs485==1){send_on_rs485(send_buffer);}
	    //HAL_UART_Transmit(&huart1,(uint8_t*)start_string, sizeof(start_string), 100);
//	  GO_TO(0x00);HAL_Delay(100);HARD_STOP();HAL_Delay(1000);
//	  GO_TO(0x1900); HAL_Delay(100);HARD_STOP();//HAL_Delay(200);

//	  Run(FORWARD_DIR, 0xfffff);HAL_Delay(200);HARD_STOP();//HAL_Delay(200);
//	  Run(BACKWARD_DIR, 0xfffff);HAL_Delay(200);HARD_STOP();//HAL_Delay(200);

	//Move(BACKWARD_DIR,200*128);HAL_Delay(2000);
//	  Run(FORWARD_DIR, step_s_2_Speed(4000));HAL_Delay(500);SOFT_STOP();HAL_Delay(500);
//	  Run(BACKWARD_DIR, step_s_2_Speed(4000));HAL_Delay(500);SOFT_STOP();HAL_Delay(500);
	 // HAL_Delay(500);
//	 while(sp<6000){
//	  GET_STATUS();sp++;}
//	 sp=0;
	 MOTOR_STATUS_RAW_DATA = GET_STATUS();
	 y=Speed_2_step_s(GET_SPEED());
	 z=ACC_DEC_2_step_s(GET_DEC());
	 x=MOTOR_STATUS;
	 //x=MOTOR_STATUS&(1<<4);
//	 if(MOTOR_DIR==FORWARD_DIR){dir=FORWARD_DIR;}
//	 else if(MOTOR_DIR==BACKWARD_DIR){dir=BACKWARD_DIR;}
	// x=MOTOR_STATUS;
//	 y=MOTOR_DIR;
//	 x=MOTOR_DIR;
	// if(MOTOR_DIR!=x){dir=3;RESET_POS();}
//	 if(count>=1){
//	 		 HARD_STOP();
//	 	 				 x=MOTOR_DIR;
//
//	 	 			 }
	 ps=Get_param(L6470_ABS_POS);
	 busy_flag=MOTOR_BUSY_FLAG;
	 TOTAL_MICROSTEP = Read_total_steps(MOTOR_DIR,MOTOR_STATUS);

//	 if(TOTAL_MICROSTEP>1000 && busy_flag==1)
//	 {
//		 if(x != 0)
//		 {   SOFT_STOP();
//			 Run(BACKWARD_DIR, 0);
//		 }
//		 else if (x==0 && toggle==0)
//		 {
//			 toggle=1;
//			// RESET_POS();
////			 GO_TO_DIR(FORWARD_DIR,25600);
//			 Move(FORWARD_DIR,256000);
////			 while(1)
////			 {
////				 TOTAL_MICROSTEP = Read_total_steps(MOTOR_DIR,MOTOR_STATUS);
////			 }
//		 }
//	 }

#if !self_test
	// if(y>15){
//	 if(RECIEVE_VALID_DATA[7] != 'C')
//	 {
		s= ( ( (y*y) ) / (2*z) )/2;


//		// }
//		if(MOTOR_DIR==FORWARD_DIR)
//		{
//
//	if(((long)(TOTAL_MICROSTEP+s))>LIMIT_M)
//	{
//		SOFT_STOP();
//		if(TOTAL_MICROSTEP<LIMIT_M){
//		Run(FORWARD_DIR, step_s_2_Speed(20));
//		}
//		else if(TOTAL_MICROSTEP>=LIMIT_M)
//		{
//		Run(BACKWARD_DIR, step_s_2_Speed(0));
//		}
//
//	}
//		}
//		 if(MOTOR_DIR==BACKWARD_DIR){
//			if((TOTAL_MICROSTEP-s)<0)
//			{
//				SOFT_STOP();
//				if(TOTAL_MICROSTEP > 0){
//				Run(BACKWARD_DIR, step_s_2_Speed(20));}
//
//			}
//				}

//	 }
//	if((TOTAL_MICROSTEP)<LIMIT)
//	{
//
//		Run(FORWARD_DIR, step_s_2_Speed(1550));
//	}
#endif
//
	// x=MOTOR_DIR;
//	 if((MOTOR_STATUS&(1<<4)==1) && (MOTOR_STATUS&(!(3<<4))==0) )
//	 {
//		 current_steps=TOTAL_MICROSTEP;
//		 TOTAL_MICROSTEP=0;
//		 total_revolutions=0;
//		 RESET_POS();
//		 current_pos=
//
//	 }
//	 else{}
	 	 //read2=Get_param(L6470_ABS_POS);
//	 	 if(read2>255){read++;read2=0;RESET_POS();}
	 	//read=Get_param(L6470_ACC);
	 	//HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_SS_Pin|MAX_RE_Pin|MAX_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SS_Pin MAX_RE_Pin MAX_DE_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin|MAX_RE_Pin|MAX_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void recieve_on_rs485(uint8_t buffer[],size_t buffer_length)
{
	HAL_GPIO_WritePin(MAX_DE_GPIO_Port, MAX_DE_Pin, 0);
	HAL_GPIO_WritePin(MAX_RE_GPIO_Port, MAX_RE_Pin, 0);
	//while(1);

	uart_state=HAL_UART_Receive_IT(&huart1, buffer, buffer_length);
//	RS_485_Data_validate = CRC_CHECK_decode(buffer,  POLYNOMIAL, buffer_length ); //returns 0 if valid ,1 if invalid.
//	CRC_CHECK_decode(buffer,  POLYNOMIAL, buffer_length );
//	while(1){
//	if (CRC_CHECK_decode(buffer,  POLYNOMIAL, buffer_length) ==0){ break;}}
}
void send_on_rs485(uint8_t* buffer)
{



	HAL_GPIO_WritePin(MAX_DE_GPIO_Port, MAX_DE_Pin, 1);
	HAL_GPIO_WritePin(MAX_RE_GPIO_Port, MAX_RE_Pin, 1);
	message_packet_with_crc(buffer,  POLYNOMIAL,BUFFER_LENGTH-1 );
	memcpy( Previous_buffer,buffer,BUFFER_LENGTH);
	//buffer[2]=0x0d;
//	if(buffer[0] == 0x55)
//	{
//		HAL_UART_Transmit(&huart1, buffer,strlen(buffer),100);
//	}
//	else
//	{
//		HAL_UART_Transmit(&huart1, Previous_buffer,strlen(Previous_buffer),100);
//
//	}
	//len=strlen(buffer);
	HAL_UART_Transmit(&huart1, buffer,BUFFER_LENGTH,100);
//	HAL_UART_Transmit(&huart1, "driver2\r\n", 9,100);
	recieve_on_rs485(recieve_buffer,BUFFER_LENGTH);

}


crc_t CRC_generate(crc_t* message1, crc_t polynomial, int nBytes )
{
	crc_t  remainder = 0;
	for (int byte = 0; byte < nBytes; ++byte)
	{
		remainder ^= ((*(message1+byte)) << (WIDTH - 8));/* Bring the next byte into the remainder.   */
		//printf("Hello World %x \n" ,remainder);
		/* Perform modulo-2 division, a bit at a time. */
		for (uint8_t bit = 8; bit > 0; --bit )
		{
			/*Try to divide the current data bit.*/
			if (remainder & TOPBIT){remainder = (remainder ^ POLYNOMIAL)<<1;}
			else{remainder = (remainder << 1);}
		}
	}
	return (remainder);/* The final remainder is the CRC result. */
}

uint8_t* message_packet_with_crc(crc_t* message2, crc_t polynomial, int nBytes )
{
	uint8_t crc_value=0;
	crc_value = CRC_generate( message2,  polynomial,  nBytes );
	message2+=nBytes;
	*message2=crc_value;
	return message2-=nBytes;
}


crc_t CRC_CHECK_decode(crc_t* message, crc_t polynomial, int nBytes )
{
	crc_t  remainder = 0;
	for (int byte = 0; byte < nBytes; ++byte)
	{
		remainder ^= ((*(message+byte)) << (WIDTH - 8));/* Bring the next byte into the remainder.   */
		//printf("Hello World %x \n" ,remainder);
		/* Perform modulo-2 division, a bit at a time. */
		for (uint8_t bit = 8; bit > 0; --bit )
		{
			/*Try to divide the current data bit.*/
			if (remainder & TOPBIT){remainder = (remainder ^ POLYNOMIAL)<<1;}
			else{remainder = (remainder << 1);}
		}
	}
	//printf(" xcv %x \n" ,remainder);
	if(remainder!=0){return 1;}
	else {return 0;}
//	return (remainder);/* The final remainder is the CRC result. */
}

void UNL_freeride()
{
	sp=((RECIEVE_VALID_DATA[4]<<8)|RECIEVE_VALID_DATA[5]);
	switch(RECIEVE_VALID_DATA[1])
	{
	case 'F':
		Run(FORWARD_DIR, step_s_2_Speed(sp));
		break;
	case 'B':
		Run(BACKWARD_DIR, step_s_2_Speed(sp));
			break;
	case 'S':
		SOFT_STOP();
			break;

	}


}


void MAN_freeride()
{
	sp=((RECIEVE_VALID_DATA[4]<<8)|RECIEVE_VALID_DATA[5]);
		switch(RECIEVE_VALID_DATA[1])
		{
		case 'F':
			if((TOTAL_MICROSTEP+s)>=(LIMIT_M-1)){SOFT_STOP();}
			else{Run(FORWARD_DIR, step_s_2_Speed(sp));}


			break;
		case 'B':

//			if((TOTAL_MICROSTEP)>= LIMIT_M ){
//			Run(BACKWARD_DIR, step_s_2_Speed(50));
//			HAL_Delay(50);
//			}
			if((TOTAL_MICROSTEP-s)<=1){SOFT_STOP();}
			else{Run(BACKWARD_DIR, step_s_2_Speed(sp));}
				break;
		case 'S':
			SOFT_STOP();
				break;

		}



}

void MAN_Videoloop()
{
	sp=((RECIEVE_VALID_DATA[4]<<8)|RECIEVE_VALID_DATA[5]);

//	MOTOR_DIR

	if(TOTAL_MICROSTEP>=0 && toggle==1)
			{
				if((TOTAL_MICROSTEP+s)>=(LIMIT_M-1))
				{
					SOFT_STOP();
				}
				else
				{
					Run(FORWARD_DIR, step_s_2_Speed(sp));
				}
				if(TOTAL_MICROSTEP>=(LIMIT_M))
				{
					SOFT_STOP();
					while(MOTOR_STATUS != 0)
					{
						MOTOR_STATUS_RAW_DATA = GET_STATUS();
					}
//					HAL_Delay(500);
					 if(DAMPING != PREVIOUS_DAMPING)
						  {
					//		  SOFT_STOP();
					//		  HAL_Delay(100);
							 if(MOTOR_STATUS==0){
							  Set_param(L6470_ACC, 		DAMPING, L6470_ACC_LEN); // min damping 0x91 //max damping 0x0a //ffa
							  Set_param(L6470_DEC, 		DAMPING, L6470_DEC_LEN);
							  PREVIOUS_DAMPING = DAMPING;
							 }
						  }
					toggle=0;
				}
			}


	else if(TOTAL_MICROSTEP<=(LIMIT_M) && toggle == 0 )
	{
		if((TOTAL_MICROSTEP-s)<=1){SOFT_STOP();}
		else{Run(BACKWARD_DIR, step_s_2_Speed(sp));}
		if(TOTAL_MICROSTEP<=0)
		{
			SOFT_STOP();
			while(MOTOR_STATUS != 0)
			{
				MOTOR_STATUS_RAW_DATA = GET_STATUS();
			}
//			HAL_Delay(500);
			 if(DAMPING != PREVIOUS_DAMPING)
				  {
			//		  SOFT_STOP();
			//		  HAL_Delay(100);
					 if(MOTOR_STATUS==0){
					  Set_param(L6470_ACC, 		DAMPING, L6470_ACC_LEN); // min damping 0x91 //max damping 0x0a //ffa
					  Set_param(L6470_DEC, 		DAMPING, L6470_DEC_LEN);
					  PREVIOUS_DAMPING = DAMPING;
					 }
				  }
			toggle=1;
		}
	}

}


void UNL_Animation()
{
//	sp=((RECIEVE_VALID_DATA[4]<<8)|RECIEVE_VALID_DATA[5]);
	long long int steps_to_move=0,final_move_position=0;
//	 movement_is_pending=0;
	final_move_position =  RECIEVE_VALID_DATA[17] << 56;
	final_move_position |= RECIEVE_VALID_DATA[18] << 48;
	final_move_position |= RECIEVE_VALID_DATA[19] << 40;
	final_move_position |= RECIEVE_VALID_DATA[20] << 32;
	final_move_position |= RECIEVE_VALID_DATA[21] << 24;
	final_move_position |= RECIEVE_VALID_DATA[22] << 16;
	final_move_position |= RECIEVE_VALID_DATA[23] << 8;
	final_move_position |= RECIEVE_VALID_DATA[24];

	if(RECIEVE_VALID_DATA[25] == '-')
	{
		final_move_position = (final_move_position)*(-1);
	}


	if(  TOTAL_MICROSTEP < final_move_position) // forword
	{
		//if((TOTAL_MICROSTEP)<(final_move_position)){Run(FORWARD_DIR, step_s_2_Speed(15));}
		if((TOTAL_MICROSTEP+s)>=(final_move_position-1)){SOFT_STOP();}
		else{Run(FORWARD_DIR, step_s_2_Speed(1550));}


	}
	else if(TOTAL_MICROSTEP > final_move_position  )  // backword
	{
//		if((TOTAL_MICROSTEP)>(final_move_position)){Run(BACKWARD_DIR, step_s_2_Speed(15));}
		if((TOTAL_MICROSTEP-s)<= final_move_position+1){SOFT_STOP();}
		else{Run(BACKWARD_DIR, step_s_2_Speed(1550));}

	}
	else if(final_move_position == TOTAL_MICROSTEP )
	{
		SOFT_STOP();
	}

//	LIMIT_M=CURRENT_POSITION;
//	if(RECIEVE_VALID_DATA[25] == 'M'  && movement_is_pending == 0) // M or S
//	{
//		final_move_position = TOTAL_MICROSTEP+steps_to_move;
//		movement_is_pending=1;
//	}

//	else if(RECIEVE_VALID_DATA[25] == 'S') // M or S
//		{
//			 movement_is_pending=0;
//		}

//	if(movement_is_pending==1)
//	{
//		switch(RECIEVE_VALID_DATA[1])
//		{
//		case 'F':
//			if((TOTAL_MICROSTEP+s)>=(final_move_position-1)){SOFT_STOP();}
//			else{Run(FORWARD_DIR, step_s_2_Speed(1550));}
//
//
//			break;
//		case 'B':
//
//
//			if((TOTAL_MICROSTEP-s)<=1){SOFT_STOP();}
//			else{Run(BACKWARD_DIR, step_s_2_Speed(1550));}
//			break;
//		case 'S':
//			SOFT_STOP();
//			break;
//
//		}
//		if(TOTAL_MICROSTEP  >= final_move_position ){SOFT_STOP();movement_is_pending=0;}
//	}


}


void MAN_Animation()
{
//	sp=((RECIEVE_VALID_DATA[4]<<8)|RECIEVE_VALID_DATA[5]);
	long long int steps_to_move=0,final_move_position=0;
//	 movement_is_pending=0;
	final_move_position =  RECIEVE_VALID_DATA[17] << 56;
	final_move_position |= RECIEVE_VALID_DATA[18] << 48;
	final_move_position |= RECIEVE_VALID_DATA[19] << 40;
	final_move_position |= RECIEVE_VALID_DATA[20] << 32;
	final_move_position |= RECIEVE_VALID_DATA[21] << 24;
	final_move_position |= RECIEVE_VALID_DATA[22] << 16;
	final_move_position |= RECIEVE_VALID_DATA[23] << 8;
	final_move_position |= RECIEVE_VALID_DATA[24];

	if(RECIEVE_VALID_DATA[25] == '-')
	{
		final_move_position = (final_move_position)*(-1);
	}
if(final_move_position<0){final_move_position=0;}
if(final_move_position>LIMIT_M){final_move_position=LIMIT_M;}

	if(  TOTAL_MICROSTEP < final_move_position) // forword
	{
		//if((TOTAL_MICROSTEP)<(final_move_position)){Run(FORWARD_DIR, step_s_2_Speed(15));}
		if((TOTAL_MICROSTEP+s)>=(final_move_position-1)){SOFT_STOP();}
		else{Run(FORWARD_DIR, step_s_2_Speed(1550));}


	}
	else if(TOTAL_MICROSTEP > final_move_position  )  // backword
	{
//		if((TOTAL_MICROSTEP)>(final_move_position)){Run(BACKWARD_DIR, step_s_2_Speed(15));}
		if((TOTAL_MICROSTEP-s)<= final_move_position+1){SOFT_STOP();}
		else{Run(BACKWARD_DIR, step_s_2_Speed(1550));}

	}
	else if(final_move_position == TOTAL_MICROSTEP )
	{
		SOFT_STOP();
	}



}
void Limit_setting()
{
	if(RECIEVE_VALID_DATA[7]== 'L')
	{

		CURRENT_POSITION =  RECIEVE_VALID_DATA[8] << 56;
		CURRENT_POSITION |= RECIEVE_VALID_DATA[9] << 48;
		CURRENT_POSITION |= RECIEVE_VALID_DATA[10] << 40;
		CURRENT_POSITION |= RECIEVE_VALID_DATA[11] << 32;
		CURRENT_POSITION |= RECIEVE_VALID_DATA[12] << 24;
		CURRENT_POSITION |= RECIEVE_VALID_DATA[13] << 16;
		CURRENT_POSITION |= RECIEVE_VALID_DATA[14] << 8;
		CURRENT_POSITION |= RECIEVE_VALID_DATA[15];
		LIMIT_M=CURRENT_POSITION;
		if(RECIEVE_VALID_DATA[16]=='-'){CURRENT_POSITION=0;}
		//CURRENT_POSITION
	}

}



 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(counter>1){}
	RS_485_Data_validate = CRC_CHECK_decode(recieve_buffer,  POLYNOMIAL, BUFFER_LENGTH); //returns 0 if valid ,1 if invalid.

	if(RS_485_Data_validate==0)
	{
		if(recieve_buffer[0]== 0x55)// if acknowledgment is valid
		{
			memcpy( RECIEVE_VALID_DATA,recieve_buffer,BUFFER_LENGTH);//store data into actual data buffer
			send_buffer[0]=0x55;//0x55 is acknowledgment for a valid data
			send_on_rs485(send_buffer);
			state_of_rs485=1;
		}
		else if(recieve_buffer[0]== 0x65)// if acknowledgment is 0x65 means data is not received properly
		{

					//send previous data
			send_buffer[0]=0x55;
			Previous_buffer[0]=0x55;
			send_on_rs485(Previous_buffer);
			state_of_rs485=2;
		}

		// send acknowledgment of valid data
//		send_buffer[0]=0x55;//0x55 is acknowledgment for a valid data


	}
	else
	{
		//neglect the data
		// send acknowledgment of invalid data
		send_buffer[0]=0x65;
		send_on_rs485(send_buffer);
		state_of_rs485=3;
	}
//	send_on_rs485(send_buffer);

	counter++;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//read2=Read_total_steps();
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
