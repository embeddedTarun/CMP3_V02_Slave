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
#include "l6470.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef uint8_t crc_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WIDTH  (8 * sizeof(crc_t))
#define TOPBIT (1 << (WIDTH - 1))

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned long read=0,read2=0;
//uint8_t rc_data[1]={0};
//uint8_t buffer[14];
//char buffer1[9]="hello01\r\n";
//uint8_t* send_buffer="h-l6470\r\n";
//uint8_t send_buffer[11] = {0x55,'-','l','6','4','7','0','\r','\n',0x00,0x00};
uint8_t send_buffer[11] = {0x55,0x02,0x01,0x01,0xf4,0x00,'0','\r','\n',0x00,0x55};
uint8_t Previous_buffer[11];
uint8_t RECIEVE_VALID_DATA[11];
uint8_t recieve_buffer[11];

uint8_t RS_485_Data_validate=0;
int BUFFER_LENGTH=11;
uint8_t state_of_rs485=1;
uint8_t counter=0;
HAL_StatusTypeDef uart_state;
int len=0;
uint16_t DAMPING =0;
uint16_t SPEED=0;
uint16_t sp=0;
uint8_t sp1=0;
uint8_t sp2=0;
uint16_t x=0x1234;
int step=0;
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

  //send_on_rs485(send_buffer);
  recieve_on_rs485(recieve_buffer,BUFFER_LENGTH);

  Spi_init();
//HAL_TIM_Base_Start_IT(&htim4);
HAL_Delay(1000);



  	HARD_HIZ();
    GET_STATUS();

	Set_param(L6470_ACC, 		0x0482, L6470_ACC_LEN);//ffa
	Set_param(L6470_DEC, 		0x0482, L6470_DEC_LEN);
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
	//Move(FORWARD_DIR,200*128);
//	HAL_Delay(4000);
//
//	GO_TO(0xfffff);

	//Run(FORWARD_DIR, step_s_2_Speed(4000));
	//HAL_Delay(4000);
	//Set_param(SOFT_STOP_CMD,0, 8);



	//Move(FORWARD_DIR,4194303);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	   DAMPING = RECIEVE_VALID_DATA[3];
	   SPEED = RECIEVE_VALID_DATA[2];
//	   sp1 = RECIEVE_VALID_DATA[4];
//	  		  sp2=RECIEVE_VALID_DATA[5];
//	  		  sp=(sp1<<8)|sp2;
	  		  sp=((RECIEVE_VALID_DATA[4]<<8)|RECIEVE_VALID_DATA[5]);
//	  HAL_Delay(20);
	  if(RECIEVE_VALID_DATA[1]== 'F')
	  {
//		   sp = RECIEVE_VALID_DATA[4];
//		  sp=sp<<8;
//		  sp|=RECIEVE_VALID_DATA[5];
		Run(FORWARD_DIR, step_s_2_Speed(sp));
		step+=256;
	  }
	  else if(RECIEVE_VALID_DATA[1]== 'R')
	  {
//		   sp = RECIEVE_VALID_DATA[4];
//		  sp=sp<<8;
//		  sp|=RECIEVE_VALID_DATA[5];
		  Run(BACKWARD_DIR, step_s_2_Speed(sp));
		  step-=256;
	  }

	  else if(RECIEVE_VALID_DATA[1]== 'S')
	 	  {
		  	  SOFT_STOP();
//	 		  uint16_t sp = RECIEVE_VALID_DATA[2];
//	 		  sp=sp<<8;
//	 		  sp|=RECIEVE_VALID_DATA[3];
//	 		  Run(BACKWARD_DIR, step_s_2_Speed(sp));
	 	  }
	  	  	  while(state_of_rs485!=1);
	  	  	  send_buffer[1]=0x01;
	  	  	  send_buffer[2]=step>>24;
			  send_buffer[3]=(step>>16)&0xff;
			  send_buffer[4]=(step>>8)&0xff;
			  send_buffer[5]=(step)&0xff;


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
	 read2=Read_total_steps();
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
	strcpy( Previous_buffer,buffer);
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
	len=strlen(buffer);
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



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(counter>1){}
	RS_485_Data_validate = CRC_CHECK_decode(recieve_buffer,  POLYNOMIAL, BUFFER_LENGTH); //returns 0 if valid ,1 if invalid.

	if(RS_485_Data_validate==0)
	{
		if(recieve_buffer[0]== 0x55)// if acknowledgment is valid
		{
			strcpy( RECIEVE_VALID_DATA,recieve_buffer);//store data into actual data buffer
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
	read2=Read_total_steps();
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
