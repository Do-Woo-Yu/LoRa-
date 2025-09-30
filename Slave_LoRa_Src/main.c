
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "app_subghz_phy.h"
#include "gpio.h"
#include "stm32wlxx_hal_tim.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include <stdio.h>
#include <ctype.h> // ctype 헤더 파일 추가
#include "extern.h"
#include "utilities_def.h"

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
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
unsigned int scan = 0, reset_flag = 0, Tx_Success = 0, Rx_Success = 0, Rx_Fail = 0, Tx_Ready = 0, Tx_Fail = 0, Tx_Stop = 0, Rx_Stop = 0, error_msg = 0;
uint8_t tx_EN = 0,display_PC = 0;
uint8_t Rx_buf[20];
uint8_t ok_Buffer[5] = {"OK"};
uint8_t go_Buffer[5] = {"\0"};

uint8_t Tx_Ready_Buffer[10] = {"Tx Ready\r\n"};
uint8_t Tx_Success_Buffer[12] = {"Tx Success\r\n"};
uint8_t Tx_Fail_Buffer[9] = {"Tx Fail\r\n"};
uint8_t Rx_Success_Buffer[12] = {"Rx Success\r\n"};
uint8_t Rx_Fail_Buffer[9] = {"Rx Fail\r\n"};
uint8_t Tx_Stop_Buffer[9] = {"Tx Stop\r\n"};
uint8_t Rx_Stop_Buffer[9] = {"Rx Stop\r\n"};
uint8_t error_msg_Buffer[79] = {"Communication failed\r\n"};

word LoRa_Master_CRC16 = 0,MD_CRC16 = 0,LoRa_Master_CRC16_Data = 0,MD_CRC16_Data = 0;

extern uint8_t BufferRx[255], BufferTx[255],Tx1_buf[255],Tx1_send_number,Tx1_index;
extern uint8_t Rx1_buf[255];
extern unsigned char Kang_TX_buf[10];
extern unsigned int num_cnt,master_start,send_tx1_cnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ENQ_Array_Sort(unsigned char arr[], int n);
bool isDuplicate(unsigned char arr[], int size, unsigned char value);
extern void LoRa_Tx(uint8_t lora_tx_num);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SubGHz_Phy_Init();
  
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /*## Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
 // if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  //{
    /* Starting Error */
 //   Error_Handler();
  //}
  
 
  USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE; 	// USART1's RXE Interrupt Enable	
  //USART1->CR1 &= ~USART_CR1_TCIE;			// USART1's TXE Interrupt Disable
  
  HAL_TIM_Base_Start_IT(&htim2);	//TIM2 interrupt start
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
	scan++;
	
    MX_SubGHz_Phy_Process();	//TX(송신)을 위해 반드시 필요함.
	
	/*static unsigned int tx_test_flag = 0;
	if(tx_test_flag)
	{
	    tx_test_flag = 0;
		UART1_wr_exe();
	}*/
	  
	if(wireres_receive)
	{
	  	wireres_receive = 0;
		
		if(BufferRx[1] == 0x04)// 읽기 함수
		{
			 Rcv2_ok = 0x04;
		}
	 	if(BufferRx[1] == 0x10)// 쓰기 함수
		{
			 Rcv2_ok = 0x10;
		}
		if(BufferRx[1] == 0x90)// 예외 응답 함수( 모터 드라이버로 데이터를 쓰지 못할때 반환 처리를 받아와 PC로 전송 )
		{
		   	 Rcv2_ok = 0x90;
		}
	}
	
	LoRa_Data_exe();
	
	/*if(tx_EN)
	{
	  tx_EN = 0;
	  
	  send_tx1_cnt = 0;
	  Tx1_send_number = 0;
	  Tx1_index = 0;
	  
	  for(int null_search_flag = 0; null_search_flag <= sizeof(Tx1_buf); null_search_flag++)
	  {
		  if(Tx1_buf[null_search_flag] == '\0')
		  {
			send_tx1_cnt++;
		  }
	  }
		
	  Tx1_send_number = sizeof(Tx1_buf) - send_tx1_cnt;
	  
	  //LoRa_Tx(5);
	  
	  for(int rx_buffer_reset = 0; rx_buffer_reset < sizeof(Rx1_buf); rx_buffer_reset++)
	  {
	     Rx1_buf[rx_buffer_reset] = '\0';
	  }
	}*/
	
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;//PRESCALER_VALUE;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 48e6/1e3-1;//PERIOD_VALUE;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}


/* USER CODE BEGIN 4 */

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
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
