/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32wlxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "subghz_phy_app.h"
#include "user_define.h"
#include "extern.h"
#include "platform.h"
#include "sys_app.h"
#include "radio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef unsigned char byte;
typedef unsigned short word;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Slave_LoRa_Tx(byte lora_tx_num);
void LoRa_Data_exe(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern SUBGHZ_HandleTypeDef hsubghz;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */
extern unsigned char RF_start_flag;
extern unsigned char Station_No;
extern unsigned char Tx_EN;
extern bool isMaster;
extern unsigned char tx_EN;
extern int Tx_EEROR_CNT;
extern byte Rcv1_ok;
int Timer_cnt = 0, T100ms_cnt = 0;
byte RS485_dead_time=0;	//1msec down counter
byte LoRa_test_buf[255] = {"Slave_LoRa_Tx_Success"};
byte tx_delay_num = 0, LoRa_Equipment_Num_Fg = 0;
word LoRa_CRC_data = 0, LoRa_temp = 0;
unsigned int md_rx_time = 0, rx_time_cnt = 0,Tx_send_EN = 0,Tx_send_flag = 0;

extern byte Rx1_data, Rx_stop_flag,Rx_Data_Flag;
extern word Rx1_index;
extern byte test_index, test_rxbuf[256];
extern byte CheckSum_ing;
extern byte Rx1_step, Rx1_next_data_no;
extern word Rx1_data_sum;
extern word Error1_cnt;
extern uint8_t Slave_BackupTxBuffer[255];
extern uint8_t Slave_BackupRxBuffer[255];
extern byte CheckSum_EN;
extern byte Rx1_Checksum_H;
extern byte Rx1_Checksum_L;
extern uint8_t BufferRx[255];
extern unsigned char MD_tx_EN;

extern word CRC16(byte *Data, word Len);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WLxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wlxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC Tamper, RTC TimeStamp, LSECSS and RTC SSRU Interrupts.
  */
void TAMP_STAMP_LSECSS_SSRU_IRQHandler(void)
{
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 0 */

  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 0 */
  HAL_RTCEx_SSRUIRQHandler(&hrtc);
  /* USER CODE BEGIN TAMP_STAMP_LSECSS_SSRU_IRQn 1 */

  /* USER CODE END TAMP_STAMP_LSECSS_SSRU_IRQn 1 */
}

/**
  * @brief This function handles DMA1 Channel 5 Interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USART2 Interrupt.
  */

byte Rx2_index = 0, Rx2_next_data_no = 0, Rcv2_cnt = 0;
word LoRa_CheckSum_data;
word Error2_cnt = 0;
byte Rx2_step = 0;

byte Rx2_Checksum_H = 0, Rx2_Checksum_L = 0;
byte LoRa_CheckSum_ing = 0;
byte LoRa_CheckSum_EN = 0;

byte LoRa_Rcv_ok = 0;

word LoRa_test_crc1 = 0, LoRa_test_crc2 = 0;
byte LoRa_Tx_buf[100] ={0x01, 0x04, 0x02, 0x00, 0x0A, 0xF8, 0xF4};
byte LoRa_Tx_index=0;
byte LoRa_Tx_send_number=0;
byte Tx2_index=0, Tx2_send_number = 100;

void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

uint8_t Tx1_buf[20] = "";
uint8_t Tx1_index = 0, Tx1_send_number = 0;
uint8_t Rx1_buf[255];
uint8_t rx1_data,rx1_index;
uint8_t Rcv1_cnt = 0,Tx1_cnt=0;
uint16_t Error_cnt = 0, USART1_int_cnt = 0;

uint32_t ISR_data=0;
uint32_t CR1_data=0;
uint32_t CR3_data=0;
uint8_t error_buf[260] = {0};

uint8_t Buf_byte_size = 0, Radio_Tx_EN = 0;
word CRC_data = 0;
word w_temp = 0;
byte Rcv2_ok = 0, RS232_dead_time = 0;
byte Data_Cnt = 0; // 모터 드라이버에 보낼 데이터 갯수 
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
		  
    uint32_t error_flags;
	USART1_int_cnt++;
	
	ISR_data = USART1->ISR;
	CR1_data = USART1->CR1;
	CR3_data = USART1->CR3;
	
	static uint8_t error_index = 0;
	
	error_flags = (ISR_data & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	if(error_flags)
	{
		if( (((CR3_data & USART_CR3_EIE) != RESET) || ((CR1_data & (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_PEIE)) != RESET)) )
		{
			/* UART parity error interrupt occurred -------------------------------------*/
			if(((ISR_data & USART_ISR_PE) != RESET) && ((CR1_data & USART_CR1_PEIE) != RESET))
			{
				USART1->ICR = UART_CLEAR_PEF;	
				//error_buf[error_index++] = 'P';
			}

			/* UART frame error interrupt occurred 98--------------------------------------*/
			if(((ISR_data & USART_ISR_FE) != RESET) && ((CR3_data & USART_CR3_EIE) != RESET))
			{
				USART1->ICR = UART_CLEAR_FEF;	
				//error_buf[error_index++] = 'F';
			}

			/* UART noise error interrupt occurred --------------------------------------*/
			if(((ISR_data & USART_ISR_NE) != RESET) && ((CR3_data & USART_CR3_EIE) != RESET))
			{
				USART1->ICR = UART_CLEAR_NEF;	
				//error_buf[error_index++] = 'N';
			}
		
			/* UART Over-Run interrupt occurred -----------------------------------------*/
			if(((ISR_data & USART_ISR_ORE) != RESET) && (((CR1_data & USART_CR1_RXNEIE_RXFNEIE) != RESET) || ((CR3_data & USART_CR3_EIE) != RESET)))
			{
				USART1->ICR = UART_CLEAR_OREF;	
				//error_buf[error_index++] = 'O';
			}
		}
		static uint16_t Uart1_error_cnt=0;
		USART1->ICR = 0xFFFFFFFF;	//USART1's Interrupt flag clear register (ALL Clear)
		Uart1_error_cnt++;
		return;
	} // End if some error occurs
	else
	{
		if((USART1->CR1 & USART_CR1_RXNEIE_RXFNEIE) && (USART1-> ISR & USART_ISR_RXNE_RXFNE))	//수신버퍼가 채워졌을 때
		{
			md_rx_time = 1;
		  
			Rcv1_cnt++;
			
			rx1_data = USART1->RDR;		//수신값 저장
			
			test_rxbuf[rx1_index++] = rx1_data;
			
			USART1->ISR = USART_ISR_RXNE_RXFNE;	// USART1's USART_ISR_RXNE_RXFNE flag clear(bit5)
			USART1->ISR &= ~USART_ISR_RXNE_RXFNE;	// USART1's USART_ISR_RXNE_RXFNE flag clear(bit5)	
			
			  switch(Rx2_step)
			  {
					case 0 :
					if(rx1_data == ENQ)// 0번 인덱스 = 국번이 같을때
					{
					  Slave_BackupTxBuffer[0] = rx1_data;
					  Rx2_step = 1;
					  Rx2_index = 1;
					}
					else if(rx1_data != ENQ && LoRa_Equipment_Num_Fg == 0) // Slave LoRa의 국번을 정하는 Command( 모터드라이버 전원 On시 최초 딱 한번만 장비번호를 불러옴 )
					{
					  ENQ = rx1_data;
					  
					  Rx2_step = 0;
					  Rx2_index = 0;
					  LoRa_Equipment_Num_Fg = 1;
					}
					else
					{
					  Rx2_step = 0;
					  Rx2_index = 0;
					}
					
					break;
					
				   case 1: // Function Code
					if((rx1_data == 0x04 || rx1_data == 0x10 || rx1_data == 0x90 || rx1_data == 0x07))// function 0x04 : Read Request // function 0x90 : Write Request - The Function Code 1 
					{										  					  					 // function 0x10 : Write Request // function 0x07 : Equipment_Num 장비 번호를 바꾸는 함수
					  Slave_BackupTxBuffer[Rx2_index++] = rx1_data;
					  Rx2_step++;
					}
					else
					{
					  Rx2_step = 0;
					  Rx2_index = 0;
					}
					
					break;
					
				   case 2: //  읽기 요청한 레지스터의 수
					Slave_BackupTxBuffer[Rx2_index++] = rx1_data;
					if(Slave_BackupTxBuffer[1] == 0x04) // Master에서 읽기 요청한 데이터 수
					{
					  Rx2_step = 10;
					  Rx2_next_data_no = Slave_BackupTxBuffer[2];
					}
					else if(Slave_BackupTxBuffer[1] == 0x10)
					{
					  Rx2_step++;
					}
					else if(Slave_BackupTxBuffer[1] == 0x90)
					{
					  Rx2_step = 6;
					}
					else if(Slave_BackupTxBuffer[1] == 0x07)
					{
					  Rx2_step = 6;
					}
					break;
					
				   case 3: // Data Address[Low]
					Slave_BackupTxBuffer[Rx2_index++] = rx1_data;
					if(Slave_BackupTxBuffer[1] == 0x10)
					{
					  Rx2_step++;
					}
					
					break;

				   case 4: // Quantity of input registers[High]
					Slave_BackupTxBuffer[Rx2_index++] = rx1_data;
					if(Slave_BackupTxBuffer[1] == 0x10)
					{
					  Rx2_step++;
					}
					
					break;
					
				   case 5: // Quantity of input registers[Low]
					Slave_BackupTxBuffer[Rx2_index++] = rx1_data;
					if(Slave_BackupTxBuffer[1] == 0x10)
					{
						Rx2_step++; 
					}
					
					break;
					
				   case 6: // CRC16 Code [Low] 
					Slave_BackupTxBuffer[Rx2_index++] = rx1_data;
					Rx2_step++;
					
					break;
					
				   case 7: // CRC16 Code [High]
					Slave_BackupTxBuffer[Rx2_index] = rx1_data;
					
					CRC_data = CRC16(Slave_BackupTxBuffer,Rx2_index - 1);
					
					// CRC_High          CRC Low
					w_temp = (rx1_data<<8) | Slave_BackupTxBuffer[Rx2_index-1];
					
					if((CRC_data == w_temp) && Slave_BackupTxBuffer[1] != 0x07)
					{
					  GREEN_LED_ON; /*LED_Green_ON */
					  memcpy(TX_BufferRx, Slave_BackupTxBuffer, sizeof(Slave_BackupTxBuffer));		//LoRa_TX_buff의 데이터를 TX_BufferRx에 복사
					}
					else if((CRC_data == w_temp) && Slave_BackupTxBuffer[1] == 0x07)
					{
					  ENQ = Slave_BackupTxBuffer[2]; // 0x07 장비 번호를 변경하는 명령어의 CRC16 값이 같다면 국번을 장비 번호로 변경
					}
				
					Rx2_index = 0;
					Rx2_step = 0;
					md_rx_time = 0;
					
					if(Slave_BackupTxBuffer[1] != 0x07)
					{
						isMaster = true; // LoRa_Slave를 TX모드로 변환
					}
					
					break;
					
				   default:
					Rx2_step = 0;
					Rx2_index = 0;
					break;
					
				   case 10: // 모터 드라이버로 부터 Read data 수신
					Slave_BackupTxBuffer[Rx2_index++] = rx1_data;
					if(Rx2_next_data_no <= 1)
					{
					  Rx2_step = 6; // CRC16_Low 수신
					}
					else
					{
					  Rx2_next_data_no--;
					}
					break;
			 }
		}
		else if((USART1->CR1 & USART_CR1_TCIE) && (USART1-> ISR & USART_CR1_TCIE))	//송신버퍼가 채워졌을 때
		{
			Tx1_cnt++;
			
			if(Tx1_send_number)
			{
				Tx1_send_number--;  
				USART1->TDR = Slave_BackupRxBuffer[Tx1_index++];
			}
			else	//모든 Data 전송
			{
				USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE; 	// USART1's RXE Interrupt Enable		
				USART1->CR1 &= ~USART_CR1_TCIE;				// USART1's TXE Interrupt Disable
			}
			return;
		}
		else
		{
			Error_cnt++;
		}
	}
  /* USER CODE END USART2_IRQn 0 */
  //  HAL_UART_IRQHandler(&huart1); // HAL 핸들러를 안쓰기 위해서 위와 같이 한거임 주석처리 해야해 안하면 통신 오류 생김!!
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

void Slave_LoRa_Tx(uint8_t lora_tx_num)	//USART1
{
	/*Send the character*/
	USART1->TDR = Slave_BackupRxBuffer[0];
	Tx1_index = 1;
	Tx1_send_number = lora_tx_num-1;
	USART1->CR1 &= ~USART_CR1_RXNEIE_RXFNEIE; 	//USART2's RXE Interrupt Disable	
	USART1->CR1 |= USART_CR1_TCIE; 		//USART2's TXE Interrupt Enable		
}

/**
  * @brief This function handles RTC Alarms (A and B) Interrupt.
  */
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */

  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/**
  * @brief This function handles SUBGHZ Radio Interrupt.
  */
void SUBGHZ_Radio_IRQHandler(void)
{
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 0 */

  /* USER CODE END SUBGHZ_Radio_IRQn 0 */
  HAL_SUBGHZ_IRQHandler(&hsubghz);
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 1 */

  /* USER CODE END SUBGHZ_Radio_IRQn 1 */
}

/**
  * @brief This function handles TIM2 Global Interrupt.
  */

void TIM2_IRQHandler(void) // 1ms cnt
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	
	//1ms timer
	static unsigned int time2_cnt = 0;
	static unsigned char sec = 0, min = 0;
	
	if(md_rx_time)
	{
	  rx_time_cnt++;
	}
	  
	
	if(Rcv2_ok)	//UART2 data received?
    {
	  	byte i, no;
		word wtemp;

		byte *bptr;
		byte step;
        if(RS232_dead_time)
            RS232_dead_time--;	//1msec 마다 1씩 감소
        else
        {
            Rcv2_ok = (Rcv2_ok<<4) | Rcv2_ok;	//5:55, 4:44, E:EE
        }
    }
	
	/*if(Tx_send_EN)
	{
	  if(Tx_send_flag)
	  {
		Tx_send_flag--;
	  }
	  else
	  {
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0); //LED_GREEN_OFF
		isMaster = true; // LoRa_Slave를 TX모드로 변환
		Tx_send_EN = 0;
	  }
	}*/
	
	Timer_cnt--;
	if(Timer_cnt <= 0)
	{
	  Timer_cnt = 0;
	}
	
	if(++time2_cnt >= 1000)
	{
	  time2_cnt = 0;
	  
	  Send_EN = 1;
	  
	  Tx_EEROR_CNT--;
	  if(Tx_EEROR_CNT <= 0)
	  {
		Tx_EEROR_CNT=0;
	  }
		//MD_tx_EN = 1;
		if(++sec >= 60)
		{
			sec = 0;
			min++;
		}
		
		//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_RED */
	  RED_LED_TOGGLE; /* LED_RED Toggle */
	}
	
	if(++T100ms_cnt >= 100)
	{
	  T100ms_cnt = 0;
	  Timer_cnt = 100;
	}
	
	/*
	static unsigned int rf_start_cnt = 0;
	if(++rf_start_cnt >= 5000 && RF_start_flag)	
	{
		RF_start_flag = 0;
		rf_start_cnt = 0;
		
		isMaster = true;	//송신모드
		Station_No = 0x01;	//로라 모듈 전원이 들어가고, 5초 뒤에 통신시작(송신)
		Tx_EN = 1;
	}*/
	
	
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
