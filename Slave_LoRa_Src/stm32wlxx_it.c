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
#include "stm32wlxx_hal_tim.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "extern.h"
#include <stdbool.h>
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include <stdio.h>
#include <ctype.h> // ctype 헤더 파일 추가
#include "utilities_def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

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
void LoRa_Tx(uint8_t lora_tx_num);
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
extern unsigned char Kang_TX_buf[10];
extern unsigned char Log_tx_buf[10];
extern uint8_t tx_EN,display_PC;
extern uint8_t ok_Buffer[5];
extern int Rx_EEROR_CNT;

unsigned int Tx_Flag_cnt = 0,T3000ms_cnt = 0,t1000ms_cnt = 0,num_cnt = 0,master_start = 0,send_tx1_cnt=0, T1000ms_cnt = 0,error_flag = 0,Tx_end_recieve_EN = 0,rx_end_num = 0,test_again_send_en = 0,test_tx_again_send_flag = 0;
int Timer_cnt = 0, T100ms_cnt = 0;
unsigned int Dial_Num = 0,send_data_cnt = 0; // PC에 출력되는 국번
		
byte rx_end_delay_num = 0;

/* USER CODE BEGIN EV */

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
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART1 Interrupt.
  */

uint8_t Tx1_buf[255] = "";
uint8_t Tx1_index = 0, Tx1_send_number = 0;
uint8_t Rx1_buf[255];
uint8_t rx1_data;
uint8_t Rx1_index = 0, Rcv1_cnt = 0;
uint16_t Error_cnt = 0, USART1_int_cnt = 0;

uint32_t ISR_data=0;
uint32_t CR1_data=0;
uint32_t CR3_data=0;
uint8_t error_buf[260] = {0};

uint8_t Buf_byte_size = 0, Radio_Tx_EN = 0;
byte Rx1_step=0,Rx1_next_data_no = 0,LoRa_Rx_ing = 0;
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

			/* UART frame error interrupt occurred --------------------------------------*/
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
		if((USART1->CR1 & USART_CR1_RXNEIE_RXFNEIE) && (USART1-> ISR & USART_ISR_RXNE_RXFNE) && Tx_go_cnt == 0)	//수신버퍼가 채워졌을 때
		{
			Rcv1_cnt++;
			
			rx1_data = USART1->RDR;		//수신값 저장
		
			USART1->ISR = USART_ISR_RXNE_RXFNE;	// USART1's USART_ISR_RXNE_RXFNE flag clear(bit5)
			USART1->ISR &= ~USART_ISR_RXNE_RXFNE;	// USART1's USART_ISR_RXNE_RXFNE flag clear(bit5)

			switch(Rx1_step) 
			{
			 case 0:
			  Rx1_buf[Rx1_index++] = rx1_data;
			  Rx1_step++;
			  break;
			  
			 case 1:
			  if((rx1_data == 0x04 || rx1_data == 0x10)) //function 0x04: Read Request 
			  {											 //function 0x10: Write Request
				GREEN_LED_ON; /*LED_Green_ON */
				Rx1_buf[Rx1_index++] = rx1_data;
				Rx1_step++;
			  }
			  break;
			  
			 case 2: //Waiting Address (High Address byte)
			  Rx1_buf[Rx1_index++] = rx1_data;
			  Rx1_step++;
			  break;
			  
			 case 3: //Waiting Address  (Low Address byte)
			  if(rx1_data == 0xEE)
			  {						
				  Rx1_step = 0;     //Waiting for  국번
				  Rx1_index = 0;
			  }
			  else                                                    
			  {
				  Rx1_buf[Rx1_index++] = rx1_data;
				  Rx1_step++;
			  }
			  break;
			  
			 case 4: //Waiting 길이 (High Length byte)
			  Rx1_buf[Rx1_index++] = rx1_data;
			  Rx1_step++;
			  break;
			  
			 case 5: //Waiting 길이 (Low Length byte)
			  Rx1_buf[Rx1_index++] = rx1_data;
			  if(Rx1_buf[1] == 0x04)
			  {
				Rx1_step = 7;
			  }
			  else if(Rx1_buf[1] == 0x10)
			  {
				Rx1_step++;
			  }
			  break;
			  
			 case 6: //쓰기 요청할 데이터 수 (The number of data bytes)
			  Rx1_buf[Rx1_index++] = rx1_data;
			  if(Rx1_buf[1] == 0x10)
			  {
				Rx1_step = 10;
				Rx1_next_data_no = Rx1_buf[5]*2;
			  }
			  else
			  {
				  Rx1_step++;		
				  Rx1_next_data_no = 0;
			  }
			  break;
			  
			 case 7: //Waiting for CRC16 (Low byte)	
			  Rx1_buf[Rx1_index++] = rx1_data;
			  Rx1_step++;
			  break;
			  
			 case 8: //Waiting for CRC16 (High byte)	
			  Rx1_buf[Rx1_index] = rx1_data;
			  Rx1_index = 0;
			  Rx1_step = 0;
			  memcpy(BufferTx, Rx1_buf, sizeof(Rx1_buf));	//LoRa_TX_buf의 데이터를 BufferTx에 복사
			  isMaster = true; // LoRa_Slave를 TX모드로 변환
			  break;
			  
			 default :
			  Rx1_index = 0;
			  Rx1_step = 0;
			  break;
			  
			 case 10: //Write data 수신
			  Rx1_buf[Rx1_index++] = rx1_data;
			  if(Rx1_next_data_no <= 1)
			  {
				Rx1_step = 7; // CRC16_Low 수신
			  }
			  else
			  {
				Rx1_next_data_no--;
			  }
			  break;
			}
		}
		else if((USART1->CR1 & USART_CR1_TCIE) && (USART1-> ISR & USART_CR1_TCIE))	//송신버퍼가 채워졌을 때
		{
			static uint8_t Tx1_cnt=0;
			Tx1_cnt++;
			
			if(Tx1_send_number)
			{
				Tx1_send_number--;  
				USART1->TDR = Master_BackupRxBuffer[Tx1_index++];
			}
			else	//모든 Data 전송
			{
				USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE; 	// USART1's RXE Interrupt Enable		
				USART1->CR1 &= ~USART_CR1_TCIE;				// USART1's TXE Interrupt Disable
			}
			return;
		  }
	}

  /* USER CODE END USART2_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

void LoRa_Tx(uint8_t lora_tx_num)	//USART1
{
	/*Send the character*/
	USART1->TDR = Master_BackupRxBuffer[0];
	Tx1_index = 1;
	Tx1_send_number = lora_tx_num-1;
	USART1->CR1 &= ~USART_CR1_RXNEIE_RXFNEIE; 	//USART1's RXE Interrupt Disable	
	USART1->CR1 |= USART_CR1_TCIE; 		//USART1's TXE Interrupt Enable		
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

void LoRa_Data_exe(void)
{
  
	byte i, no;
	word wtemp;

	byte *bptr;
	
	if(Rcv2_ok == 0x44) // Motor-Driver로 부터 데이터를 정상적으로 읽어 왔을때( 읽기 요청 )
	{
		Rcv2_ok = 0;
		if(BufferRx[1] == 0x04)
		{
			wtemp = (Master_BackupTxBuffer[2] << 8) + Master_BackupTxBuffer[3]; // 읽기 시작 주소( Data Address )
			//bptr = (byte *)(0x20000000 + wtemp); // #define 0x20000000
			no = BufferRx[2]; // Master에서 읽기 요청한 데이터 수
			for(i = 0; i < no; i++)
			{
				//*bptr = BufferRx[3+i];	// Motor-Driver에서 받아온 값을 _LoRa_Data 구조체에 대입
				//bptr++; // Motor-Driver에서 받아온 값을 _LoRa_Data 구조체에 대입
				send_data_cnt++;
			}
			
			for(int RxBuf_reset = 0; RxBuf_reset < sizeof(BufferRx); RxBuf_reset++) // BufferRx 초기화 
			{
		      BufferRx[RxBuf_reset] = '\0';
			}
			
			LoRa_Tx(send_data_cnt+5);
			
			GREEN_LED_OFF; /*LED_Green_OFF */
			
			//현재 수신 감도( 통신 테스트용 )
			/*char rssi_text[] = "\r\n● Reception sensitivity : "; 
			
			HAL_UART_Transmit(&huart1, (uint8_t *)rssi_text,strlen(rssi_text), 100);
			
			char rssi_data[] = "         "; 
				
			sprintf(rssi_data,"%ddB\r\n",RssiValue);
			
			HAL_UART_Transmit(&huart1, (uint8_t *)rssi_data,strlen(rssi_data), 100);*/
			
			send_data_cnt = 0;
		}
		
		//Rx_Success_Flag = 1; // PC에 받아온 데이터를 출력
	}
	if(Rcv2_ok == 0x110) // Motor-Driver로 부터 데이터를 정상적으로 읽어 왔을때( 쓰기 요청 )
	{
	  Rcv2_ok = 0;
	  for(int RxBuf_reset = 0; RxBuf_reset < sizeof(BufferRx); RxBuf_reset++) // BufferRx 초기화 
	  {
	    BufferRx[RxBuf_reset] = '\0';
	  }
	  
	  LoRa_Tx(8);
	  
	  GREEN_LED_OFF; /*LED_Green_OFF */
	}
	if(Rcv2_ok == 0x990) // Motor-Driver로 부터 데이터를 정상적으로 읽어 왔을때( 쓰기 요청 )
	{
	  Rcv2_ok = 0;
	  for(int RxBuf_reset = 0; RxBuf_reset < sizeof(BufferRx); RxBuf_reset++) // BufferRx 초기화 
	  {
	    BufferRx[RxBuf_reset] = '\0';
	  }
	  
	  LoRa_Tx(5);
	  
	  GREEN_LED_OFF; /*LED_Green_OFF */
	}
}

/**
  * @brief This function handles TIM2 Global Interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	
	//1ms timer
	static unsigned int time2_cnt = 0;
	static unsigned char sec = 0, min = 0;
	static unsigned short num = 0;
	
	if(Rcv2_ok)	// Motor-Driver에서 받은 데이터를 _LoRa_Data 구조체에 대입
    {
	  Rcv2_ok = (Rcv2_ok<<4) | Rcv2_ok;	//5:55, 4:44, E:EE
    }
	
	/*if(Tx_end_recieve_EN)
	{
		if(Tx_end_recieve_flag)
		{
		  Tx_end_recieve_flag--;
		}
		else
		{
		  isMaster = true; // LoRa_Master를 다시 TX모드로 변환(끝) 
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
		  Tx_end_recieve_EN = 0;
		  send_data_cnt = 0;
		}
	}*/
	
	Timer_cnt--;
	if(Timer_cnt <= 0)
	{
	  Timer_cnt = 0;
	}
	
	if(++T100ms_cnt >= 100)
	{
	  T100ms_cnt = 0;
	  Timer_cnt = 100;
	}
	
	
	
	if(++time2_cnt >= 1000)
	{
	  time2_cnt = 0;
	  
	  if(num_cnt == 1)
	  {
	    num++;
		/*master_start++;
		if(master_start == 1)// 마스터 송수신 상태
		{
		  HAL_UART_Transmit(&huart1, (uint8_t *)ok_Buffer,2, 100);
		}*/
		if(master_start >= 2)
		{
		  master_start = 2;
		}
	  }
	  
	  /*if(ENQ != 0)
	  {
		if(BufferRx[0] == ENQ)
		{
		   Rx_Success_Flag = 1;
		}
	  }
	  
	  if(isMaster == false)
	  {
		error_flag++;
		if(error_flag == 5) // 5초 동안 데이터를 받아오지 못한다면 error 메세지 출력 및 LoRa_Master를 Tx로 변환 
		{
		  error_flag = 0;
		  error_msg = 1;
		  
		  //isMaster = true; // 주석 해제 해야해!!
		}
	  }*/
		//1sec task
		//sprintf(Kang_TX_buf, "%d", num);
		
		//tx_EN = 1;

		/*for(unsigned char i = 0; Kang_TX_buf[i]!=0x00; i++)
		{
			Log_tx_buf[i] = Kang_TX_buf[i];
			Log_tx_buf[i+1] = '\n';
			Log_tx_buf[i+2] = '\r';
		}*/
		if(++sec >= 60)
		{
			sec = 0;
			min++;
		}
		//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_RED */
	   RED_LED_TOGGLE; /* LED_RED Toggle */
	}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
