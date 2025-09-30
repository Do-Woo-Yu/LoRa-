/***************************Mater**********************************/
#include <stdbool.h>
/***********************환자 이름 데이터*******************************/
/*환자 이름은 한글자당 최대 2byte씩, 총 글자수는 6개까지 가능하다.( ex : 대한민국 사람 )
 SD 카드에 저장하는 이름 데이터의 종류는 CP949이며, 한글 길이는 최대 4자리 까지 가능하다.
LCD에 출력되는 한글 글자수는 최대 6자리까지 가능하며 UTF-8로 표시된다.
모터드라이버와 LCD간 UART 통신을 통해 데이터를 받아와서 글자를 출력할 때는 
UART는 1byte씩 받아옴으로 받아온 UTF-8 데이터 2개를 합쳐서 2byte로 만들어야 한다( 이게 한글자 )
또한 PC에서 모터 드라이브로 환자 정보 데이터를 보낼때는 UTF-8 형식으로 보내야한다.*/
/***********************환자 이름 데이터*******************************/

typedef unsigned short word;
typedef unsigned char byte;

/***************************Mater**********************************/

/**lED 함수( RED, GREEN, BLUE )**/

#define BLUE_LED_ON			GPIOB->ODR |= GPIO_PIN_12;	 //PB12---->1
#define BLUE_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_12;	 //PB12---->0

#define GREEN_LED_ON		GPIOB->ODR |= GPIO_PIN_8;	 //PB8---->1
#define GREEN_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_8;	 //PB8---->0

#define RED_LED_TOGGLE		GPIOB->ODR ^= GPIO_PIN_2;	 //PB2---->0,1

/**lED 함수( RED, GREEN, BLUE )**/

extern byte ENQ,R_or_W;
extern word Rcv2_ok;
extern byte RS232_dead_time;
extern uint8_t rx_one_data,rx1_index;
extern word CRC_data;
extern word w_temp;
extern uint8_t Master_BackupTxBuffer[255];
extern uint8_t Master_BackupRxBuffer[255];

extern byte Rx2_index, Rx2_next_data_no, Rcv2_cnt;
extern word LoRa_CheckSum_data;
extern word Error2_cnt;
extern byte Rx2_step;

extern byte Rx2_Checksum_H, Rx2_Checksum_L;
extern byte LoRa_CheckSum_ing;
extern byte LoRa_CheckSum_EN;

extern byte LoRa_Rcv_ok;

extern word LoRa_test_crc1, LoRa_test_crc2;
extern byte LoRa_Tx_buf[100];
extern byte LoRa_Tx_index;
extern byte LoRa_Tx_send_number;
extern byte Tx2_index, Tx2_send_number;
extern uint8_t rx_one_data,rx1_index;
extern word CRC_data;
extern word w_temp;

extern byte wireres_receive,Send_EN,Tx_cnt,rx_end_delay_num,LoRa_Rx_ing;

extern int Rx_Success_Flag,Rx_Delay_cnt;

extern word CRC16(byte *Data, word Len);

extern void LoRa_Data_exe(void);

extern uint8_t BufferRx[255], BufferTx[255],Tx1_buf[255],Tx1_send_number,Tx1_index,Rx1_index;

extern uint8_t Rx1_buf[255];

extern int8_t RssiValue;

extern bool isMaster;

extern unsigned int error_msg,Tx_end_recieve_EN,Tx_end_recieve_flag,Rx_go_cnt,Again_Tx_Flag,test_again_send_en,Tx_go_cnt;

extern unsigned int scan, reset_flag, Tx_Success, Rx_Success, Rx_Fail, Tx_Ready, Tx_Fail, Tx_Stop, Rx_Stop, error_msg,error_flag,rx_end_num;
extern uint8_t tx_EN,display_PC;
extern uint8_t Rx_buf[20];
extern uint8_t ok_Buffer[5];
extern uint8_t go_Buffer[5];

extern uint8_t Tx_Ready_Buffer[10];
extern uint8_t Tx_Success_Buffer[12];
extern uint8_t Tx_Fail_Buffer[9];
extern uint8_t Rx_Success_Buffer[12];
extern uint8_t Rx_Fail_Buffer[9];
extern uint8_t Tx_Stop_Buffer[9];
extern uint8_t Rx_Stop_Buffer[9];
extern uint8_t error_msg_Buffer[79];

extern word CRC16(byte *Data, word Len);
extern word LoRa_Master_CRC16,MD_CRC16,LoRa_Master_CRC16_Data,MD_CRC16_Data;

extern void UART1_rd_exe(void); // 읽기 요청 함수
extern void UART1_wr_exe(void); // 쓰기 요청 함수

extern unsigned char LoRa_TX_buf[255];

extern char Master_State[3]; // 문자열을 저장할 수 있는 문자 배열 정

extern unsigned int Dial_Num; // PC에 출력되는 국번

extern uint8_t ENQ_Array[100];