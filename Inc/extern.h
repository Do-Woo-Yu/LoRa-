/***************************Slave**********************************/
#include <stdbool.h>
/***********************환자 이름 데이터*******************************/
/*환자 이름은 한글자당 최대 2byte씩, 총 글자수는 6개까지 가능하다.( ex : 대한민국 사람 )
 SD 카드에 저장하는 이름 데이터의 종류는 CP949이며, 한글 길이는 최대 4자리 까지 가능하다.
LCD에 출력되는 한글 글자수는 최대 6자리까지 가능하며 UTF-8로 표시된다.
모터드라이버와 LCD간 UART 통신을 통해 데이터를 받아와서 글자를 출력할 때는 
UART는 1byte씩 받아옴으로 받아온 UTF-8 데이터 2개를 합쳐서 2byte로 만들어야 한다( 이게 한글자 )
또한 PC에서 모터 드라이브로 환자 정보 데이터를 보낼때는 UTF-8 형식으로 보내야한다.*/
/***********************환자 이름 데이터*******************************/

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;

/**lED 함수( RED, GREEN, BLUE )**/

#define BLUE_LED_ON			GPIOB->ODR |= GPIO_PIN_12;	 //PB12---->1
#define BLUE_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_12;	 //PB12---->0

#define GREEN_LED_ON		GPIOB->ODR |= GPIO_PIN_8;	 //PB8---->1
#define GREEN_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_8;	 //PB8---->0

#define RED_LED_TOGGLE		GPIOB->ODR ^= GPIO_PIN_2;	 //PB2---->0,1

/**lED 함수( RED, GREEN, BLUE )**/

extern byte ENQ; // Slave LoRa 국번

extern word LoRa_CRC_data, LoRa_temp;
extern byte Rx1_data, Rx_stop_flag,Send_EN;
extern word Rx1_index;
extern byte test_index, test_rxbuf[256];
extern byte CheckSum_ing;
extern byte Rx1_step, Rx1_next_data_no,Tx_Permit_cnt;
extern byte Data_Cnt; // 모터 드라이버에 보낼 데이터 갯수
extern word Rx1_data_sum;
extern word Error1_cnt;

extern char wireres_receive,Slave_LoRa_Tx_Mode_Flag;
extern int LoRa_send_MD_Flag;

extern void LoRa_Data_exe(void);
extern uint8_t Rcv1_cnt,Tx1_cnt;
extern byte Comp_ENQ,tx_delay_num;

extern unsigned int Tx_send_EN,Tx_send_flag;
extern uint8_t TX_BufferRx[255];

extern bool isMaster;
/***************************Slave**********************************/