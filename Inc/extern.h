/***************************Slave**********************************/
#include <stdbool.h>
/***********************ȯ�� �̸� ������*******************************/
/*ȯ�� �̸��� �ѱ��ڴ� �ִ� 2byte��, �� ���ڼ��� 6������ �����ϴ�.( ex : ���ѹα� ��� )
 SD ī�忡 �����ϴ� �̸� �������� ������ CP949�̸�, �ѱ� ���̴� �ִ� 4�ڸ� ���� �����ϴ�.
LCD�� ��µǴ� �ѱ� ���ڼ��� �ִ� 6�ڸ����� �����ϸ� UTF-8�� ǥ�õȴ�.
���͵���̹��� LCD�� UART ����� ���� �����͸� �޾ƿͼ� ���ڸ� ����� ���� 
UART�� 1byte�� �޾ƿ����� �޾ƿ� UTF-8 ������ 2���� ���ļ� 2byte�� ������ �Ѵ�( �̰� �ѱ��� )
���� PC���� ���� ����̺�� ȯ�� ���� �����͸� �������� UTF-8 �������� �������Ѵ�.*/
/***********************ȯ�� �̸� ������*******************************/

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;

/**lED �Լ�( RED, GREEN, BLUE )**/

#define BLUE_LED_ON			GPIOB->ODR |= GPIO_PIN_12;	 //PB12---->1
#define BLUE_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_12;	 //PB12---->0

#define GREEN_LED_ON		GPIOB->ODR |= GPIO_PIN_8;	 //PB8---->1
#define GREEN_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_8;	 //PB8---->0

#define RED_LED_TOGGLE		GPIOB->ODR ^= GPIO_PIN_2;	 //PB2---->0,1

/**lED �Լ�( RED, GREEN, BLUE )**/

extern byte ENQ; // Slave LoRa ����

extern word LoRa_CRC_data, LoRa_temp;
extern byte Rx1_data, Rx_stop_flag,Send_EN;
extern word Rx1_index;
extern byte test_index, test_rxbuf[256];
extern byte CheckSum_ing;
extern byte Rx1_step, Rx1_next_data_no,Tx_Permit_cnt;
extern byte Data_Cnt; // ���� ����̹��� ���� ������ ����
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