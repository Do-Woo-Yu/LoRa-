/***************************Mater**********************************/
#include <stdbool.h>
/***********************ȯ�� �̸� ������*******************************/
/*ȯ�� �̸��� �ѱ��ڴ� �ִ� 2byte��, �� ���ڼ��� 6������ �����ϴ�.( ex : ���ѹα� ��� )
 SD ī�忡 �����ϴ� �̸� �������� ������ CP949�̸�, �ѱ� ���̴� �ִ� 4�ڸ� ���� �����ϴ�.
LCD�� ��µǴ� �ѱ� ���ڼ��� �ִ� 6�ڸ����� �����ϸ� UTF-8�� ǥ�õȴ�.
���͵���̹��� LCD�� UART ����� ���� �����͸� �޾ƿͼ� ���ڸ� ����� ���� 
UART�� 1byte�� �޾ƿ����� �޾ƿ� UTF-8 ������ 2���� ���ļ� 2byte�� ������ �Ѵ�( �̰� �ѱ��� )
���� PC���� ���� ����̺�� ȯ�� ���� �����͸� �������� UTF-8 �������� �������Ѵ�.*/
/***********************ȯ�� �̸� ������*******************************/

typedef unsigned short word;
typedef unsigned char byte;

/***************************Mater**********************************/

/**lED �Լ�( RED, GREEN, BLUE )**/

#define BLUE_LED_ON			GPIOB->ODR |= GPIO_PIN_12;	 //PB12---->1
#define BLUE_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_12;	 //PB12---->0

#define GREEN_LED_ON		GPIOB->ODR |= GPIO_PIN_8;	 //PB8---->1
#define GREEN_LED_OFF		GPIOB->ODR &= ~GPIO_PIN_8;	 //PB8---->0

#define RED_LED_TOGGLE		GPIOB->ODR ^= GPIO_PIN_2;	 //PB2---->0,1

/**lED �Լ�( RED, GREEN, BLUE )**/

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

extern void UART1_rd_exe(void); // �б� ��û �Լ�
extern void UART1_wr_exe(void); // ���� ��û �Լ�

extern unsigned char LoRa_TX_buf[255];

extern char Master_State[3]; // ���ڿ��� ������ �� �ִ� ���� �迭 ��

extern unsigned int Dial_Num; // PC�� ��µǴ� ����

extern uint8_t ENQ_Array[100];