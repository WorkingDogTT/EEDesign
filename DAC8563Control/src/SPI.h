/*
 * SPI.h
 *
 *  Created on: 2017��5��13��
 *      Author: tt
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

//------Ӳ��SPI�ܽŵĺ궨��---------------//
#define SPI_SIMO  BIT7    //P1.7ΪSIMO��
#define SPI_SOMI  BIT6    //P1.6λSOMI��
#define SPI_CLK   BIT5    //P1.5ΪCLK��
#define SPI_CS   BIT4    //P1.4ΪSTEƬѡ�źſ�

//------Ӳ��SPI���ƶ˿ڵĺ궨��----------//
#define SPI_SEL2  P1SEL2
#define SPI_SEL   P1SEL
#define SPI_DIR   P1DIR
#define SPI_OUT   P1OUT
#define SPI_REN   P1REN

//-----ʹ�ܶ�CS�˿ں궨��--------------//
#define SPI_CS_SEL2   P2SEL
#define SPI_CS_SEL    P2SEL
#define SPI_CS_OUT    P2OUT
#define SPI_CS_DIR    P2DIR

//-----���巢��/���ջ���---------------//
unsigned char *SPI_Tx_Buffer;
unsigned char *SPI_Rx_Buffer;

//-----���������/���յ��ֽ���---------//
unsigned char SPI_Tx_Size;
unsigned char SPI_Rx_Size;
//-----���巢��/����ģʽ��ʶ----------//
unsigned char SPI_Rx_Or_Tx;        //0 ������  �� 1  ������  �� 2 �շ�ģʽ


extern void SPI_INIT();//SPI�ӿڵĳ�ʼ������
extern void sendDACData(unsigned int data,unsigned char control_bits);
extern void SPI_CS_HIGH();
extern void SPI_CS_LOW();
void SPI_Interrupt_Sel(unsigned char onOff);
unsigned char SPI_RxFrame(unsigned char *pBuffer, unsigned int size);
unsigned char SPI_TxFrame(unsigned char *pBuffer, unsigned int size);
extern void SPI_TxISR();
extern void SPI_RxISR();
#endif /* SRC_SPI_H_ */
