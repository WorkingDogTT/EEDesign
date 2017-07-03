/*
 * SPI.h
 *
 *  Created on: 2017年5月13日
 *      Author: tt
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

//------硬件SPI管脚的宏定义---------------//
#define SPI_SIMO  BIT7    //P1.7为SIMO口
#define SPI_SOMI  BIT6    //P1.6位SOMI口
#define SPI_CLK   BIT5    //P1.5为CLK口
#define SPI_CS   BIT4    //P1.4为STE片选信号口

//------硬件SPI控制端口的宏定义----------//
#define SPI_SEL2  P1SEL2
#define SPI_SEL   P1SEL
#define SPI_DIR   P1DIR
#define SPI_OUT   P1OUT
#define SPI_REN   P1REN

//-----使能端CS端口宏定义--------------//
#define SPI_CS_SEL2   P2SEL
#define SPI_CS_SEL    P2SEL
#define SPI_CS_OUT    P2OUT
#define SPI_CS_DIR    P2DIR

//-----定义发送/接收缓存---------------//
unsigned char *SPI_Tx_Buffer;
unsigned char *SPI_Rx_Buffer;

//-----定义待发送/接收的字节数---------//
unsigned char SPI_Tx_Size;
unsigned char SPI_Rx_Size;
//-----定义发送/接收模式标识----------//
unsigned char SPI_Rx_Or_Tx;        //0 仅接收  ； 1  仅发送  ； 2 收发模式


extern void SPI_INIT();//SPI接口的初始化函数
extern void sendDACData(unsigned int data,unsigned char control_bits);
extern void SPI_CS_HIGH();
extern void SPI_CS_LOW();
void SPI_Interrupt_Sel(unsigned char onOff);
unsigned char SPI_RxFrame(unsigned char *pBuffer, unsigned int size);
unsigned char SPI_TxFrame(unsigned char *pBuffer, unsigned int size);
extern void SPI_TxISR();
extern void SPI_RxISR();
#endif /* SRC_SPI_H_ */
