/*
 * SPI.h
 *
 *  Created on: 2017��7��3��
 *      Author: tt
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#define   HARD_SPI    //�������룬HARD_SPI����Ӳ��SPI����
//#define SOFT_SPI            //�������룬SOFT_SPI�������SPI����
unsigned char  SPI_Tx_Size;
unsigned char  SPI_Rx_Size;

extern void SPI_CS_High(void);
extern void SPI_CS_Low(void);
extern void SPI_CLK_High(void);
extern void SPI_CLK_Low(void);
extern void SPI_HighSpeed(void);
extern void SPI_LowSpeed(void);
extern void SPI_init(void);
extern unsigned char SPI_TxFrame(unsigned char  *pBuffer, unsigned int   size);
extern unsigned char SPI_RxFrame(unsigned char  *pBuffer, unsigned int size);


#endif /* SRC_SPI_H_ */
