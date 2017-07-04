/*
 * SPI.h
 *
 *  Created on: 2017年7月3日
 *      Author: tt
 */

#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#define   HARD_SPI    //条件编译，HARD_SPI启用硬件SPI代码
//#define SOFT_SPI            //条件编译，SOFT_SPI启用软件SPI代码
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
