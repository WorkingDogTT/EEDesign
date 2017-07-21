/*
 * FPGA_SPI.c
 *
 *  Created on: 2017Äê7ÔÂ21ÈÕ
 *      Author: tt
 */

#include "global.h"
unsigned char write_FPGA(unsigned char *address, unsigned char *data){
    SPI_sendBuff[0]=(*address);
    SPI_sendBuff[1]=(*data);
    SPI_CS_High();
    __delay_cycles(16);
    SPI_CS_Low();
    while(!SPI_TxFrame(SPI_sendBuff,2));
    while(UCB0STAT & UCBUSY);
    SPI_CS_High();
    return 1;
}
unsigned char read_FPGA(unsigned char *address, unsigned char *data){
    SPI_CS_High();
   __delay_cycles(16);
   SPI_CS_Low();
    while(!SPI_TxFrame(address,1));
    while(UCB0STAT & UCBUSY);
    while(!SPI_RxFrame(data,1));
    while(UCB0STAT & UCBUSY);
    SPI_CS_High();
    return 1;
}
