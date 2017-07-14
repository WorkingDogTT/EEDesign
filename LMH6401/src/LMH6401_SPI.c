/*
 * MAX_SPI.c
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */
#include "global.h"


unsigned char write_lmh6401(unsigned char *address, unsigned char *data){

    unsigned char sendBuff[2];
    sendBuff[0]=(*address);
    sendBuff[1]=(*data);
    SPI_CS_High();
    __delay_cycles(16);
    SPI_CS_Low();
    while(!SPI_TxFrame(sendBuff,2));
    while(UCA0STAT & UCBUSY);
    SPI_CS_High();
    return 1;
}

unsigned char read_lmh6401(unsigned char *address, unsigned char *data){
    SPI_CS_High();
   __delay_cycles(16);
   SPI_CS_Low();
    while(!SPI_TxFrame(address,1));
    while(UCA0STAT & UCBUSY);
    while(!SPI_RxFrame(data,1));
    while(UCA0STAT & UCBUSY);
    SPI_CS_High();
    return 1;
}

