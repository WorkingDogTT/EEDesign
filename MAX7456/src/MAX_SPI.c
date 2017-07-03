/*
 * MAX_SPI.c
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */
#include "global.h"


unsigned char write_max7456(unsigned char *address, unsigned char *data){
    (*address)=(*address)&0x7F;
    SPI_TxFrame(address,8);
    SPI_TxFrame(data,8);
    return 1;
}

unsigned char read_max7456(unsigned char *address, unsigned char *data){
    (*address)=(*address) | 0x80;
    SPI_TxFrame(address,8);
    SPI_RxFrame(data,8);
    return 1;
}

