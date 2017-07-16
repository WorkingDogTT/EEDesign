/*
 * MAX7456.c
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */
#include "global.h"
void test_lmh6401(void){
    address=0x80;
   data=0x10;
   read_lmh6401(&address,&data);
   data=data;
   _nop();
    address=0x02;
    data=control_data;
   //read_lmh6401(&address,&data);
    write_lmh6401(&address,&data);
    address=0x04;
    data=0x32;
    write_lmh6401(&address,&data);
    address=0x05;
    data=0x52;
    write_lmh6401(&address,&data);
    _nop();
}

