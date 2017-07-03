/*
 * SystemUtils.c
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */
#include "global.h"

void __delay_us(unsigned int us){
    unsigned int i=0;
    unsigned int delay=us*(DelayMCLK_FREQ/16000000);
    for(i=0;i<delay;i++)
        __delay_cycles(DelayMCLK_FREQ/16000000);
}

void __delay_ms(unsigned int ms){
    unsigned int i=0;
    unsigned int delay=ms*(DelayMCLK_FREQ/16000000);
    for(i=0;i<delay;i++)
        __delay_cycles(DelayMCLK_FREQ/16000);
}

void __delay_s(unsigned int s){
    unsigned int i=0;
    unsigned int delay=s*(DelayMCLK_FREQ/16000000);
    for(i=0;i<delay;i++)
        __delay_cycles(DelayMCLK_FREQ/16);
}



