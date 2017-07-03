/*
 * SystemUtils.c
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */
#include "global.h"

void __delay_us(unsigned char us){
    unsigned int i=0;
    unsigned int delay=us*(DelayMCLK_FREQ/10000000);
    for(i=0;i<delay;i++)
        __delay_cycles(DelayMCLK_FREQ/10000000);
}

