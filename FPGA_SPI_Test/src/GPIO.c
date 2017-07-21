/*
 * GPIO.c
 *
 *  Created on: 2017Äê7ÔÂ21ÈÕ
 *      Author: tt
 */
#include "global.h"
void init_GPIO(void){
    P2DIR |= BIT0 | BIT1;
    P2OUT &= ~( BIT0 | BIT1 );
    P2IE |= BIT0 | BIT1;
    P2IES &= ~(BIT0 | BIT1);
    P2IFG &= ~(BIT0 | BIT1);
}

