/*
 * Dect.c
 *
 *  Created on: 2017��7��3��
 *      Author: tt
 */
#include "global.h"

void Dect_init(void){
    P1DIR &= ~BIT3;
    P1OUT&=~BIT3;
}

void Dect(void){
    test_lmh6401();

}

