/*
 * Dect.c
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */
#include "global.h"

void Dect_init(void){
    P1DIR &= ~BIT3;
    P1OUT&=~BIT3;
}

void Dect(void){
    //test_node_B();
    //write_node_B();

    if((P1IN&BIT3)){
        write_node_C();
    }else{
        write_node_C_clear();
    }

}

