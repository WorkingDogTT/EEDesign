/*
 * Dect.c
 *
 *  Created on: 2017��7��3��
 *      Author: tt
 */
#include "global.h"

void Dect_init(void){
    P1DIR &= ~BIT6;
}

void Dect(void){
    if((P1IN&BIT6)==BIT6){
        write_node_B_clear();
    }else{
        write_node_B();
    }
}

