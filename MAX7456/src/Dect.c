/*
 * Dect.c
 *
 *  Created on: 2017��7��3��
 *      Author: tt
 */
#include "global.h"

void Dect_init(void){
    P1DIR &= ~BIT3;
}

void Dect(void){
    if((P1IN&BIT3)!=BIT3){
        write_node_B_clear();
    }else{
        write_node_B();
    }
}

