/*
 * USART_HMI.c
 *
 *  Created on: 2017��7��15��
 *      Author: tt
 */
#include "global.h"
void cmdMatch(void){
    if(recvBuff[0]==0x71){
        state=0x01;//��������״̬
    }else{
        state=0x00;//Ĭ��״̬
    }
}

