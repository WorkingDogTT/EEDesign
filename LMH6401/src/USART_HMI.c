/*
 * USART_HMI.c
 *
 *  Created on: 2017年7月15日
 *      Author: tt
 */
#include "global.h"
void cmdMatch(void){
    if(recvBuff[0]==0x71){
        state=0x01;//进入设置状态
    }else{
        state=0x00;//默认状态
    }
}

