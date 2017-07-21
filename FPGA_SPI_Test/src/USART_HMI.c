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
void display(void){
    int i=0;
    sendBuffLength=25;
    //首先发送频率
    Zhenshu = (unsigned long)Freq_250_result;
    Xiaoshu = (unsigned long)((Freq_250_result-Zhenshu)*10000.0);
    long_temp0 = Zhenshu;
    //bytes[0] = long_temp0/100000000;
    //long_temp0 = long_temp0 - bytes[0]*100000000;
    for(i=8;i>=0;i--){
        bytes[i] = long_temp0 / pow(10,i);
        long_temp0 = long_temp0 - bytes[i]*pow(10,i);
    }
    snprintf(sendBuff,25,"t0.txt=%d%d%d%d%d%d%d%d%d,%d",bytes[8],bytes[7],bytes[6],bytes[5],bytes[4],bytes[3],bytes[2],bytes[1],bytes[0],(unsigned int)Xiaoshu);
    UART_OnTX(sendBuff,25);

    //接下来发送占空比
    Zhenshu = (unsigned long)Duty_Result;
    Xiaoshu = (unsigned long)((Duty_Result-Zhenshu)*10000.0);

    snprintf(sendBuff,25,"t1.txt=%d.%d",(unsigned int)Zhenshu,(unsigned int)Xiaoshu);
    UART_OnTX(sendBuff,25);
}

