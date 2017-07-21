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
void display(void){
    int i=0;
    sendBuffLength=25;
    //���ȷ���Ƶ��
    //Zhenshu = (unsigned long)Freq_250_result;
    Zhenshu = Signal_Rise_Count;
    Xiaoshu = 0;
    //Xiaoshu = (unsigned long)((double)(((double)((double)Freq_250_result-(double)Zhenshu))*10000.0));
    long_temp0 = Zhenshu;
    //bytes[0] = long_temp0/100000000;
    //long_temp0 = long_temp0 - bytes[0]*100000000;
    bytes[8] = long_temp0/100000000;
    long_temp0 = long_temp0 - bytes[8]*100000000;
    bytes[7] = long_temp0/10000000;
    long_temp0 = long_temp0 - bytes[7]*10000000;
    bytes[6] = long_temp0/1000000;
    long_temp0 = long_temp0 - bytes[6]*1000000;
    bytes[5] = long_temp0/100000;
    long_temp0 = long_temp0 - bytes[5]*100000;
    bytes[4] = long_temp0/10000;
    long_temp0 = long_temp0 - (unsigned long)bytes[4]*(unsigned long)10000;
    bytes[3] = long_temp0/1000;
    long_temp0 = long_temp0 - bytes[3]*1000;
    bytes[2] = long_temp0/100;
    long_temp0 = long_temp0 - bytes[2]*100;
    bytes[1] = long_temp0/10;
    long_temp0 = long_temp0 - bytes[1]*10;
    bytes[0] = long_temp0;


    snprintf(sendBuff,30,"t0.txt=\"%d%d%d,%d%d%d,%d%d%d.%dHz\"",bytes[8],bytes[7],bytes[6],bytes[5],bytes[4],bytes[3],bytes[2],bytes[1],bytes[0],(unsigned int)Xiaoshu);
    UART_OnTX(sendBuff,30);

    //����������ռ�ձ�
    Zhenshu = (unsigned long)Duty_Result;
    Xiaoshu = (unsigned long)((Duty_Result-Zhenshu)*10000.0);

    snprintf(sendBuff,30,"t1.txt=\"%d.%d%%\"",(unsigned int)Zhenshu,(unsigned int)Xiaoshu);
    UART_OnTX(sendBuff,30);
}

