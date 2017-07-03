/*
 * UART.c
 *
 *  Created on: 2017��5��15��
 *      Author: tt
 */
#include "UART.h"
#include <msp430.h>

void UART_INIT(){
    /*******************************************************************
         * init UART
         *******************************************************************/
        //====��������������IO�ڵĹ���========//
        P1SEL = BIT1 | BIT2;
        P1SEL2 = BIT1 | BIT2;
        //=====����UARTʱ��ԴΪ���þ����и��ߵ�׼ȷ��======//
        UCA0BR0 = 0x00 ;        //32.768k/115200=0.284          UCBRx=  INT(0.284)=0
        UCA0BR1 = 0x00;         //δ֪ʱ��Դ���趨   ACLK�� UCLK��
        UCA0CTL1 |= UCSSEL_1;   //ѡ������ʱ��Դ��ΪBRCLK
        //ACLK���÷�ʽΪ��UCA0BR1 = UCSSEL_1
        UCA0MCTL = UCBRS1 + UCBRS0;    //UCBRSx=round((0.284-0)x8)=round(2.272)=3
        UCA0CTL1 &= ~UCSWRST;          //��������λ
        IE2 |= UCA0RXIE ;    //ʹ�ý����ж�
        IFG2 &=~ UCA0TXIFG;
}
void UART_OnTX(unsigned char *pbuf,unsigned char length){
    unsigned char i;
    for(i=0;i<length;i++){
        if(*(pbuf + i)==0x00){
            break;  //����������Ϊ��ʱֱ���������������
        }else{
            while(UCA0STAT & UCBUSY);//�ȴ�UART�˿ڵ�����״̬
            UCA0TXBUF = *(pbuf + i);//����ǰ������װ�ص�BUF�д���
            *(pbuf + i) = 0x00;//��շ������� ��Ҫ����Ļ��ÿ������ָ��ǲ�һ���ģ�����������Ӱ��ϵͳ����
        }

        for(i=0;i<3;i++){
            while(UCA0STAT & UCBUSY);
            UCA0TXBUF = 0xFF;//��������0xFF�Խ���ָ��
        }
    }
}


void UART_OnRX(unsigned char *pbuf,unsigned char length){
    static unsigned char stopBitsCount=0;
    *(pbuf + recvBuffIndex) = UCA0RXBUF;
    if(*(pbuf + recvBuffIndex)==0xFF){
        stopBitsCount++;
    }else{
        stopBitsCount=0;
    }
    if(stopBitsCount >= 3 || recvBuffIndex >= length){
        getCmd=1;
    }else{
        getCmd=0;
    }
}


