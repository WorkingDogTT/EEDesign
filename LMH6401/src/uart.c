/*
 * usrt.c
 *
 *  Created on: 2017��7��15��
 *      Author: tt
 */
#include "global.h"


void UART_init(){
    /*******************************************************************
     * init UART
     *******************************************************************/
    //====��������������IO�ڵĹ���========//
        P1SEL |= BIT1 | BIT2;
        P1SEL2 |= BIT1 | BIT2;
        UCA0CTL1|=UCSWRST;
        /*ѡ��USCI_AxΪUARTģʽ*/
        UCA0CTL0 &= ~UCSYNC;
        //=====����UARTʱ��ԴΪ���þ����и��ߵ�׼ȷ��======//
        UCA0BR0 = 0x10 ;        //2M/115200=17.77          UCBRx=  INT(17.77)=17
        UCA0BR1 = 0x00;         //δ֪ʱ��Դ���趨   ACLK�� UCLK��
        UCA0CTL1 |= UCSSEL_3;   //ѡ��MCLKʱ��Դ��ΪBRCLK
        //ACLK���÷�ʽΪ��UCA0BR1 = UCSSEL_1
        UCA0MCTL = 0x01;    //UCBRSx=round((17.77-17)x8)=round(6.16)=7
        UCA0CTL1 &= ~UCSWRST;          //��������λ
        IE2 |= UCA0RXIE ;    //���������ж�
        //��ս����жϱ�־
        IFG2 &= ~UCA0RXIFG;
}

//�ǳ���Ҫ���ǣ��������ȥ����0
void UART_OnTX(char *pbuf,unsigned char length){
    unsigned char i;
    for(i=0;i<length;i++){
        if(*(pbuf + i)==0x00){
            break;
        }else{
            while(UCA0STAT & UCBUSY);
            UCA0TXBUF = *(pbuf + i);
            *(pbuf + i)=0x00;
        }

    }
    for(i=0;i<3;i++){
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = 0xFF;
    }
}

void UART_OnRX(){
    static int stopbitsCount=0;
    recvBuff[RecvBuffIndex]=UCA0RXBUF;
    if(recvBuff[RecvBuffIndex]==0xFF) {
        stopbitsCount++;
    }else{
        stopbitsCount=0;
    }
    if(stopbitsCount>=3||RecvBuffIndex>=9){
        cmdMatch();
    }else {
        RecvBuffIndex++;
    }
}


