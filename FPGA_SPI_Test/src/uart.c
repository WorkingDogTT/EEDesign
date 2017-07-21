/*
 * usrt.c
 *
 *  Created on: 2017年7月15日
 *      Author: tt
 */
#include "global.h"


void UART_init(){
    /*******************************************************************
     * init UART
     *******************************************************************/
    //====启动并配置两个IO口的功能========//
        P1SEL |= BIT1 | BIT2;
        P1SEL2 |= BIT1 | BIT2;
        UCA0CTL1|=UCSWRST;
        /*选择USCI_Ax为UART模式*/
        UCA0CTL0 &= ~UCSYNC;
        //=====设置UART时钟源为外置晶振有更高的准确率======//
        //UCA0BR0 = 0x16 ;        //2M/115200=17.36          UCBRx=  INT(17.36)=17
        UCA0BR0=0x08;
        UCA0BR1 = 0x00;         //未知时钟源的设定   ACLK？ UCLK？
        UCA0CTL1 |= UCSSEL_3;   //选择MCLK时钟源作为BRCLK
        //ACLK设置方式为：UCA0BR1 = UCSSEL_1
        UCA0MCTL = UCBRS_0|UCBRF_11;    //UCBRSx=round((17.36-17)x8)=round(2.88)=3
        UCA0CTL1 &= ~UCSWRST;          //清除软件复位
        IE2 |= UCA0RXIE ;    //开启接收中断
        //清空接收中断标志
        IFG2 &= ~UCA0RXIFG;
}

//非常重要的是，这个函数去掉了0
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
    //自动加上三位0xFF
    for(i=0;i<3;i++){
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = 0xFF;
    }
}

void UART_OnRX(){
    recvBuff[RecvBuffIndex]=UCA0RXBUF;
    if(recvBuff[RecvBuffIndex]==0xFF) {
        stopbitsCount++;
    }else{
        stopbitsCount=0;
    }
    if(stopbitsCount>=3||RecvBuffIndex>=9){
        //cmdMatch();这个应用不需要管屏幕发出来的数据
        RecvBuffIndex=0;
    }else {
        RecvBuffIndex++;
    }
}


