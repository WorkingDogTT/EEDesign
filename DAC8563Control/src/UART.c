/*
 * UART.c
 *
 *  Created on: 2017年5月15日
 *      Author: tt
 */
#include "UART.h"
#include <msp430.h>

void UART_INIT(){
    /*******************************************************************
         * init UART
         *******************************************************************/
        //====启动并配置两个IO口的功能========//
        P1SEL = BIT1 | BIT2;
        P1SEL2 = BIT1 | BIT2;
        //=====设置UART时钟源为外置晶振有更高的准确率======//
        UCA0BR0 = 0x00 ;        //32.768k/115200=0.284          UCBRx=  INT(0.284)=0
        UCA0BR1 = 0x00;         //未知时钟源的设定   ACLK？ UCLK？
        UCA0CTL1 |= UCSSEL_1;   //选择外置时钟源作为BRCLK
        //ACLK设置方式为：UCA0BR1 = UCSSEL_1
        UCA0MCTL = UCBRS1 + UCBRS0;    //UCBRSx=round((0.284-0)x8)=round(2.272)=3
        UCA0CTL1 &= ~UCSWRST;          //清除软件复位
        IE2 |= UCA0RXIE ;    //使用接收中断
        IFG2 &=~ UCA0TXIFG;
}
void UART_OnTX(unsigned char *pbuf,unsigned char length){
    unsigned char i;
    for(i=0;i<length;i++){
        if(*(pbuf + i)==0x00){
            break;  //当发送内容为空时直接跳过后面的内容
        }else{
            while(UCA0STAT & UCBUSY);//等待UART端口到空闲状态
            UCA0TXBUF = *(pbuf + i);//将当前的内容装载到BUF中传送
            *(pbuf + i) = 0x00;//清空发送内容 主要是屏幕的每条传送指令都是不一样的，多余的清除会影响系统运作
        }

        for(i=0;i<3;i++){
            while(UCA0STAT & UCBUSY);
            UCA0TXBUF = 0xFF;//发送三个0xFF以结束指令
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


