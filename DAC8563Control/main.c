#include <msp430.h> 
#include "SPI.h"
#include "UART.h"


/*
 * main.c
 */
//定义全局变量
int MODE=0;//模式定义   默认为0模式  手动控制模式
unsigned char recvBuff[10]={0};//定义接收缓冲  从屏幕获得的接收信息
unsigned char sendBuff[20]={0};//定义发送缓冲  向屏幕传送的发送信息
unsigned int  dacData=0;
unsigned char controldata=0;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    BCSCTL1 |= CALBC1_16MHZ;//Setup internal DCO clock to 16MHZ
	DCOCTL |= CALDCO_16MHZ;
	BCSCTL3 = XT2S_0 | LFXT1S_0 | XCAP_1;//Using internal VLO 12MHz clock as ACLK
	BCSCTL2 |= SELM_0;//MCLK using internal DCO
    getCmd=0;
    recvBuffIndex=0;
    P2DIR |= BIT2;
    P2OUT &=~BIT2;
    SPI_INIT();
    UART_INIT();
    _enable_interrupts();
    //几个初始化DAC的步骤

    //Power up
    dacData=0x03;
    controldata=0x20;//power up DAC-A and DAC-B
    sendDACData(dacData,controldata);
    dacData=0x01;
    controldata=0xFF;//CMD为111 设置内部基准源并将增益设置为2
    sendDACData(dacData,controldata);
    dacData=0x03;
    controldata=0x02;//set DAC-A DAC-B gain to 1
    sendDACData(dacData,controldata);
    while(1){
        //系统主要处理环节
        if(getCmd){
            dacData=((unsigned int)recvBuff[1])|(((unsigned int)recvBuff[2])<<8);
            controldata = 0x1F;
            sendDACData(dacData,controldata);
        }else{
            P2OUT |= BIT2;
            controldata = 0x0F;
            sendDACData(dacData,controldata);
            P2OUT &=~BIT2;
        }
    }
	return 0;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR_HOOK(void){
    if (IFG2 & UCA0TXIFG) {
        UART_OnTX(sendBuff, 20);//UART发送中断
    }else if (IFG2 & UCB0TXIFG) {
       //----发送中断事件处理--------//
       SPI_TxISR();   //SPI发送中断
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void){
    if(IFG2 & UCA0RXIFG){    //UART的接收中断
        UART_OnRX(recvBuff,10);
    }else if (IFG2 & UCB0RXIFG){  //SPI的接收中断
        SPI_RxISR();
    }
}
