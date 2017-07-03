#include <msp430.h> 
#include "SPI.h"
#include "UART.h"


/*
 * main.c
 */
//����ȫ�ֱ���
int MODE=0;//ģʽ����   Ĭ��Ϊ0ģʽ  �ֶ�����ģʽ
unsigned char recvBuff[10]={0};//������ջ���  ����Ļ��õĽ�����Ϣ
unsigned char sendBuff[20]={0};//���巢�ͻ���  ����Ļ���͵ķ�����Ϣ
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
    //������ʼ��DAC�Ĳ���

    //Power up
    dacData=0x03;
    controldata=0x20;//power up DAC-A and DAC-B
    sendDACData(dacData,controldata);
    dacData=0x01;
    controldata=0xFF;//CMDΪ111 �����ڲ���׼Դ������������Ϊ2
    sendDACData(dacData,controldata);
    dacData=0x03;
    controldata=0x02;//set DAC-A DAC-B gain to 1
    sendDACData(dacData,controldata);
    while(1){
        //ϵͳ��Ҫ������
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
        UART_OnTX(sendBuff, 20);//UART�����ж�
    }else if (IFG2 & UCB0TXIFG) {
       //----�����ж��¼�����--------//
       SPI_TxISR();   //SPI�����ж�
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void){
    if(IFG2 & UCA0RXIFG){    //UART�Ľ����ж�
        UART_OnRX(recvBuff,10);
    }else if (IFG2 & UCB0RXIFG){  //SPI�Ľ����ж�
        SPI_RxISR();
    }
}
