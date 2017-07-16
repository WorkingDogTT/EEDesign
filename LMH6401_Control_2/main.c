#include <msp430.h> 
#include "MSP430G2553.h"
#include "stdio.h"
#include "math.h"


/*
 * main.c
 */
void UART_OnTX(char *pbuf,unsigned char length);
void UART_OnRX();
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    /***************************************************************
     * init SystemClock
     **************************************************************/
    BCSCTL1 |= CALBC1_16MHZ; DCOCTL |= CALDCO_16MHZ;
    BCSCTL3 = XT2S_0 | LFXT1S_3 | XCAP_3;
    //BCSCTL3 = XT2S_0 | LFXT1S_0 | XCAP_2;
    /******************************************************************
     * init Resigter
     ******************************************************************/
    /*******************************************************************
     * init UART
     *******************************************************************/
    //====��������������IO�ڵĹ���========//
        P1SEL = BIT1 | BIT2;
        P1SEL2 = BIT1 | BIT2;
        //=====����UARTʱ��ԴΪ���þ����и��ߵ�׼ȷ��======//
        UCA0BR0 = 0x04 ;        //500k/115200=4.34          UCBRx=  INT(4.34)=4
        UCA0BR1 = 0x00;         //δ֪ʱ��Դ���趨   ACLK�� UCLK��
        UCA0CTL1 |= UCSSEL_1;   //ѡ������ʱ��Դ��ΪBRCLK
        //ACLK���÷�ʽΪ��UCA0BR1 = UCSSEL_1
        UCA0MCTL = UCBRS1 + UCBRS0;    //UCBRSx=round((4.34-4)x8)=round(2.72)=3
        UCA0CTL1 &= ~UCSWRST;          //��������λ
        IE2 |= UCA0RXIE ;    //���������ж�
    _enable_interrupts();
    while(1){
//        if(ampADResult>maxAmpADResult) maxAmpADResult=ampADResult;
//        if(ampADResult<minAmpADResult) minAmpADResult=ampADResult;
    }
}

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

void stateDect(){
    if(recvBuff[0]==0xFF){
                snprintf(sendBuff,sendBuffLength,"page 3");
                UART_OnTX(sendBuff,6);
                snprintf(sendBuff,sendBuffLength,"bug.txt=\"BEGIN recv 0xFF\"");
                UART_OnTX(sendBuff,25);
    }
    switch(state){
    case 0x00:
        //��ֹģʽ
        TA0CTL&=~MC_3;//���жϵ�˲��ֹͣ���еĶ�ʱ���Ͳ���
        TA1CTL&=~MC_3;
        ADC10CTL0 &=  ~(ADC10ON|ENC|ADC10SC);
        sinDecSum=102;
        RecvBuffIndex=0;
        sinIntInterval=0;
        sinIndex=0;
        Freq_now=10;
        stepInterval=0;
        ampADResult=0;
        maxAmpADResult=0;
        avePhaADResult=0;
        phaADResult=0;
        ampResult=0.0;
        phaResult=0.0;
        scan_Step=0;
        break;
    case 0xA1:
        //��Ƶģʽ
        if(recvBuff[0]==0x71){
            unsigned int tempFreq;
            tempFreq=recvBuff[1]+recvBuff[2]*256;
            float temp=0.0;//���������temp��ֵ
            temp=PointMax/((1/(float)Freq_now)/0.00001);
            Freq_now=(int)tempFreq;
            int setSinINTInterval=0;
            setSinINTInterval=(int)temp;
            int setSinDECInterval=0;
            setSinDECInterval=(int)(temp*1000.0-setSinINTInterval*1000);
            sinIntInterval=setSinINTInterval;
            sinDecInterval=setSinDECInterval;
            TA0CCR0=TA0CCR0_VAL;
            TA0CTL|=MC_1;
            TA1CCR0=65535;
            TA1CTL|=MC_1;//freqChange��������ɨƵʱ����Ƶ�ʲ����ĺ���ͬʱҲ���������㵱ǰ������������С������ֵ�ĺ���
            ADC10CTL0|=ADC10ON;
        }
        break;
    case 0xA2:
        //ɨƵģʽ
        if(recvBuff[0]==0x71){
            if(recvBuff[1]==0x01){
                scan_Step=10;
            }else if(recvBuff[1]==0x02){
                scan_Step=100;
            }
            Freq_now=10;
            TA0CCR0=TA0CCR0_VAL;
            TA1CCR0=TA1CCR0_VAL_LowFreq;
            TA0CTL|=MC_1;
            TA1CTL|=MC_1;
            ADC10CTL0|=ADC10ON;
        }
        break;
    case 0xB0:
        scan_Step=10;
        TA0CCR0=TA0CCR0_VAL;
        TA1CCR0=TA1CCR0_VAL_LowFreq;
        TA0CTL|=MC_1;
        TA1CTL|=MC_1;
        ADC10CTL0|=ADC10ON;
        break;
    default:
        break;
    }
}
inline void cmdMatch(){
    if(recvBuff[0]==0xA0){
        state=0x00;
        TA0CTL&=~MC_1;
        TA1CTL&=~MC_1;
        ADC10CTL0 &=  ~(ADC10ON|ENC|ADC10SC);
        sinDecSum=102;
        RecvBuffIndex=0;
        sinIntInterval=0;
        sinIndex=0;
        Freq_now=10;
        stepInterval=0;
        ampADResult=0;
        maxAmpADResult=0;
        avePhaADResult=0;
        phaADResult=0;
        ampResult=0.0;
        phaResult=0.0;
        scan_Step=0;
    }else if(recvBuff[0]==0xA1){
        state=0xA1;
    }else if(recvBuff[0]==0xA2){
        state=0xA2;
    }else if(recvBuff[0]==0xB0){
        state=0xB0;
    }else{
        stateDect();
    }
    RecvBuffIndex=0;
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
    if(stopbitsCount>=10||stopbitsCount<0){
        while(1){
            snprintf(sendBuff,sendBuffLength,"page 3");
            UART_OnTX(sendBuff,6);
            snprintf(sendBuff,sendBuffLength,"bug.txt=\"System Crash need PUC\"");
            UART_OnTX(sendBuff,25);
        }
    }
}
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    _disable_interrupts();
    TA0CTL&=~MC_3;
    TA1CTL&=~MC_3;
    ADC10CTL0&=~(ENC|ADC10ON|ADC10SC);
    //���뵽����״̬��ͣ�µ�ǰ���й���
    if (IFG2 & UCA0RXIFG) {
       IFG2&=~UCA0RXIFG;   // �ֶ������־λ
       UART_OnRX();// ����Tx�¼�������

    }
    else if (IFG2 & UCB0RXIFG) {
    }
    _enable_interrupts();
}





