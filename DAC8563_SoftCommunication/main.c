#include <msp430.h> 
#include "MSP430G2553.h"
#define DAC_DATA BIT0  //P2.0 DIN
#define DAC_SCLK BIT1  //P2.1 SCLK
#define DAC_SYNC BIT2  //P2.2 SYNC
#define DAC_LOAD BIT4  //P2.4 LOAD
#define DAC_CLR  BIT3  //P2.3 CLR

#define SPI_DIR  P2DIR
#define SPI_OUT  P2OUT


#define DATA_LOW P2OUT&=~BIT0
#define DATA_HIGH P2OUT|=BIT0
#define SCLK_LOW P2OUT&=~BIT1
#define SCLK_HIGH P2OUT|=BIT1
#define SYNC_LOW P2OUT&=~BIT2
#define SYNC_HIGH P2OUT|=BIT2
#define LOAD_LOW P2OUT&=~BIT4
#define LOAD_HIGH P2OUT|=BIT4
#define CLR_LOW P2OUT&=~BIT3
#define CLR_HIGH P2OUT|=BIT3

unsigned int dataBITS=0;
unsigned char controlBITS=0;

unsigned char recvBuff[10]={0};
unsigned char sendBuff[20]={0};

unsigned char DACSETMODE=0x00;


unsigned char recvBuffIndex;

unsigned char getCmd=0;
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
        getCmd=1;//��־��ǰ�յ���һ������ָ��
        recvBuffIndex=0;//��ս��ջ���
        if(*(pbuf)==0xAA){
            DACSETMODE=0xAA;//������ǰ���õ���DAC-A
        }
        if(*(pbuf)==0xBB){
            DACSETMODE=0xBB;//������ǰ���õ���DAC-B
        }
    }else{
        getCmd=0;
    }
    recvBuffIndex++;
}

void sendDATA(unsigned int p){
    int index=0;
    for(index=0;index<16;index++){
        SCLK_HIGH;
        __delay_cycles(1);
        if((p<<index)&0xC000){
            DATA_HIGH;
        }else{
            DATA_LOW;
        }
        __delay_cycles(1);
        SCLK_HIGH;
        __delay_cycles(1);
    }
}

void sendCONTROL(unsigned char p){
    int index2=0;
    for(index2=0;index2<8;index2++){
        SCLK_LOW;
        __delay_cycles(1);
        if((p<<index2)&0xC0){
            DATA_HIGH;
        }else{
            DATA_LOW;
        }
        __delay_cycles(1);
        SCLK_HIGH;
        __delay_cycles(1);
    }
}
void DAC_INIT(){
    SPI_DIR |= DAC_DATA | DAC_SCLK | DAC_SYNC | DAC_LOAD | DAC_CLR;
    SPI_OUT |= DAC_CLR | DAC_SCLK;//ʱ���ź����κ�ʱ������ߵ�ƽ  ���λ�ڳ�ʼ���Ǳ���Ϊ��

    /*
     * ��ʼ��DACоƬ
     */
    //POWER UP DAC-A DAC-B
    dataBITS=0x0003;
    controlBITS=0x20;
    sendCONTROL(controlBITS);
    sendDATA(dataBITS);

    //INACTIVE LOAD
    dataBITS=0x0003;
    controlBITS=0x30;
    sendCONTROL(controlBITS);
    sendDATA(dataBITS);
    //Enable internal Ref.
    dataBITS=0x0001;
    controlBITS=0x3C;
    sendCONTROL(controlBITS);
    sendDATA(dataBITS);
    //Set GAIN=1
    dataBITS=0x0003;
    controlBITS=0x02;
    sendCONTROL(controlBITS);
    sendDATA(dataBITS);
    //Set DAC-A =0 Set DAC-B=32767
    dataBITS=0x0000;
    controlBITS=0x1C;
    sendCONTROL(controlBITS);
    sendDATA(dataBITS);
    //Set DAC-B = 32767
    dataBITS=0xC000;
    controlBITS=0x1D;
    sendCONTROL(controlBITS);
    sendDATA(dataBITS);
}

void CLK_INIT(){
    BCSCTL1 |= CALBC1_16MHZ;//Setup internal DCO clock to 16MHZ
    DCOCTL |= CALDCO_16MHZ;
    BCSCTL3 = XT2S_0 | LFXT1S_1 | XCAP_1;//Using EXternal  32.768KHZ clock as ACLK
    BCSCTL2 |= SELM_0;//MCLK using internal DCO
}
/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	CLK_INIT();
	DAC_INIT();
	UART_INIT();
	_enable_interrupt();
	while(1){
	    if(getCmd==1&&recvBuff[0]==0x71){//�ж�������ֵ���յ�����������ʱִ�����²���
	        _disable_interrupt();//��������ʵʱ����ʱҪ�ر��ж�
	        dataBITS=(unsigned int)recvBuff[1]|((unsigned int)recvBuff[2])<<8;//����DAC�ļĴ�����ֵ
	        if(DACSETMODE==0xAA){
	            controlBITS=0x1C;//ѡ��DAC-A������
	        }else if(DACSETMODE==0xBB){
	            controlBITS=0x1D;//ѡ��DAC-B������
	        }else{
	            controlBITS=0x0F;//û���õ� ֱ������Ϊupdate all DACs
	        }
	        sendCONTROL(controlBITS);
	        sendDATA(dataBITS);
	        getCmd=0;
	        _enable_interrupt();
	    }
	    else{
	        //���������ԣ�ʵ��ʹ��ʱȥ��
	        controlBITS=0x0F;
	        dataBITS=0xAA;
	        sendCONTROL(controlBITS);
	        sendDATA(dataBITS);
	    }
	}
	return 0;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR_HOOK(void){
    _disable_interrupt();
    if (IFG2 & UCA0TXIFG) {
        UART_OnTX(sendBuff, 20);//UART�����ж�
    }
    _enable_interrupt();
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void){
    _disable_interrupt();
    if(IFG2 & UCA0RXIFG){    //UART�Ľ����ж�
        UART_OnRX(recvBuff,10);
    }
    _enable_interrupt();
}
