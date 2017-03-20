#include <msp430.h> 
#include "MSP430G2553.h"
#include "stdlib.h"
#include "stdio.h"
//#include "math.h"
#define maxPoint 360
#define sendBuffLength 35
#define TA0CCR0_VAL 4
#define TA1CCR0_VAL_LowFreq 15000
#define TA1CCR0_VAL_HighFreq 10000
unsigned char state;//Global State INDEX
                    /*in 0x00 system ready
                     * 0xA0  Main Page
                     * 0xA1  SetFreq
                     * 0xA2  sweepFreq
                     * 0xB0  Debug Mode (reversed)
                     */
/****************************************************
 * In this lib we share a 360 points sin table
 */
const unsigned char sinLib[]={128 ,125 ,123 ,121 ,119 ,116 ,114 ,112 ,110 ,108 ,105 ,103 ,101 ,
                              99 ,97 ,95 ,92 ,90 ,88 ,86 ,84 ,82 ,80 ,78 ,76 ,74 ,72 ,70 ,68 ,
                              66 ,64 ,62 ,60 ,58 ,56 ,54 ,53 ,51 ,49 ,47 ,46 ,44 ,42 ,41 ,39 ,
                              37 ,36 ,34 ,33 ,31 ,30 ,28 ,27 ,26 ,24 ,23 ,22 ,21 ,19 ,18 ,17 ,
                              16 ,15 ,14 ,13 ,12 ,11 ,10 ,9 ,8 ,8 ,7 ,6 ,6 ,5 ,4 ,4 ,3 ,3 ,2 ,
                              2 ,2 ,1 ,1 ,1 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,1 ,1 ,1 ,2 ,2 ,
                              2 ,3 ,3 ,4 ,4 ,5 ,6 ,6 ,7 ,8 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,
                              17 ,18 ,19 ,21 ,22 ,23 ,24 ,26 ,27 ,28 ,30 ,31 ,33 ,34 ,36 ,37 ,
                              39 ,41 ,42 ,44 ,46 ,47 ,49 ,51 ,53 ,54 ,56 ,58 ,60 ,62 ,64 ,66 ,
                              68 ,70 ,72 ,74 ,76 ,78 ,80 ,82 ,84 ,86 ,88 ,90 ,92 ,95 ,97 ,99 ,
                              101 ,103 ,105 ,108 ,110 ,112 ,114 ,116 ,119 ,121 ,123 ,125 ,128 ,
                              130 ,132 ,134 ,136 ,139 ,141 ,143 ,145 ,147 ,150 ,152 ,154 ,156 ,
                              158 ,160 ,163 ,165 ,167 ,169 ,171 ,173 ,175 ,177 ,179 ,181 ,183 ,
                              185 ,187 ,189 ,191 ,193 ,195 ,197 ,199 ,201 ,202 ,204 ,206 ,208 ,
                              209 ,211 ,213 ,214 ,216 ,218 ,219 ,221 ,222 ,224 ,225 ,227 ,228 ,
                              229 ,231 ,232 ,233 ,234 ,236 ,237 ,238 ,239 ,240 ,241 ,242 ,243 ,
                              244 ,245 ,246 ,247 ,247 ,248 ,249 ,249 ,250 ,251 ,251 ,252 ,252 ,
                              253 ,253 ,253 ,254 ,254 ,254 ,255 ,255 ,255 ,255 ,255 ,255 ,255 ,
                              255 ,255 ,255 ,255 ,254 ,254 ,254 ,253 ,253 ,253 ,252 ,252 ,251 ,
                              251 ,250 ,249 ,249 ,248 ,247 ,247 ,246 ,245 ,244 ,243 ,242 ,241 ,
                              240 ,239 ,238 ,237 ,236 ,234 ,233 ,232 ,231 ,229 ,228 ,227 ,225 ,
                              224 ,222 ,221 ,219 ,218 ,216 ,214 ,213 ,211 ,209 ,208 ,206 ,204 ,
                              202 ,201 ,199 ,197 ,195 ,193 ,191 ,189 ,187 ,185 ,183 ,181 ,179 ,
                              177 ,175 ,173 ,171 ,169 ,167 ,165 ,163 ,160 ,158 ,156 ,154 ,152 ,
                              150 ,147 ,145 ,143 ,141 ,139 ,136 ,134 ,132 ,130
};

char recvBuff[10]={0};//recv char commond give buff in size 10
char sendBuff[35]={0};//leave 35 char space to send message to HMI
unsigned char sendData;//Data will be sent to DAC8032
int scan_Step;//sweepFreq step
unsigned int recvBuffIndex;//Index of recvBuff
int sinIntInterval;//INT part of sin index interval
int sinDecInterval;//dec part of sin index interval
int sinDecSum;//SUM of dec part
int sinIndex;//Index of sinLib
int Freq_now;//restore the real-time freq
unsigned int ampADResult;//10 bit unsigned int ADC Result in P1.5
unsigned int maxAmpADResult;//max 10 bit unsigned int ADC Result
unsigned int minAmpADResult;//min 10 bit unsigned int ADC Result
unsigned int phaADResult;//in P1.0
unsigned int avePhaADResult;//pha ADResult /5
float ampResult;//=(max...-min...)/1024*2.5
float ampResult_old;//restore the last freq value
float phaResult_V;//=(pha+pha...)/5
float phaResult_old_V;//retore the last pha value
int ampResult_dB;
int ampResult_old_dB;
int x,x2,y,y2;//��ͼʱʹ�õ���ʱ����
static int stopbitsCount=0;
/*
 * main.c
 */
//declaration some function
void UART_OnTX(char *pbuf,unsigned char length);
void UART_OnRX();
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    /***************************************************************
    * init SystemClock
    **************************************************************/
    BCSCTL1 |= CALBC1_16MHZ; DCOCTL |= CALDCO_16MHZ;
    BCSCTL3 = XT2S_0 | LFXT1S_3 | XCAP_3;//use ext 500KHZ
    /******************************************************************
     * init Resigter
     ******************************************************************/
    sinDecSum=102;//init in 10Hz
    recvBuffIndex=0;
    sinIntInterval=0;
    sinIndex=0;
    Freq_now=10;
    ampADResult=0;
    maxAmpADResult=0;
    minAmpADResult=1024;//Ҫ������СֵΪ���ֵ��ȷ���䱻������λ
    avePhaADResult=0;
    phaADResult=0;
    ampResult=0.0;
    ampResult_old=0.0;
    phaResult_V=0.0;
    phaResult_old_V=0.0;
    scan_Step=0;
    sinDecInterval=0;
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
    /******************************************************************
     * init DAC Pin
     ******************************************************************/
    P1DIR |= BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    P1OUT |= BIT4 | BIT7;
    P1OUT &=~(BIT5 | BIT6);
    P2OUT =0x00;
    //test Pin
    P1DIR|=BIT0;
    P1OUT&=~BIT0;
    /*****************************************************************
     * init ADC
     ****************************************************************/
        /*����˵��
         * SREF_1 �ڲ�Vref��VSSΪ��׼Դ
         * ADC10SHT_2 16XADC10CLK
         * REFON  �����ڲ�2.5V�Ļ�׼���
         * REF2_5V ��׼��Ϊ2.5V
         * ADC10ON ����ADCģ��
         * ADC10CTL1
         * INCH_5ͨ������ΪP1.5�ı����������
         * ADC10DIV_ ����ʱ��1��Ƶ
         * ADC10SSEL_2 ����ʱ��MCLK��Ϊʱ��Դ ��ת������Ϊ500K
         * CONSEQ_2��ͨ���ظ�ת��ģʽ
         */
//        ADC10CTL0 |= SREF_1 | ADC10SHT_2 | REFON | REF2_5V |ADC10ON ;//�����������
//        ADC10CTL1 |= INCH_5  | ADC10DIV_0 | ADC10SSEL_2 | CONSEQ_0 ;//����ʹ�õ�����ʱ��1��Ƶ16MHZ����������ʱ��Ϊ1/16MHZ * 16 =1us ��������3.9us��Ҫ��  ���Ȳɼ������˲���������
//        /*****************************************************************
         * init TA without start TA
         ****************************************************************/
        TA0CTL|=TACLR;
        TA1CTL|=TACLR;
        //����TA0���ڹ̶�ʱ�������ж��������ͷ��������ź� ���Ϊ10us ������ʱ���ԣ
        //���ڴ˴�ʹ��MCģʽ��������ͣ��ʱ��
        TA0CTL |= TASSEL_1 | ID_0 ;
        TA0CTL |= TAIE;
        TA1CTL |= TASSEL_1 | ID_1 ;//����ɨƵ�ٶ�
        TA1CTL |= TAIE;
        //�������õ���P1.0�˷������
        //P1.5Ϊ�˲���������
        _enable_interrupts();
    while(1){

    }
	return 0;
}

inline void sendDACData(unsigned char data){
        P1OUT=(data&0x18)<<2;
        P2OUT=(((data&0x80)>>2|(data&0x20)<<2)&0xE0)>>2|((data&0x40)>>2|(data&0x01)<<2);
        //_delay_cycles(1);
        P1OUT&=~BIT4;  //WR1�ܽ�����0
        //_delay_cycles(8);
        P1OUT&=~BIT7;
        //_delay_cycles(8);
        P1OUT|=BIT4;
        P1OUT|=BIT7;
}

inline void genDAC(){
    P1OUT|=BIT0;
    if(sinIndex>=maxPoint) sinIndex=sinIndex-maxPoint;
    sendData=sinLib[sinIndex];
    sendDACData(sendData);
    sinIndex+=sinIntInterval;
    sinDecSum+=sinDecInterval;
    if(sinDecSum>=1000){
        sinDecSum=sinDecSum-1000;
        sinIndex++;
    }
    P1OUT&=~BIT0;
}

/********************************************************
 * ��Ƶ���ģʽר��
 *******************************************************/
void freqSet(){
    TA0CTL&=~MC_3;
    TA1CTL&=~MC_3;
    TA0CCR0=0;//������й�0�����ȷ����ʧЧ�ɹ�
    TA1CCR0=0;
//    ADC10CTL0&=~(ENC|ADC10SC|ADC10ON);//ȡ��AD�����ã���AD��ͣ����
//    ADC10CTL1&=~(INCH_5);//ȥ��ת��ͨ�����趨������ǰ��ת�����Ż�ΪP1.0
//    ampResult=(((float)(maxAmpADResult-minAmpADResult))/2048.0)*2.5;//�Ѿ�������2������㵱ǰ�ĵ�ѹֵ,ʹ��ǿ������ת����תΪ�������ĳ˷�
//    int i=0;
//    //����5��ѭ��ȡƽ��ֵ�ļ��ģʽ�������𽺵ļ����Ҫȡƽ��ֵ��Ϊ����
//    for(i=0;i<5;i++){
//        ADC10CTL0|=(ENC|ADC10SC|ADC10ON);
//        while(ADC10CTL1&ADC10BUSY);//�ȴ�DACת�����
//        phaADResult=ADC10MEM;
//        avePhaADResult=(phaADResult+avePhaADResult);//ȡ��
//    }
//    avePhaADResult=avePhaADResult/5;//������ƽ��ֵ
//    phaResult_V=(avePhaADResult/1024.0)*2.5;//�����ѹֵ
//    //
////    snprintf(sendBuff,sendBuffLength,"amp.txt=\"%d\"",(int)(-20*log10(ampResult*100.0)));//����ǰֵת��Ϊ�������Ȳ����
////    UART_OnTX(sendBuff,20);
//    snprintf(sendBuff,sendBuffLength,"pha.txt=\"%d\"",(int)(phaResult_V*100.0));
//    UART_OnTX(sendBuff,15);
//    //
//    maxAmpADResult=0;//�˳��жϺ�������ֵ
//    avePhaADResult=0;
//    minAmpADResult=1024;//����Сֵ����Ϊ���ֵȷ������������Сֵ
//    //��������ADC
//    ADC10CTL1 |= INCH_5 ;//��ͨ�����±�ΪP1.5ȥ����������������
    //==========================================
    //�ں������ý�βʱ���¿��Ŷ�ʱ��
    TA1CCR0=65535;//��Ƶ״̬������Ϊ����������������Ի�ø��ȶ��Ĳ���
    TA0CCR0=TA0CCR0_VAL;
    TA0CTL|=MC_1;
    TA1CTL|=MC_1;//up mode
}


/*********************************************************
 * ɨƵģʽר�õı�Ƶָ��
 */
void freqChange(){
    //Close every TA mode once in this function
    TA0CTL&=~MC_3;//���жϵ�˲��ֹͣ���еĶ�ʱ���Ͳ���
    TA1CTL&=~MC_3;
    TA0CCR0=0;
    TA1CCR0=0;
//    ADC10CTL0&=~(ENC|ADC10SC|ADC10ON);
//    ADC10CTL1&=~(INCH_5);
//    ampResult=(((float)(maxAmpADResult-minAmpADResult))/2048.0)*2.5;//�Ѿ�������2������
//    int i=0;
//    for(i=0;i<5;i++){
//        ADC10CTL0|=(ENC|ADC10SC|ADC10ON);
//        while(ADC10CTL1&ADC10BUSY);
//        avePhaADResult+=ADC10MEM;
//    }
//    avePhaADResult=avePhaADResult/5;
//    phaResult_V=((float)avePhaADResult/1024.0)*2.5;
//    float temp=0.0;//���ݵ�ǰƵ��ֵʵʱ����Ƶ��
//    temp=maxPoint/((1/(float)Freq_now)/0.00001);//���㵱ǰƵ������Ҫ�ļ��
//    int setSinINTInterval=0;
//    setSinINTInterval=(int)temp;//��ǰ�������������
//    int setSinDECInterval=0;
//    setSinDECInterval=(int)(temp*1000.0-setSinINTInterval*1000);//��ǰ�����С������
//    sinIntInterval=setSinINTInterval;//������ĸ�ֵ����
//    sinDecInterval=setSinDECInterval;
//    Freq_now+=scan_Step;
//    if(Freq_now>2000){
//        Freq_now=10;
//    }
//    if(Freq_now<=10){
//        snprintf(sendBuff,sendBuffLength,"page 3");
//        UART_OnTX(sendBuff,10);
//        snprintf(sendBuff,sendBuffLength,"bug.txt=Freq<=10");
//        UART_OnTX(sendBuff,20);
//    }else{
//        //ʹ��log������ֵ���ͳ�ȥ
////         x=(int)(100*log10(Freq_now-scan_Step));
////         x2=(int)(100*log10(Freq_now));
////         ampResult_dB=(int)(3*20*log10(ampResult));
////         ampResult_old_dB=(int)(3*20*log10(ampResult_old));
//         y=ampResult_old_dB+30;
//         y2=ampResult_dB+30;
//         snprintf(sendBuff,sendBuffLength,"line %d,%d,%d,%d,31",x,y,x2,y2);
//         UART_OnTX(sendBuff,11);
//         snprintf(sendBuff,sendBuffLength,"line %d,%d,%d,%d,2016",x,(int)(phaResult_old_V),x2,(int)(phaResult_V));
//         UART_OnTX(sendBuff,11);
//         snprintf(sendBuff,sendBuffLength,"n0.val=%d",Freq_now);
//         UART_OnTX(sendBuff,11);
//         if((y<=3&&y2>3)||(y>=3&&y2<3)){
//             //�ҵ���ֹƵ�ʵ�
//             snprintf(sendBuff,sendBuffLength,"n1.val=%d",Freq_now);
//             UART_OnTX(sendBuff,11);
//         }
//    }
//    ampResult_old=ampResult;//�����ɵ�ֵ
//    phaResult_old_V=phaResult_V;
//    maxAmpADResult=0;//�˳��жϺ�������ֵ
//    avePhaADResult=0;
//    minAmpADResult=1024;
//    //��������ADC
//    ADC10CTL1 |= INCH_5 ;//����ʹ�õ�����ʱ��2��Ƶ8MHZ����������ʱ��Ϊ1/8MHZ * 16 =2us ��������3.9us��Ҫ��  ���Ȳɼ������˲���������
    //==========================================
    //�ں������ý�βʱ���¿��Ŷ�ʱ��
    if(Freq_now>=100) TA1CCR0=TA1CCR0_VAL_HighFreq;
    else TA1CCR0=TA1CCR0_VAL_LowFreq;
    TA0CCR0=TA0CCR0_VAL;
    TA0CTL|=MC_1;
    TA1CTL|=MC_1;
}

inline void UART_OnTX(char *pbuf,unsigned char length){
    unsigned char i;
    for(i=0;i<length;i++){
        if(*(pbuf + i)==0x00) break;
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = *(pbuf + i);
        *(pbuf + i)=0x00;//�ڷ��ͽ���ʱ���

    }
    for(i=0;i<3;i++){//send Stop Bits
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = 0xFF;
    }
}

void stateDect(){
    if(recvBuff[0]==0xFF){
        snprintf(sendBuff,sendBuffLength,"page 3");
        UART_OnTX(sendBuff,10);
        snprintf(sendBuff,sendBuffLength,"bug.txt=\"Begin recv 0xFF\"");
        UART_OnTX(sendBuff,25);
    }
    switch(state){
    case 0x00:
        //stop everything
        sinDecSum=102;//init in 10Hz
        recvBuffIndex=0;
        sinIntInterval=0;
        sinIndex=0;
        Freq_now=10;
        ampADResult=0;
        maxAmpADResult=0;
        minAmpADResult=1024;//Ҫ������СֵΪ���ֵ��ȷ���䱻������λ
        avePhaADResult=0;
        phaADResult=0;
        ampResult=0.0;
        ampResult_old=0.0;
        phaResult_V=0.0;
        phaResult_old_V=0.0;
        scan_Step=0;
        sinDecInterval=0;
    case 0xA1:
        //setFreq mode
        if(recvBuff[0]==0x71){
            unsigned int tempFreq;
            tempFreq=recvBuff[1]+recvBuff[2]*256;
            float temp=0.0;//������������е�ֵ��������ǰӦ��ʹ�õļ��ֵ
            temp=maxPoint/((1/(float)Freq_now)/0.00001);
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
            //ADC10CTL0|=ADC10ON;
        }else{
            snprintf(sendBuff,sendBuffLength,"page 3");
            UART_OnTX(sendBuff,10);
            snprintf(sendBuff,sendBuffLength,"bug.txt=recv %d",recvBuff[0]);
            UART_OnTX(sendBuff,25);
        }
        break;
    case 0xA2:
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
        }else{
            snprintf(sendBuff,sendBuffLength,"page 3");
            UART_OnTX(sendBuff,10);
            snprintf(sendBuff,sendBuffLength,"bug.txt=recv %d",recvBuff[0]);
            UART_OnTX(sendBuff,25);
        }
        break;
    case 0xB0:
        scan_Step=10;
        TA0CCR0=TA0CCR0_VAL;
        TA1CCR0=TA1CCR0_VAL_LowFreq;
        TA0CTL|=MC_1;
        TA1CTL|=MC_1;
       // ADC10CTL0|=ADC10ON;
        break;
    default:
        break;
    }
}

inline void cmdMatch(){
    switch(recvBuff[0]){
    case 0xA0:state=0x00;stateDect();break;
    case 0xA1:state=0xA1;break;
    case 0xA2:state=0xA2;break;
    case 0xB0:state=0xB0;break;
    default:
        stateDect();break;
    }
    recvBuffIndex=0;
}

void UART_OnRX(){
    recvBuff[recvBuffIndex]=UCA0RXBUF;
    if(recvBuff[recvBuffIndex]==0xFF) {
        stopbitsCount++;
    }else{
        stopbitsCount=0;
    }
    if(stopbitsCount>=3||recvBuffIndex>9){
        cmdMatch();
        _nop();
    }else {
        recvBuffIndex++;
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    _disable_interrupts();
    TA0CTL&=~MC_3;
    TA1CTL&=~MC_3;
   // ADC10CTL0&=~(ENC|ADC10ON|ADC10SC);
    //���뵽����״̬��ͣ�µ�ǰ���й���
    if (IFG2 & UCA0RXIFG) {
       IFG2&=~UCA0RXIFG;   // �ֶ������־λ
       UART_OnRX();// ����Tx�¼�������

    }
    else if (IFG2 & UCB0RXIFG) {
    }
    _enable_interrupts();
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR_HOOK(void)
{
    /* USER CODE START (section: TIMER0_A1_ISR_HOOK) */
    _disable_interrupts();
    genDAC();
    //_nop();
    _enable_interrupts();
    /* USER CODE END (section: TIMER0_A1_ISR_HOOK) */
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR_HOOK(void)
{
    _disable_interrupts();
    switch(state){
    case 0xA1:freqSet();break;
    case 0xA2:freqChange();break;
    default:
        break;
    }
    _enable_interrupts();
}

