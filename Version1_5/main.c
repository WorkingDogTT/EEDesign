#include <msp430.h> 
#include "MSP430G2553.h"
#include "stdio.h"
#include "math.h"
#define PointMax 360
#define sendBuffLength 35
#define TA0CCR0_VAL 4
#define TA1CCR0_VAL_LowFreq 15000
#define TA1CCR0_VAL_HighFreq 10000
unsigned char state;//����������ǰ��״̬
const unsigned char sinLib[]={128 ,125 ,123 ,121 ,119 ,116 ,114 ,112 ,110 ,108 ,105 ,103 ,101 ,99 ,97 ,95 ,92 ,90 ,88 ,86 ,84 ,82 ,80 ,78 ,76 ,74 ,72 ,70 ,68 ,66 ,64 ,62 ,60 ,58 ,56 ,54 ,53 ,51 ,49 ,47 ,46 ,44 ,42 ,41 ,39 ,37 ,36 ,34 ,33 ,31 ,30 ,28 ,27 ,26 ,24 ,23 ,22 ,21 ,19 ,18 ,17 ,16 ,15 ,14 ,13 ,12 ,11 ,10 ,9 ,8 ,8 ,7 ,6 ,6 ,5 ,4 ,4 ,3 ,3 ,2 ,2 ,2 ,1 ,1 ,1 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,1 ,1 ,1 ,2 ,2 ,2 ,3 ,3 ,4 ,4 ,5 ,6 ,6 ,7 ,8 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,17 ,18 ,19 ,21 ,22 ,23 ,24 ,26 ,27 ,28 ,30 ,31 ,33 ,34 ,36 ,37 ,39 ,41 ,42 ,44 ,46 ,47 ,49 ,51 ,53 ,54 ,56 ,58 ,60 ,62 ,64 ,66 ,68 ,70 ,72 ,74 ,76 ,78 ,80 ,82 ,84 ,86 ,88 ,90 ,92 ,95 ,97 ,99 ,101 ,103 ,105 ,108 ,110 ,112 ,114 ,116 ,119 ,121 ,123 ,125 ,128 ,130 ,132 ,134 ,136 ,139 ,141 ,143 ,145 ,147 ,150 ,152 ,154 ,156 ,158 ,160 ,163 ,165 ,167 ,169 ,171 ,173 ,175 ,177 ,179 ,181 ,183 ,185 ,187 ,189 ,191 ,193 ,195 ,197 ,199 ,201 ,202 ,204 ,206 ,208 ,209 ,211 ,213 ,214 ,216 ,218 ,219 ,221 ,222 ,224 ,225 ,227 ,228 ,229 ,231 ,232 ,233 ,234 ,236 ,237 ,238 ,239 ,240 ,241 ,242 ,243 ,244 ,245 ,246 ,247 ,247 ,248 ,249 ,249 ,250 ,251 ,251 ,252 ,252 ,253 ,253 ,253 ,254 ,254 ,254 ,255 ,255 ,255 ,255 ,255 ,255 ,255 ,255 ,255 ,255 ,255 ,254 ,254 ,254 ,253 ,253 ,253 ,252 ,252 ,251 ,251 ,250 ,249 ,249 ,248 ,247 ,247 ,246 ,245 ,244 ,243 ,242 ,241 ,240 ,239 ,238 ,237 ,236 ,234 ,233 ,232 ,231 ,229 ,228 ,227 ,225 ,224 ,222 ,221 ,219 ,218 ,216 ,214 ,213 ,211 ,209 ,208 ,206 ,204 ,202 ,201 ,199 ,197 ,195 ,193 ,191 ,189 ,187 ,185 ,183 ,181 ,179 ,177 ,175 ,173 ,171 ,169 ,167 ,165 ,163 ,160 ,158 ,156 ,154 ,152 ,150 ,147 ,145 ,143 ,141 ,139 ,136 ,134 ,132 ,130
};
char recvBuff[10]={0};
char sendBuff[35]={0};
unsigned char sendData;
int scan_Step;
unsigned int RecvBuffIndex;
int sinIntInterval;
int sinDecSum;//����С�����ֵĺ�
int sinDecInterval;
int sinIndex;//����ָʾ��ǰ�����͵�sin���ݵ������±�
int Freq_now;//���浱ǰ��Ƶ��ֵ
unsigned int stepInterval;//��ʶ��ǰ��Ƶ�ʲ������
unsigned int ampADResult;
unsigned int maxAmpADResult;
unsigned int minAmpADResult;
unsigned int avePhaADResult;
unsigned int phaADResult;
float ampResult;//Ӧ����ԭ�ȵĽ���Ļ����ϳ���10000
float ampResult_old;//Ϊ����ֵ��һ������
float phaResult;//Ӧ����ԭ�ȵĽ���Ļ����ϳ���10000
float phaResult_old;
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
    sinDecSum=102;//Ĭ�ϵ�С��������10Hz�����
    RecvBuffIndex=0;
    sinIntInterval=0;
    sinIndex=0;
    Freq_now=10;
    stepInterval=0;
    ampADResult=0;
    maxAmpADResult=0;
    minAmpADResult=1024;//Ҫ������СֵΪ���ֵ��ȷ���䱻������λ
    avePhaADResult=0;
    phaADResult=0;
    ampResult=0.0;
    ampResult_old=0.0;
    phaResult=0.0;
    phaResult_old=0.0;
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
         * ADC10DIV_1 ����ʱ��2��Ƶ
         * ADC10SSEL_2 ����ʱ��MCLK��Ϊʱ��Դ ��ת������Ϊ500K
         * CONSEQ_2��ͨ���ظ�ת��ģʽ
         */
        ADC10CTL0 |= SREF_1 | ADC10SHT_2 | REFON | REF2_5V |ADC10ON ;//�����������
        ADC10CTL1 |= INCH_5  | ADC10DIV_0 | ADC10SSEL_2 | CONSEQ_0 ;//����ʹ�õ�����ʱ��2��Ƶ8MHZ����������ʱ��Ϊ1/8MHZ * 16 =2us ��������3.9us��Ҫ��  ���Ȳɼ������˲���������

        //�������õ���P1.0�˷������
        //P1.5Ϊ�˲���������
    /*****************************************************************
     * init TA
     ****************************************************************/
    TA0CTL|=TACLR;
    TA1CTL|=TACLR;
    //����TA0���ڹ̶�ʱ�������ж��������ͷ��������ź� ���Ϊ10us ������ʱ���ԣ
    //���ڴ˴�ʹ��MCģʽ��������ͣ��ʱ��
    TA0CTL |= TASSEL_1 | ID_0 ;
    TA0CTL |= TAIE;
    TA1CTL |= TASSEL_1 | ID_1 ;//����ɨƵ�ٶ�
    TA1CTL |= TAIE;
    _enable_interrupts();
    while(1){
//        if(ampADResult>maxAmpADResult) maxAmpADResult=ampADResult;
//        if(ampADResult<minAmpADResult) minAmpADResult=ampADResult;
    }
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
    if(sinIndex>=PointMax) sinIndex=sinIndex-PointMax;
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
    ADC10CTL0&=~(ENC|ADC10SC|ADC10ON|ADC10CT);//ȡ��DTC������
    ADC10CTL1&=~(INCH_5);
    ampResult=((maxAmpADResult-minAmpADResult)/2048.0)*2.5;//�Ѿ�������2����
    int i=0;
    for(i=0;i<5;i++){
        ADC10CTL0|=(ENC|ADC10SC|ADC10ON);
        while(ADC10CTL1&ADC10BUSY);
        phaADResult=ADC10MEM;
        avePhaADResult=(unsigned int)(phaADResult+avePhaADResult)/2;
    }
    ADC10CTL1|=INCH_5;
    phaResult=(avePhaADResult/1024.0)*2.5;
    snprintf(sendBuff,sendBuffLength,"amp.txt=\"%d\"",(int)(-20*log10(ampResult*100.0)));
    UART_OnTX(sendBuff,15);
    snprintf(sendBuff,sendBuffLength,"pha.txt=\"%d\"",(int)( phaResult*100.0));
    UART_OnTX(sendBuff,15);
    maxAmpADResult=0;//�˳��жϺ�������ֵ
    avePhaADResult=0;
    minAmpADResult=1024;
    //��������ADC
    ADC10CTL1 |= INCH_5 ;//����ʹ�õ�����ʱ��2��Ƶ8MHZ����������ʱ��Ϊ1/8MHZ * 16 =2us ��������3.9us��Ҫ��  ���Ȳɼ������˲���������
    //==========================================
    //�ں������ý�βʱ���¿��Ŷ�ʱ��
    TA1CCR0=65535;//��Ƶ״̬������Ϊ����������������Ի�ø��ȶ��Ĳ���
    TA0CCR0=TA0CCR0_VAL;
    TA0CTL|=MC_1;
    TA1CTL|=MC_1;
}
/*********************************************************
 * ɨƵģʽר�õı�Ƶָ��
 */
void freqChange(){
    TA0CTL&=~MC_3;//���жϵ�˲��ֹͣ���еĶ�ʱ���Ͳ���
    TA1CTL&=~MC_3;
    TA0CCR0=0;
    TA1CCR0=0;
    ADC10CTL0&=~(ENC|ADC10SC|ADC10ON);
    ADC10CTL1&=~(INCH_5);
    ampResult=((maxAmpADResult-minAmpADResult)/2048.0)*2.5;//�Ѿ�������2������
    int i=0;
    for(i=0;i<5;i++){
        ADC10CTL0|=(ENC|ADC10SC|ADC10ON);
        while(ADC10CTL1&ADC10BUSY);
        avePhaADResult=(avePhaADResult+ADC10MEM)/2;
    }
    ADC10CTL1|=INCH_5;
    phaResult=(avePhaADResult/1024.0)*2.5;
    float temp=0.0;
    temp=PointMax/((1/(float)Freq_now)/0.00001);
    int setSinINTInterval=0;
    setSinINTInterval=(int)temp;
    int setSinDECInterval=0;
    setSinDECInterval=(int)(temp*1000.0-setSinINTInterval*1000);
    sinIntInterval=setSinINTInterval;
    sinDecInterval=setSinDECInterval;
    Freq_now+=scan_Step;
    if(Freq_now>2000){
        Freq_now=10;
    }
    if((Freq_now)<=10){
    }else{
         snprintf(sendBuff,sendBuffLength,"line %d,%d,%d,%d,31",(int)(100*log10(Freq_now-scan_Step)),(int)(3*20*log10(ampResult_old)+30),(int)(100*log10(Freq_now)),(int)(3*20*log10(ampResult)+30));
         UART_OnTX(sendBuff,11);
         snprintf(sendBuff,sendBuffLength,"line %d,%d,%d,%d,2016",(int)(100*log10(Freq_now-scan_Step)),(int)(phaResult_old),(int)(100*log10(Freq_now)),(int)(phaResult));
         UART_OnTX(sendBuff,11);
         snprintf(sendBuff,sendBuffLength,"n0.val=%d",Freq_now);
         UART_OnTX(sendBuff,11);
         if(20*log10(ampResult)<=3&&20*log10(ampResult_old)>3){
             snprintf(sendBuff,sendBuffLength,"n1.val=%d",Freq_now);
             UART_OnTX(sendBuff,11);
         }
    }
    ampResult_old=ampResult;
    phaResult_old=phaResult;
    maxAmpADResult=0;//�˳��жϺ�������ֵ
    avePhaADResult=0;
    minAmpADResult=1024;
    //��������ADC
    ADC10CTL1 |= INCH_5 ;//����ʹ�õ�����ʱ��2��Ƶ8MHZ����������ʱ��Ϊ1/8MHZ * 16 =2us ��������3.9us��Ҫ��  ���Ȳɼ������˲���������
    //==========================================
    //�ں������ý�βʱ���¿��Ŷ�ʱ��
    if(Freq_now>=100) TA1CCR0=TA1CCR0_VAL_HighFreq;
    else TA1CCR0=TA1CCR0_VAL_LowFreq;
    TA0CCR0=TA0CCR0_VAL;
    TA0CTL|=MC_1;
    TA1CTL|=MC_1;
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
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR_HOOK(void)
{
    _nop();
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
    /* USER CODE START (section: TIMER0_A1_ISR_HOOK) */
    _disable_interrupts();
    switch(__even_in_range(TA1IV,TA1IV_TAIFG)){
        case TA1IV_TACCR1 :  break;
        case TA1IV_TACCR2 : break;
        case TA1IV_TAIFG :
             switch(state){
             case 0xA2:freqChange();break;
             case 0xA1:freqSet();break;
             default:break;
             }
             break;
        default : break;
        }
    //_nop();
    _enable_interrupts();
    /* USER CODE END (section: TIMER0_A1_ISR_HOOK) */
}



