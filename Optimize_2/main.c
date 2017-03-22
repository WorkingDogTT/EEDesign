#include <msp430.h>
#include "MSP430G2553.h"
#include "stdio.h"
#include "math.h"
#define maxPoint 360 //���ﶨ����ǵ��������ֵ
#define sendBuffLength 35 //Ԥ���ķ��ͻ������Ĵ�С
#define TA0CCR0_VAL 4  //��������sin���ε��жϵĳ���ʱ��
#define INTCOUNT_VAL_LowFreq 25 //��������ڵ�Ƶ״̬�¶��ٴ�sin�жϺ󴥷���Ƶ����
#define INTCOUNT_VAL_HighFreq 25

unsigned char state;//ȫ�ֵ�״̬��
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

char recvBuff[10]={0};//��ʼ�����ջ�����
char sendBuff[35]={0};//��ʼ�����ͻ�����
unsigned char sendData=0x00;//������������ż���Ҫ���͸�DAC��sin����
int sweep_Step=10;//�����������趨ɨƵʱ��Ƶ�ʼ����
unsigned int recvBuffIndex=0x00;//�������ý��ջ�������ָʾ��
int sinIntInterval=0;//������ʶsinָ����������
int sinDecInterval=102;//������ʶsinָ���С�����
int sinDecSum=0;     //������ʶsinָ���С��������
int sinIndex=0;      //sin�б��ָ��
int Freq_now=10;      //������ʶ��ǰ��Ƶ��ֵ
int Freq_past=0;      //������ʾ��һ�δ�����Ƶ�жϵ�Ƶ��ֵ
unsigned int ampADResult=0;  //ֱ�ӽ�AD����ת���Ľ��ֵ��������P1.3
unsigned int maxAmpADResult=0;//�Ƚϳ���������AD����ֵ
unsigned int minAmpADResult=1024;//�Ƚϳ�������С��AD����ֵ
unsigned int phaADResult=0;  //ֱ�ӽ�AD����λ����ֵ��������
unsigned int avephaADResult=0;//��λ�Ĳ���ʹ�õ��ǲ���5�κ�ȡƽ��ֵ�İ취
float ampResult=0.0;          //�����ŵ���ת�����ķ������ռ����
float ampResult_old=0.0;      //�����ŵ�����һ��Ƶ��ֵ�����õ��ľ͵÷���ֵ
float phaResult_V=0.0;        //�����ŵ��ǽǶȲ����ĵ��ĵ�ѹֵ
float phaResult_V_old=0.0;    //�����ŵ�����һ��Ƶ��ֵ�����õ�����λ�ĵ�ѹֵ
int x=0,x2=0,y=0,y2=0;        //������������ǻ�ͼʱ��X,Y����ֵ
int INTCount=0;//������ǵ�ǰ���жϴ�������ҪĿ���Ǽ��ٶ�ʱ����ʹ�ã�������һ����ʱ����������й���
unsigned int maxINTCount=3750;
/*ʹ���жϼ����ķ�����ȷ�ϵ�ǰ�Ƿ���Ҫ��Ƶ���߽�����������*/
/***������һЩ�м����***/
unsigned int max_min=0;
unsigned int mul_5=0;
float div_4096=0.0;
float pointInterval=0.0;//��ʱ�������������㵱ǰ��sin����
int setSinINTInterval=0;
int setSinDECInterval=0;
unsigned int mul_9=0.0;
float getDec;
int mul_1000=0;
unsigned char tempData=0x00;

inline void genDAC();
void freqChange();
void freqSet();
void UART_OnTX(char *pbuf,unsigned char length);
void UART_OnRX();
void stateDect();
inline void cmdMatch();



/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    DCOCTL |= CALDCO_16MHZ;
    BCSCTL1 |= CALBC1_16MHZ;
    //BCSCTL3 = LFXT1S_3 | XCAP_3;//�������þ���
    BCSCTL3 |= LFXT1S_0 | XCAP_1;//ʹ��32.768KHZ
    /*******************************************************************
     * init UART
     *******************************************************************/
    //====��������������IO�ڵĹ���========//
    P1SEL = BIT1 | BIT2;
    P1SEL2 = BIT1 | BIT2;
    //=====����UARTʱ��ԴΪ���þ����и��ߵ�׼ȷ��======//
    UCA0BR0 = 0x04 ;        //500k/115200=4.34          UCBRx=  INT(4.34)=4
//    UCA0BR0 = 0x06 ;        //32.768k/115200=2.84         UCBRx=  INT(2.84)=2
    UCA0BR1 = 0x00;         //δ֪ʱ��Դ���趨   ACLK�� UCLK��
    UCA0CTL1 |= UCSSEL_1;   //ѡ������ʱ��Դ��ΪBRCLK
    //ACLK���÷�ʽΪ��UCA0BR1 = UCSSEL_1
    UCA0MCTL =UCBRS1 | UCBRS0;  //UCBRSx=round((4.34-4)x8)=round(2.72)=3
    //UCBRSx=round((2.84-2)x8)=round(6.72)=7
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
    //���þ��������
    P2DIR &= ~BIT6;
    P2SEL |= BIT6 + BIT7;
    P2SEL2 &= ~(BIT6 + BIT7);
    /*****************************************************************
     * init ADC
    *************************************************************/
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
    ADC10CTL0 |= SREF_1 | ADC10SHT_2 | REFON | REF2_5V |ADC10ON ;//�����������
    ADC10CTL1 |= INCH_3  | ADC10DIV_0 | ADC10SSEL_2 | CONSEQ_0 ;//����ʹ�õ�����ʱ��1��Ƶ16MHZ����������ʱ��Ϊ1/16MHZ * 16 =1us ��������3.9us��Ҫ��  ���Ȳɼ������˲���������
    /*****************************************************************
    / * init TA without start TA
     ****************************************************************/
    TA0CTL|=TACLR;
    //����TA0���ڹ̶�ʱ�������ж��������ͷ��������ź� ���Ϊ10us ������ʱ���ԣ
    //���ڴ˴�����TA0CCR0����ͣ��ʱ��
    TA0CTL |= TASSEL_1 | ID_0 | MC_1;
    TA0CTL |= TAIE;
    //�������õ���P1.0�˷������
    //P1.3Ϊ�˲���������
    _enable_interrupts();
    while(1){
        //�����д������ݷŵ��������н��У�Ŀ�����ڼ����ж�ʱ�Ĵ���ǿ�ȣ���������������Ƿ��������λ����
        if(INTCount>=maxINTCount){
            /*���뵽����Ƶ�ʵ�ģʽʱ����һ���¾��ǹر�TA�ж�*/
            TA0CCR0=0;//�ر�TA�жϣ���ֹ����ж̴���
            _disable_interrupts();
            TA0IV=0;
            TA0CTL&=~MC_1;
            INTCount=0;
            //freqChange();        /*
            switch(state){
            case 0xA1://���ڵ�Ƶ���ģʽ
                freqSet();
                break;
            case 0xA2://����ɨƵ���ģʽ
                freqChange();
                break;
            default:
                break;
            }
            //�˳�ʱ�ָ���ʱ�����ж�
            TA0CCR0=TA0CCR0_VAL;
            TA0CTL|=MC_1;
            _enable_interrupts();
        }
    }
    return 0;
}

inline void genDAC(){
    P1OUT|=BIT0;//���Թܽ���λ
    /*ת��ADCP1.3�˿ڲ�����ֵ*/
    ADC10CTL1&=~CONSEQ_3;//�ֶ������ͨ���ظ�ת��ģʽ
    ADC10CTL0|=ADC10ON | ENC | ADC10SC;//����ADC��ת��
    if(sinIndex>=maxPoint) sinIndex-=maxPoint;//�����Ĳ���ֱ�Ӽ�����������������
    sendData=sinLib[sinIndex];//ȡ����ǰ��sinֵ
    /*
     * @ʹ��λ���㽫sinֵ�����DAC
     */
    tempData=(sendData&0x18)<<2;
    P1OUT=tempData;
//    P2OUT=(((sendData&0x80)>>2|(sendData&0x20)<<2)&0xE0)>>2|((sendData&0x40)>>2|(sendData&0x01)<<2);
    tempData=(sendData&0x01)<<2 | (sendData&0x22) | (sendData&0x44)>>2 | (sendData&0x80)>>4;
    P2OUT=tempData;
    // snprintf(sendBuff,5,"%d",(int)sinIndex);
    //sendBuff[0]=sendData;
    //UART_OnTX(sendBuff,5);
    sinIndex+=sinIntInterval;//���������ֵ�ֱ�����
    sinDecSum+=sinDecInterval;//����С�����ֵĺ�ֵ
    if(sinDecSum>=1000){//��С�����ֵĺ�ֵ����1��ʱ�����������ּ�1��λ��ͬʱ�������������
        sinDecSum-=1000;
        sinIndex++;
    }
    /*ȷ��ADC10ת����ɵ�����½�ֵȡ�����Ƚϻ�������Сֵ*/
    while(ADC10CTL1&ADC10BUSY);
    ampADResult=ADC10MEM;
    if(ampADResult>maxAmpADResult) maxAmpADResult=ampADResult;
    if(ampADResult<minAmpADResult) minAmpADResult=ampADResult;
    /*���ˣ���������ʱ��Ϊ1.636us������ֵ��*/
    INTCount++;
    P1OUT&=BIT0;
}

void freqChange(){
    /*��AD��ͨ��Ū��P1.0�����ɼ���ǵķ���*/
    ADC10CTL0&=~(ADC10ON|ENC|ADC10SC);//�ȹر�ADC10���ⲻ��Ҫ������
    ADC10CTL1 &=~INCH_3;//��Ϊ����ֻ��ʹ����INCH_3 ��ֱ�����λ����
    ampResult_old=ampResult;//�����¹�ȥ�ķ���AD����ֵ
    max_min=maxAmpADResult-minAmpADResult;
    mul_5=max_min*5;
    ampResult=mul_5/4096.0;
   // ampResult=((float)(maxAmpADResult-minAmpADResult)/2048.0)*2.5;//�����л����ڼ���и��ӵ���ѧ����
    int i=0;

    for(i=0;i<5;i++){
        ADC10CTL1&=~CONSEQ_3;//�ֶ������ͨ���ظ�ת��ģʽ
        ADC10CTL0 |= (ENC|ADC10ON|ADC10SC);
        while(ADC10CTL1&ADC10BUSY);
        phaADResult+=ADC10MEM;
    }
    phaResult_V_old=phaResult_V;//�����¹�ȥ�ĽǶ�AD����ֵ
    phaResult_V=phaADResult/2048.0;//�Ѿ�����5�ľ�ֵ�� ���ó����õ�ƽ��ֵ
//
    /*��Ƶ�����*/
    Freq_past=Freq_now;//�ڱ�Ƶ֮ǰ�ȱ��浱ǰ��Ƶ��ֵ
    Freq_now+=sweep_Step;//����ɨƵ�Ĳ���
    if(Freq_now>2000){
        //��Ƶ��ֵ����2000ʱ����ǿ����ص���ͷ
        Freq_now=10;
    }

    /*���ֵ���㲿��*/
    //��������ֵ
    mul_9=Freq_now*9;
    pointInterval=mul_9/2500.0;
    //temp=maxPoint/((1.0/(float)Freq_now)/0.00001);//����С�����ֵļ���ֵ

    setSinINTInterval=(int)pointInterval;//ȡ��temp�е���������
    getDec=pointInterval-setSinINTInterval;//ȡ�����е�С������
//    setSinDECInterval=(int)(pointInterval*1000.0-setSinINTInterval*1000);//ȡ��temp�е�С������
    sinIntInterval=setSinINTInterval;
    mul_1000=getDec*1000;//С����������Ϊ����
    sinDecInterval=mul_1000;//��ֵ
    /*��ɼ���ֵ*/
    if(Freq_now>10){
        /*���淢�͵�ǰ�Ļ�����������*/
        x=(int)(Freq_past/10+100);
        y=(int)(ampResult_old*20);
        x2=(int)(Freq_now/10+100);
        y2=(int)(ampResult*20);

        snprintf(sendBuff,sendBuffLength,"line %d,%d,%d,%d,31",x,y,x2,y2);
        UART_OnTX(sendBuff,35);
        snprintf(sendBuff,sendBuffLength,"n0.val=%d",Freq_now);
        UART_OnTX(sendBuff,15);
        /*��������ֹƵ�ʲ��ҽ��䷢�ͳ�ȥ*/
        int a1=0;//������ŷ��������DBֵ��Ѱ��3dB��
        int a2=0;
        a1=-20*log10(ampResult_old);
        a2=-20*log10(ampResult);
        if((a1<3&&a2>=3)){
            //�����ǵ�ͨ�˲��������
            snprintf(sendBuff,sendBuffLength,"n1.val=%d",Freq_now);
            UART_OnTX(sendBuff,15);
        }else if(a1=3&&a2<3){
            //�����Ǹ�ͨ�˲��������
            snprintf(sendBuff,sendBuffLength,"n1.val=%d",Freq_past);
            UART_OnTX(sendBuff,15);
        }
    }
    /*�����ǽ�����ǰ����ʱ���е���մ���*/
    maxAmpADResult=0;
    minAmpADResult=1024;
    //phaADResultÿ���������������ͻ�õ����¸�ֵ���������
    ADC10CTL0&=~(ADC10ON|ENC|ADC10SC);//������ȫ�ر�ADC��������һ���Ĳ���
    ADC10CTL1|=INCH_3;//�����������õ�P1.3�˿�ȥ�ɼ�����ֵ
    ADC10CTL0|=ADC10ON|ENC;//���´�ADC�����ÿ���ʱ������������״̬
    if(Freq_now>=1000) maxINTCount=INTCOUNT_VAL_HighFreq;
    else maxINTCount=INTCOUNT_VAL_LowFreq;
    //�ָ���ʱ����ֵ������ָ�

}

/*
 * ���洦����ǵ�Ƶ״̬���
 * ��Ƶ���Ϊÿ65535*4*0.000002 500ms���Ͻ���һ���жϷ���һ�β�������
 * ����жϺ���ֻ�ܱ�genDAC����
 */
void freqSet(){
    ADC10CTL0&=~(ADC10ON|ADC10SC|ENC);//���ȹر�ADC
    ADC10CTL1&=~INCH_3;//��������ΪP1.0�˿�ȥ�ɼ��˷�������λ��Ϣ
    ADC10CTL0|=(ADC10ON);//����ADC�����ü���ʱ��ȴ���׼������
    int i=0;
    max_min=maxAmpADResult-minAmpADResult;
    mul_5=max_min*5;
    ampResult=mul_5/4096.0;
    for(i=0;i<5;i++){
        ADC10CTL1&=~CONSEQ_3;//�ֶ������ͨ���ظ�ת��ģʽ
        ADC10CTL0 |= (ENC|ADC10ON|ADC10SC);
        while(ADC10CTL1&ADC10BUSY);
        phaADResult+=ADC10MEM;
    }
    phaResult_V_old=phaResult_V;//�����¹�ȥ�ĽǶ�AD����ֵ
    phaResult_V=phaADResult/2048.0;//�Ѿ�����5�ľ�ֵ�� ���ó����õ�ƽ��ֵ

    snprintf(sendBuff,sendBuffLength,"amp.txt=%d",(int)(ampResult*100));
    UART_OnTX(sendBuff,15);
    snprintf(sendBuff,sendBuffLength,"pha.txt=%d",(int)(phaResult_V*100));
    UART_OnTX(sendBuff,15);
    maxINTCount=65535;
    //����ʹ��ADC10
    ADC10CTL0 &=~ (ADC10ON|ENC);
    ADC10CTL1|=INCH_3;
    ADC10CTL0|=ADC10ON|ENC;

}

/*
 * �˺����������ݵ�ǰ�����������ڵı仯
 */
void stateDect(){
    switch(state){
    case 0x00:
        TA0CCR0=0;
        ADC10CTL0&=~(ENC|ADC10SC);
        break;
    case 0xA1:
        if(recvBuff[0]==0x71){
            unsigned int tempFreq;
            tempFreq=recvBuff[1]+recvBuff[2]*256;
            Freq_now=(int)tempFreq;
            /*���ֵ���㲿��*/
            //��������ֵ
            mul_9=Freq_now*9;
            pointInterval=mul_9/2500.0;
            //temp=maxPoint/((1.0/(float)Freq_now)/0.00001);//����С�����ֵļ���ֵ

            setSinINTInterval=(int)pointInterval;//ȡ��temp�е���������
            getDec=pointInterval-setSinINTInterval;//ȡ�����е�С������
        //    setSinDECInterval=(int)(pointInterval*1000.0-setSinINTInterval*1000);//ȡ��temp�е�С������
            sinIntInterval=setSinINTInterval;
            mul_1000=getDec*1000;//С����������Ϊ����
            sinDecInterval=mul_1000;//��ֵ
            maxINTCount=(unsigned int)65535;
        }else{
            snprintf(sendBuff,sendBuffLength,"page 3");
            UART_OnTX(sendBuff,6);
            snprintf(sendBuff,sendBuffLength,"bug.txt=recv %d at 0xA1",(int)recvBuff[1]);
            UART_OnTX(sendBuff,15);
        }

        break;
    case 0xA2:
        if(recvBuff[0]==0x71){
            if(recvBuff[1]==0x01){
                sweep_Step=10;
            }else if(recvBuff[1]==0x02){
                sweep_Step=100;
            }
            Freq_now=10;
            maxINTCount=INTCOUNT_VAL_LowFreq;
        }else{
            snprintf(sendBuff,sendBuffLength,"page 3");
            UART_OnTX(sendBuff,6);
            snprintf(sendBuff,sendBuffLength,"bug.txt=recv %d at 0xA2",(int)recvBuff[1]);
            UART_OnTX(sendBuff,15);
        }
        break;
    default:
        break;
    }
    if(state==0xA1||state==0xA2) {
        TA0R=0;
        TA0CCR0=TA0CCR0_VAL;
        TA0CTL|=MC_1;
    }
}

/*
 * �˺������ڼ�⵱ǰ��ָ��״̬
 */
inline void cmdMatch(){
    switch(recvBuff[0]){
    case 0x00:TA0CCR0=0;state=0x00;break;//�ر�Ψһ�Ķ�ʱ��
    case 0xA0:TA0CCR0=0;state=0x00;break;
    case 0xA1:state=0xA1;break;//��־λ�õ���Ƶ���ģʽ
    case 0xA2:state=0xA2;break;//��־λ�õ�ɨƵ���ģʽ
    case 0xB0:state=0x00;break;
    default:
        stateDect();
        break;
    }
    recvBuffIndex=0;
}


/*
 *�˺������ڽ�������
 */
void UART_OnRX(){
    static int stopBitsCount=0;//������¼��ǰֹͣλ�ĸ���
    recvBuff[recvBuffIndex]=UCA0RXBUF;
    if(recvBuff[recvBuffIndex]==0XFF){
        stopBitsCount++;
    }else{
        stopBitsCount=0;
    }

    if(stopBitsCount>=3||recvBuffIndex>9){
        //���յ�ֹͣλ�������ʻ��߳������ջ������Ĵ�С
        cmdMatch();
        _nop();
    }else{
        recvBuffIndex++;
    }
}

void UART_OnTX(char *pbuf,unsigned char length){
    unsigned char i;
    for(i=0;i<length;i++){
        if(*(pbuf + i)==0x00){
            break;
        }else{
            while(UCA0STAT & UCBUSY);//�ȴ���һ���ֽڷ������
            UCA0TXBUF = *(pbuf + i);
            *(pbuf + i)=0x00;//˳����ջ�����
        }
    }
    for(i=0;i<3;i++){
        /*�����3��ֹͣλ*/
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = 0xFF;
    }
}
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR_HOOK(void){
    /*���ȹر����ж�*/
    _disable_interrupts();
    /*�����еģ���ȡTA0IV������ʹ�жϱ�־λ��λ*/
    switch(__even_in_range(TA0IV,TA0IV_TAIFG)){
    case TA0IV_TACCR1: break;
    case TA0IV_TACCR2: break;
    case TA0IV_TAIFG: genDAC(); break;
    default:break;
    }
    /*�����жϺ������ж�*/
    _enable_interrupts();
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    _disable_interrupts();
    TA0CTL&=~MC_3;
    TA0CCR0=0;
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
