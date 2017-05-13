#include <msp430.h>

int main(void){
    WDTCTL = WDTPW | WDTHOLD;
    DCOCTL |= CALDCO_16MHZ;
    BCSCTL1 |= CALBC1_16MHZ;
    BCSCTL2 |= DIVS_3;//8��Ƶ
    BCSCTL3 |= LFXT1S_0 | XCAP_1;//ʹ��32.768KHZ

    P1DIR |= BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    P1OUT |= BIT4 | BIT7;
    P1OUT &=~(BIT5 | BIT6);
    P2OUT =0x00;
    //test Pin
    P1DIR|=BIT0|BIT1;
    P1OUT&=~BIT0;
    //���þ��������
    P2DIR &= ~BIT6;
    P2SEL |= BIT6 + BIT7;
    P2SEL2 &= ~(BIT6 + BIT7);

    TA0CTL|=TACLR;
    //����TA0���ڹ̶�ʱ�������ж��������ͷ��������ź� ���Ϊ10us ������ʱ���ԣ
    //���ڴ˴�����TA0CCR0����ͣ��ʱ��
    TA0CTL |= TASSEL_2 | ID_1 | MC_1;
    TA0CTL |= TAIE;
    TA0CCR0=20;//ģ��100KHZ�����
    _enable_interrupts();
    while(1){

    }
}

void genDAC(){
    static unsigned char sendData=0x00;
//    tempData=(sendData&0x18)<<2;
    P1OUT|=BIT0;
//    P1OUT=(sendData&0x18)<<2;
//    P2OUT=(sendData&0x01)<<2 | (sendData&0x22) | (sendData&0x44)>>2 | (sendData&0x80)>>4;

    if(sendData&BIT0) P2OUT |= BIT2; else P2OUT &=~ BIT2;
    if(sendData&BIT1) P2OUT |= BIT1; else P2OUT &=~ BIT1;
    if(sendData&BIT2) P2OUT |= BIT0; else P2OUT &=~ BIT0;
    if(sendData&BIT3) P1OUT |= BIT5; else P1OUT &=~ BIT5;
    if(sendData&BIT4) P1OUT |= BIT6; else P1OUT &=~ BIT6;
    if(sendData&BIT5) P2OUT |= BIT5; else P2OUT &=~ BIT5;
    if(sendData&BIT6) P2OUT |= BIT4; else P2OUT &=~ BIT4;
    if(sendData&BIT7) P2OUT |= BIT3; else P2OUT &=~ BIT3;

//    tempData=(sendData&0x01)<<2 | (sendData&0x22) | (sendData&0x44)>>2 | (sendData&0x80)>>4;
    if(sendData==0xFF){
        sendData=0x00;
    }else{
        sendData+=1;
    }
    P1OUT&=~BIT4;  //WR1�ܽ�����0
    //_delay_cycles(8);
    P1OUT&=~BIT7;
     //_delay_cycles(8);
    P1OUT|=BIT4;
    P1OUT|=BIT7;
    P1OUT&=~BIT0;
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

