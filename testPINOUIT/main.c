#include <msp430.h>

int main(void){
    WDTCTL = WDTPW | WDTHOLD;
    DCOCTL |= CALDCO_16MHZ;
    BCSCTL1 |= CALBC1_16MHZ;
    BCSCTL2 |= DIVS_3;//8分频
    BCSCTL3 |= LFXT1S_0 | XCAP_1;//使用32.768KHZ

    P1DIR |= BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    P1OUT |= BIT4 | BIT7;
    P1OUT &=~(BIT5 | BIT6);
    P2OUT =0x00;
    //test Pin
    P1DIR|=BIT0|BIT1;
    P1OUT&=~BIT0;
    //配置晶振的引脚
    P2DIR &= ~BIT6;
    P2SEL |= BIT6 + BIT7;
    P2SEL2 &= ~(BIT6 + BIT7);

    TA0CTL|=TACLR;
    //设置TA0用于固定时长发生中断来产生和发送正弦信号 间隔为10us 理论上时间充裕
    //不在此处配置TA0CCR0以暂停定时器
    TA0CTL |= TASSEL_2 | ID_1 | MC_1;
    TA0CTL |= TAIE;
    TA0CCR0=20;//模拟100KHZ的输出
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
    P1OUT&=~BIT4;  //WR1管脚先置0
    //_delay_cycles(8);
    P1OUT&=~BIT7;
     //_delay_cycles(8);
    P1OUT|=BIT4;
    P1OUT|=BIT7;
    P1OUT&=~BIT0;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR_HOOK(void){
    /*首先关闭总中断*/
    _disable_interrupts();
    /*必须有的，读取TA0IV向量促使中断标志位复位*/
    switch(__even_in_range(TA0IV,TA0IV_TAIFG)){
    case TA0IV_TACCR1: break;
    case TA0IV_TACCR2: break;
    case TA0IV_TAIFG: genDAC(); break;
    default:break;
    }
    /*结束中断后开启总中断*/
    _enable_interrupts();
}

