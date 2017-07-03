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
        getCmd=1;//标志当前收到的一条完整指令
        recvBuffIndex=0;//清空接收缓存
        if(*(pbuf)==0xAA){
            DACSETMODE=0xAA;//表明当前设置的是DAC-A
        }
        if(*(pbuf)==0xBB){
            DACSETMODE=0xBB;//表明当前设置的是DAC-B
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
    SPI_OUT |= DAC_CLR | DAC_SCLK;//时钟信号在任何时候持续高电平  清楚位在初始化是保持为高

    /*
     * 初始化DAC芯片
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
	    if(getCmd==1&&recvBuff[0]==0x71){//判断是数字值且收到了完整命令时执行如下部分
	        _disable_interrupt();//进入命令实时处理时要关闭中断
	        dataBITS=(unsigned int)recvBuff[1]|((unsigned int)recvBuff[2])<<8;//计算DAC的寄存器的值
	        if(DACSETMODE==0xAA){
	            controlBITS=0x1C;//选中DAC-A来配置
	        }else if(DACSETMODE==0xBB){
	            controlBITS=0x1D;//选中DAC-B来配置
	        }else{
	            controlBITS=0x0F;//没有用的 直接配置为update all DACs
	        }
	        sendCONTROL(controlBITS);
	        sendDATA(dataBITS);
	        getCmd=0;
	        _enable_interrupt();
	    }
	    else{
	        //仅用作测试，实际使用时去除
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
        UART_OnTX(sendBuff, 20);//UART发送中断
    }
    _enable_interrupt();
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void){
    _disable_interrupt();
    if(IFG2 & UCA0RXIFG){    //UART的接收中断
        UART_OnRX(recvBuff,10);
    }
    _enable_interrupt();
}
