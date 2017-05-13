#include <msp430.h> 



/*
 * main.c
 */
//定义全局变量
int MODE=0;//模式定义   默认为0模式  手动控制模式
char recvBuff[10]={0};//定义接收缓冲  从屏幕获得的接收信息
char sendBuff[20]={0};//定义发送缓冲  向屏幕传送的发送信息


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	DCOCTL |= CALDOC_16MHZ;
	BCSCTL1 |= CALBC1_16MHZ;//Setup internal DCO clock to 16MHZ
	BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_1;//Using internal VLO 12MHz clock as ACLK
	BCSCTL2 |= SELM_0;//MCLK using internal DCO
	
	/******************************************************************
	 * 配置SPI通信块
	 ******************************************************************/
	UCB0CTL1 |= UCSWRST;//首先进行复位操作
	UCB0CTL0 &=~ (UCCKPH | UCCKPL | UC7BIT | UCMODE_3);
	UCBOCTL0 |= UCMSB | UCMST | UCSYNC //设置SPI  时钟相位选择为在第一个UCLK边沿上捕获信号
	//  UCCKPL=0 设置时钟极性为高电平状态有效     UC7BIT=0 设置每一帧的长度为8位  一共需要3帧处理数据
	//  UCMSB=1 表明数据是先发高位后发低位的   UCMST说明当前工作在主控模式下    UCSYNC置位工作在同步模式下
    //数据概要:  * * * * * * * * + * * * * * * * * 为2个16位的数据位  数据位先发高8位后发低8位
	//          * * * * * * * * 为8位的控制位，其中低3位是地址位 中3位是控制位   高2位无效任意值
	UCB0CTL1 = UCSSEL_3;//使用SMCLK 并且清除软件复位使能位
	UCB0BR0 = 2; // 16MHZ/2=8MHZ
	UCB0BR1 = 0;


    /*******************************************************************
     * init UART
     *******************************************************************/
    //====启动并配置两个IO口的功能========//
    P1SEL = BIT1 | BIT2;
    P1SEL2 = BIT1 | BIT2;
    //=====设置UART时钟源为外置晶振有更高的准确率======//
    UCA0BR0 = 0x04 ;        //500k/115200=4.34          UCBRx=  INT(4.34)=4
    UCA0BR1 = 0x00;         //未知时钟源的设定   ACLK？ UCLK？
    UCA0CTL1 |= UCSSEL_1;   //选择外置时钟源作为BRCLK
    //ACLK设置方式为：UCA0BR1 = UCSSEL_1
    UCA0MCTL = UCBRS1 + UCBRS0;    //UCBRSx=round((4.34-4)x8)=round(2.72)=3
    UCA0CTL1 &= ~UCSWRST;          //清除软件复位
    IE2 |= UCA0RXIE ;    //使用接收中断




	return 0;
}
