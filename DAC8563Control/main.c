#include <msp430.h> 



/*
 * main.c
 */
//����ȫ�ֱ���
int MODE=0;//ģʽ����   Ĭ��Ϊ0ģʽ  �ֶ�����ģʽ
char recvBuff[10]={0};//������ջ���  ����Ļ��õĽ�����Ϣ
char sendBuff[20]={0};//���巢�ͻ���  ����Ļ���͵ķ�����Ϣ


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	DCOCTL |= CALDOC_16MHZ;
	BCSCTL1 |= CALBC1_16MHZ;//Setup internal DCO clock to 16MHZ
	BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_1;//Using internal VLO 12MHz clock as ACLK
	BCSCTL2 |= SELM_0;//MCLK using internal DCO
	
	/******************************************************************
	 * ����SPIͨ�ſ�
	 ******************************************************************/
	UCB0CTL1 |= UCSWRST;//���Ƚ��и�λ����
	UCB0CTL0 &=~ (UCCKPH | UCCKPL | UC7BIT | UCMODE_3);
	UCBOCTL0 |= UCMSB | UCMST | UCSYNC //����SPI  ʱ����λѡ��Ϊ�ڵ�һ��UCLK�����ϲ����ź�
	//  UCCKPL=0 ����ʱ�Ӽ���Ϊ�ߵ�ƽ״̬��Ч     UC7BIT=0 ����ÿһ֡�ĳ���Ϊ8λ  һ����Ҫ3֡��������
	//  UCMSB=1 �����������ȷ���λ�󷢵�λ��   UCMST˵����ǰ����������ģʽ��    UCSYNC��λ������ͬ��ģʽ��
    //���ݸ�Ҫ:  * * * * * * * * + * * * * * * * * Ϊ2��16λ������λ  ����λ�ȷ���8λ�󷢵�8λ
	//          * * * * * * * * Ϊ8λ�Ŀ���λ�����е�3λ�ǵ�ַλ ��3λ�ǿ���λ   ��2λ��Ч����ֵ
	UCB0CTL1 = UCSSEL_3;//ʹ��SMCLK ������������λʹ��λ
	UCB0BR0 = 2; // 16MHZ/2=8MHZ
	UCB0BR1 = 0;


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
    IE2 |= UCA0RXIE ;    //ʹ�ý����ж�




	return 0;
}
