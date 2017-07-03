/*
 * SPI.c
 *
 *  Created on: 2017��7��3��
 *      Author: tt
 */


/*
 * SPI.c
 * ����CKPH��CKPL����λ�Ĳ�ͬ��SPI��ʱ����4��Լ��
 * ���ļ�����CKPH=0��CKPL=0��ģʽ��
 * ��1��CLK������Tx���½���Rx
 * ��2��ʱ�ӿ���״̬Ϊ�͵�ƽ
 * ��3��CS�͵�ƽʹ�ܺ�ĵ�һ��CLK���ؿ�ʼ�շ����ݡ�
 *  Created on: 2013-4-3
 *      Author: Administrator
 */
/*
#include"MSP430G2553.h"
#include"SPI.h"
*/
#include "global.h"
#ifdef  SOFT_SPI                //Begin of SOFT_SPI
static unsigned char SPI_Delay=16;          //=1us

//-----�Դӻ�ʹ��CS��STE�����ź궨��-----
#define SPI_CS_HIGH         P2OUT |=BIT4
#define SPI_CS_LOW          P2OUT &=~BIT4
//-----��ͨ��ʱ��CLK���ź궨��-----
#define SPI_CLK_HIGH        P1OUT |=BIT4
#define SPI_CLK_LOW     P1OUT &=~BIT4
//-----�Դ�������SIMO���ź궨��-----
#define SPI_SIMO_HIGH   P1OUT |=BIT2
#define SPI_SIMO_LOW        P1OUT &=~BIT2
//-----�Դӷ�����SOMI���ź궨��-----
#define SPI_SOMI_IN         P1IN &BIT1
/******************************************************************************************************
 * ��       �ƣ�SPI_init(void)
 * ��       �ܣ�����Ӳ��SPI�ĳ�ʼ������
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ����ʵ�ʹ����ǳ�ʼ��SPI���IO�ķ���
 * ��       ������
 ******************************************************************************************************/
void SPI_init(void)
{
    P1DIR |=BIT2+BIT4;
    P2DIR |= BIT4;
    P1DIR &=~BIT1;
}
/******************************************************************************************************
 * ��       �ƣ�SPI_CS_High(void)
 * ��       �ܣ������ϲ㺯���ĵ���
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ������
 * ��       ������
 ******************************************************************************************************/
void SPI_CS_High(void)
{
    SPI_CS_HIGH;
}
/******************************************************************************************************
 * ��       �ƣ�SPI_CS_Low(void)
 * ��       �ܣ������ϲ㺯���ĵ���
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ������
 * ��       ������
 ******************************************************************************************************/
void SPI_CS_Low(void)
{
    SPI_CS_LOW;
}
/******************************************************************************************************
 * ��       �ƣ�delay_us()
 * ��       �ܣ�us����ʱ
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ����ͨ��ȫ�ֱ���SPI_Delay�������
 * ��       ������
 ******************************************************************************************************/
static void delay_us(void)
{
    unsigned int i=0;
    for(i=0;i<SPI_Delay;i++)
        __delay_cycles(DelayMCLK_FREQ/10000000);        //us��ʱ
}
/******************************************************************************************************
 * ��       �ƣ�Tx_Char()
 * ��       �ܣ�������ӻ�����1���ֽ�����
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ������
 * ��       ������
 ******************************************************************************************************/
void Tx_Char(unsigned char data)
{
    unsigned char i=0;
    for(i=0;i<8;i++)
    {
        SPI_CLK_LOW;            delay_us();
        if((data<<i)&BIT7)          SPI_SIMO_HIGH;
        else        SPI_SIMO_LOW;
        delay_us();
        SPI_CLK_HIGH;       delay_us();
    }
}
/******************************************************************************************************
 * ��       �ƣ�Rx_Char()
 * ��       �ܣ��������մӻ�1���ֽ�����
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ������
 * ��       ������
 ******************************************************************************************************/
unsigned char Rx_Char()
{
    unsigned char i=0;
    unsigned char Temp=0;
    for(i=0;i<8;i++)
    {
//      SPI_CLK_HIGH;       delay_us();
//      SPI_CLK_LOW ;       delay_us();

        SPI_CLK_LOW ;       delay_us();
        SPI_CLK_HIGH;       delay_us();
        Temp=Temp<<1;       //��λ����������ǰ��
        if(SPI_SOMI_IN )            //���ո�λ
            Temp |=BIT0;            //��1
//      else Temp &=~BIT0;  //��ʡ�ԣ�Ĭ�Ͼ���0
    }
    return Temp;
}
/******************************************************************************************************
 * ��       �ƣ�SPI_TxFrame()
 * ��       �ܣ�������ӻ�����1��֡����
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ������
 * ��       ������
 ******************************************************************************************************/
unsigned char SPI_TxFrame(unsigned char  *pBuffer, unsigned int   size)
{
    _disable_interrupts();
    unsigned char i=0;
    for(i=0;i<size;i++)                     //Ȼ�����η��͸��ֽ�����
    {
        Tx_Char(*pBuffer);
        pBuffer++;
    }
     _enable_interrupts();
    return 1;
}

/******************************************************************************************************
 * ��       �ƣ�SPI_RxFrame()
 * ��       �ܣ��������մӻ�1֡����
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ������
 * ��       ������
 ******************************************************************************************************/
unsigned char SPI_RxFrame(unsigned char  *pBuffer, unsigned int size)
{
    unsigned char i=0;
    _disable_interrupts();
    for(i=0;i<size;i++)                     //Ȼ�����ν��ո����ֽ�����
    {
         *pBuffer=Rx_Char();
         pBuffer++;
    }
     _enable_interrupts();
    return 1;
}
/****************************************************************************
* ��       �ƣ�SPI_HighSpeed()
* ��       �ܣ�����SPIΪ����
* ��ڲ�������
* ���ڲ�������
* ˵       ������ЩSPI�豸�ɹ����ڸ���SPI״̬
* ʹ�÷�������
****************************************************************************/
void SPI_HighSpeed()
{
    SPI_Delay=1;
}
/****************************************************************************
* ��       �ƣ�SPI_LowSpeed()
* ��       �ܣ�����SPIΪ����
* ��ڲ�������
* ���ڲ�������
* ˵       ������ЩSPI�豸��Ҫ�����ڵ���SPI״̬
* ʹ�÷�������
****************************************************************************/
void SPI_LowSpeed()
{
 SPI_Delay=50;
}
#endif      //End of SOFT_SPI

#ifdef HARD_SPI         //Begin of HRAD_SPI

//-----Ӳ��SPI�ܽź궨��-----
#define SPI_SIMO            BIT2        //1.2
#define SPI_SOMI            BIT1        //1.1
#define SPI_CLK             BIT4        //1.4
#define SPI_CS              BIT4        //P2.4
//-----Ӳ��SPI���ƶ˿ں궨��-----
#define SPI_SEL2            P1SEL2
#define SPI_SEL                 P1SEL
#define SPI_DIR             P1DIR
#define SPI_OUT             P1OUT
#define SPI_REN             P1REN
//-----ʹ�ܶ�CS�˿ں궨��-----
#define SPI_CS_SEL2       P2SEL2
#define SPI_CS_SEL           P2SEL
#define SPI_CS_OUT       P2OUT
#define SPI_CS_DIR           P2DIR

//-----���巢��/���ջ���-----
unsigned char  *SPI_Tx_Buffer;
unsigned char  *SPI_Rx_Buffer;
//-----���������/���յ��ֽ���-----
unsigned char  SPI_Tx_Size=0;
unsigned char  SPI_Rx_Size=0;
//-----���巢��/����ģʽ��־-----
unsigned char SPI_Rx_Or_Tx =0;          // 0:������    1��������   2���շ�
/****************************************************************************
* ��    �ƣ�SPI_init()
* ��    �ܣ���Ӳ��SPI���г�ʼ������
* ��ڲ�������
* ���ڲ�������
* ˵    ��������ʹ�ú���Ķ�д�������ڳ���ʼ�����ȵ��øó�ʼ������
* ʹ�÷�����SPI_init();
****************************************************************************/
void SPI_init(void)
{
    //-----�ܽų�ʼ��Ϊ SPI ����-----
    SPI_SEL |= SPI_CLK + SPI_SOMI + SPI_SIMO;
    SPI_SEL2 |= SPI_CLK + SPI_SOMI + SPI_SIMO;
    SPI_DIR |= SPI_CLK + SPI_SIMO;
    //-----SD ��SPIģʽ�£���Ҫ��SOMI����������-----
    SPI_REN |= SPI_SOMI;
    SPI_OUT |= SPI_SOMI;
    //-----ʹ��CS�ܽ�Ϊ�������-----
    SPI_CS_SEL   &= ~SPI_CS;
    SPI_CS_SEL2 &= ~SPI_CS;
    SPI_CS_OUT |= SPI_CS;
    SPI_CS_DIR  |= SPI_CS;
    //-----��λUCA0-----
    UCA0CTL1 |= UCSWRST;
    //-----3-pin, 8-bit SPI ����ģʽ- ������----
    UCA0CTL0 = UCCKPL + UCMSB + UCMST + UCMODE_0 + UCSYNC;
    //-----ʱ��ѡ��SMCLK��MSB first-----
    UCA0CTL1 = UCSWRST + UCSSEL_2;
    //-----f_UCxCLK = 12MHz/50 = 240kHz-----
    UCA0BR0 = 50;
    UCA0BR1 = 0;
    UCA0MCTL = 0;
    //-----����UCA0-----
    UCA0CTL1 &= ~UCSWRST;
    //-----����жϱ�־λ-----
    IFG2 &= ~(UCA0RXIFG+UCA0TXIFG );
    __bis_SR_register(GIE);
}
/****************************************************************************
* ��       �ƣ�SPI_CS_High()
* ��       �ܣ�3��Ӳ��SPIģʽ������ʹ��CS�ܽ�Ϊ�ߵ�ƽ
* ��ڲ�������
* ���ڲ�������
* ˵       �����˴���CS�ܽſ��Ը���Ӳ������Ҫ������ָ���ܽ���CS���ɡ�
* ʹ�÷�����SPI_CS_High();
****************************************************************************/
void SPI_CS_High(void)
{
    SPI_CS_OUT |= SPI_CS;
}
/****************************************************************************
* ��    �ƣ�SPI_CS_Low()
* ��    �ܣ�3��Ӳ��SPIģʽ������ʹ��CS�ܽ�Ϊ�͵�ƽ
* ��ڲ�������
* ���ڲ�������
* ˵    ���� �˴���CS�ܽſ��Ը���Ӳ������Ҫ������ָ���ܽ���CS���ɡ�
* ʹ�÷�����SPI_CS_SLow();
****************************************************************************/
void SPI_CS_Low(void)
{
    SPI_CS_OUT &= ~SPI_CS;
}
/****************************************************************************
* ��    �ƣ�SPI_Interrupt_Sel()
* ��    �ܣ��������ͻ�����ж�
* ��ڲ�����onOff = 0 :�رշ����жϣ��򿪽����жϣ�
*                   onOff = 1 :�رս����жϣ��򿪷����жϣ�
* ���ڲ�������
* ˵    ���� ʹ�ô˺���������ѡ��ǰ�ն�ģʽ�����ں����������ж�
* ʹ�÷�����SPI_Interrupt_Sel(0);        // �رշ����жϣ��򿪽����жϣ�
*                   SPI_Interrupt_Sel(1);       // �رս����жϣ��򿪷����жϣ�
****************************************************************************/
void SPI_Interrupt_Sel(unsigned char onOff)
{
    if(onOff == 0)                      // ֻ�������ж�
    {
          IE2 &=~UCA0TXIE ;
          IE2 |= UCA0RXIE ;
    }
    else    if(onOff==1)                // ֻ���������ж�
    {
          IE2 &=~UCA0RXIE ;
          IE2 |= UCA0TXIE ;
    }
    else                                        //�շ�ȫ��
    {
         IE2 |= UCA0RXIE ;
         IE2 |= UCA0TXIE ;
    }
}
/****************************************************************************
* ��    �ƣ�SPI_RxFrame()
* ��    �ܣ�3��Ӳ��SPIģʽ�£�����ָ����Ŀ���ֽ�
* ��ڲ�����pBuffer��ָ���Ž������ݵ�����
*                   size��Ҫ���յ��ֽ���
* ���ڲ�����0����ǰӲ��SPI��æ��
*                   1����ǰ�����ѷ�����ϣ�
* ˵    ����ʹ�øú������Խ���ָ��������һ֡����
* ʹ�÷�����SPI_RxFrame(CMD,6);// ����6���ֽڣ������η���CMD��
****************************************************************************/
unsigned char SPI_RxFrame(unsigned char  *pBuffer, unsigned int size)
{
    if(size==0)                                 return (1);
    if(UCA0STAT & UCBUSY)           return  (0);            // �ж�Ӳ��SPI��æ������0
     _disable_interrupts();                                         // �ر����ж�
    SPI_Rx_Or_Tx = 0;                                                   // ��������ģʽ
    SPI_Rx_Buffer = pBuffer;                                        // �����ͻ���ָ������͵������ַ
    SPI_Rx_Size = size-1;                                               // �����͵����ݸ���
    SPI_Interrupt_Sel(SPI_Rx_Or_Tx);                            // SPI�жϿ���ѡ��
    _enable_interrupts();                                               // �����ж�
    UCA0TXBUF = 0xff;                                                   // �ڽ���ģʽ�£�ҲҪ�ȷ���һ�ο��ֽڣ��Ա��ṩͨ��ʱ�ӡ�
    _bis_SR_register(LPM0_bits);                                    // ����͹���ģʽ0
    return (1);
}
/****************************************************************************
* ��    �ƣ�SPI_TxFrame()
* ��    �ܣ�3��Ӳ��SPIģʽ�£�����ָ����Ŀ���ֽڻ���
* ��ڲ�����pBuffer��ָ������͵������ַ
*                   size�������͵��ֽ���
* ���ڲ�����0����ǰӲ��SPI��æ��
*                   1����ǰ�����ѷ�����ϣ�
* ˵    ����ʹ�øú������Է���ָ��������һ֡����
* ʹ�÷�����SPI_TxFrame(CMD,6);  // ��CMD��ȡ��������6���ֽ�
****************************************************************************/
unsigned char SPI_TxFrame(unsigned char  *pBuffer, unsigned int  size)
{
    if(size==0)                                 return (1);
    if(UCA0STAT & UCBUSY)           return  (0);            // �ж�Ӳ��SPI��æ������0
    _disable_interrupts();                                          // �ر����ж�
    SPI_Rx_Or_Tx = 1;                                                   // ��������ģʽ
    SPI_Tx_Buffer = pBuffer;                                        // �����ͻ���ָ������͵������ַ
    SPI_Tx_Size = size-1;                                               // �����͵����ݸ���
    SPI_Interrupt_Sel(SPI_Rx_Or_Tx);                            // SPI�жϿ���ѡ��
    _enable_interrupts();                                               // �����ж�
    UCA0TXBUF = *SPI_Tx_Buffer;                             // �ȷ��͵�һ���ֽ��˹�������һ��"����"�ж�
    _bis_SR_register(LPM0_bits);                                    // ����͹���ģʽ0
    return (1);
}
//-----��ǰ�����¼���������-----
static void SPI_TxISR();
static void SPI_RxISR();
/******************************************************************************************************
 * ��       �ƣ�USCI0TX_ISR_HOOK()
 * ��       �ܣ���ӦTx�жϷ���
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ��������������ѭ��CPU�Ĵ���
 * ��       ������
 ******************************************************************************************************/
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR_HOOK(void)
{
    //-----�����ж��¼����溯��-----
    SPI_TxISR();
    //-----�жϴ˴β����Ƿ���ɣ�������˳��͹���-----
     if(SPI_Tx_Size==0)
    _bic_SR_register_on_exit(LPM0_bits);
}
/******************************************************************************************************
 * ��       �ƣ�USCI0RX_ISR_HOOK()
 * ��       �ܣ���ӦRx�жϷ���
 * ��ڲ�������
 * ���ڲ�������
 * ˵       ��������������ѭ��CPU�Ĵ���
 * ��       ������
 ******************************************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    //-----�����ж��¼����溯��-----
     SPI_RxISR();
    //-----�жϴ˴β����Ƿ���ɣ�������˳��͹���-----
     if(SPI_Rx_Size==0)
    _bic_SR_register_on_exit(LPM0_bits);
}
/******************************************************************************************************
 * ��       �ƣ�SPI_RxISR()
 * ��       �ܣ�SPI��Rx�¼���������
 * ��ڲ�������
 * ���ڲ�������
 * ˵       �����Խ��յ������ݣ�����Դ����д���
 * ��       ������
 ******************************************************************************************************/
static void SPI_RxISR()
{

    *SPI_Rx_Buffer = UCA0RXBUF;                             //  ��ȡ���ջ��棬ͬʱ�����������UCA0RXIFG���жϱ�־λ
    if(SPI_Rx_Size!=0)
    {
        SPI_Rx_Size-- ;                                                     // �����͵����ݼ�1
        SPI_Rx_Buffer++;                                                // ����ָ������һ�ֽ�ƫ��
        UCA0TXBUF = 0xff;                                               // ����Ϊ���ṩCLK��UCA0TXIFG��־λͬʱ�������
    }
    IFG2 &= ~UCA0TXIFG;                                             // ��������жϱ�־λ

}
/******************************************************************************************************
 * ��       �ƣ�SPI_TxISR()
 * ��       �ܣ�SPI��Tx�¼���������
 * ��ڲ�������
 * ���ڲ�������
 * ˵       �����Խ��յ������ݣ�����Դ����д���
 * ��       ������
 ******************************************************************************************************/
static void SPI_TxISR()
{
    UCA0RXBUF;                                                          // Tx��Rx�жϱ�־λ������λ���˴���UCA0RXBUF�ղ��������������UCA0RXIFG���жϱ�־λ
    if(SPI_Tx_Size!=0)
    {
        SPI_Tx_Size-- ;                                                     // �����͵����ݼ�1
        SPI_Tx_Buffer++;                                            // ����ָ������һ�ֽ�ƫ��
        UCA0TXBUF = *SPI_Tx_Buffer;                         // ���뷢�ͻ��棬ͬʱ�����������UCA0TXIFG���жϱ�־λ
    }
    else
        IFG2 &= ~UCA0TXIFG;                                         // ���һ�Σ����ڲ���UCA0TXBUF���в�������Ҫ��Ϊ��������жϱ�־λ
}
/****************************************************************************
* ��       �ƣ�SPI_HighSpeed()
* ��       �ܣ�����SPIΪ����
* ��ڲ�������
* ���ڲ�������
* ˵       ������ЩSPI�豸�ɹ����ڸ���SPI״̬
* ʹ�÷�������
****************************************************************************/
void SPI_HighSpeed()
{
    UCA0CTL1 |= UCSWRST;
    UCA0BR0 = 2;                                // f_UCxCLK = 12MHz/2 = 6MHz
    UCA0BR1 = 0;
    UCA0MCTL = 0;
    UCA0CTL1 &= ~UCSWRST;
}
/****************************************************************************
* ��       �ƣ�SPI_LowSpeed()
* ��       �ܣ�����SPIΪ����
* ��ڲ�������
* ���ڲ�������
* ˵       ������ЩSPI�豸��Ҫ�����ڵ���SPI״̬
* ʹ�÷�������
****************************************************************************/
void SPI_LowSpeed()
{
    UCA0CTL1 |= UCSWRST;
    UCA0BR0 =50;                                // f_UCxCLK = 12MHz/50 = 240KHz
    UCA0BR1 = 0;
    UCA0MCTL = 0;
    UCA0CTL1 &= ~UCSWRST;
}
#endif  //end of HARD_SPI