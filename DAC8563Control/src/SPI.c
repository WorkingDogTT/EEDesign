/*
 * SPI.c
 *
 *  Created on: 2017��5��13��
 *      Author: tt
 */
#include <msp430.h>
#include "SPI.h"
void SPI_INIT(){
    //-------��ʼ��8���ŵĸ�������--------------------//
    SPI_SEL |= SPI_CLK | SPI_SOMI | SPI_SIMO ;
    SPI_SEL2 |= SPI_CLK | SPI_SOMI | SPI_SIMO ;
    SPI_DIR |= SPI_CLK | SPI_SIMO;

    //---------�����Ǵ�����ģʽ����Ҫ��������----------//
    //SPI_REN |= SPI_SOMI;
    //SPI_OUT |= SPI_SOMI;

    //----------ʹ��CS�ܽ���Ϊ�������----------------//
    SPI_CS_SEL  &=~SPI_CS;
    SPI_CS_SEL2 &=~SPI_CS;
    SPI_CS_OUT  |= SPI_CS;
    SPI_CS_DIR  |= SPI_CS;

    //----------��λUCB0---------------------------//
    UCB0CTL1 |= UCSWRST;

    //-----------3 - PIN 8 BIT MASTER POSEDGE--------//
    UCB0CTL0 = UCCKPL | UCMSB | UCMSB | UCMST | UCMODE_0 | UCSYNC;

    //----------ʱ��ѡ��  SMCLK MSB first---------------//
    UCB0CTL1 = UCSWRST | UCSSEL_2;

    //----------f_UCxCLK = 16MHZ/2 = 8MHZ  -----------//
    UCB0BR0 = 2;
    UCB0BR1 = 0;
    //UCB0MCTL = 0;

    //----------����UCB0-------------------------------//
    UCB0CTL1 &=~ UCSWRST;
    //----------����жϱ�־λ---------------------------//
    IFG2 &=~ (UCB0RXIFG + UCB0TXIFG);
}

void SPI_CS_HIGH(void){
    SPI_CS_OUT |= SPI_CS;
}

void SPI_CS_LOW(void){
    SPI_CS_OUT &=~ SPI_CS;
}


void SPI_Interrupt_Sel(unsigned char onOff){
    if(onOff==0){
        IE2 &=~ UCB0TXIE;
        IE2 |=  UCB0RXIE;    //�����������ж�
    }else if (onOff==1){
        IE2 &=~ UCB0RXIE;
        IE2 |=  UCB0TXIE;    //�������������ж�
    }else if (onOff==2){
        IE2 |= UCB0RXIE;
        IE2 |= UCB0TXIE;     //����������ж�ͬʱ����
    }else{
        IE2 &=~ UCB0TXIE;
        IE2 &=~ UCB0RXIE;
    }
}


unsigned char SPI_RxFrame(unsigned char *pBuffer, unsigned int size){
    if(size==0)      return 1;
    if(UCB0STAT&UCBUSY)  return 0;//��ǰUSCI������æ������0��ʾ����ʧ��
    _disable_interrupts();
    SPI_Rx_Or_Tx = 0;
    SPI_Rx_Buffer = pBuffer;
    SPI_Rx_Size = size - 1;
    SPI_Interrupt_Sel(SPI_Rx_Or_Tx);
    _enable_interrupts();
    UCB0TXBUF = 0xFF;    //����ģʽ�·��Ϳ��ֽ��Ի��ͨ��ʱ��
    return 1;//���ͳɹ�����1
}


unsigned char SPI_TxFrame(unsigned char *pBuffer, unsigned int size){
    if(size==0)   return 1;
    if(UCB0STAT&UCBUSY)  return 0;//��ǰUSCI�˿���æ������0��ʾ����ʧ��
    _disable_interrupts();
    SPI_Rx_Or_Tx=1;
    SPI_Tx_Buffer = pBuffer;
    SPI_Tx_Size = size - 1;
    SPI_Interrupt_Sel(SPI_Rx_Or_Tx);
    _enable_interrupts();
    UCB0TXBUF = *SPI_Tx_Buffer;  //���͵�һ���ֽڴ��������ж�
    return 1;
}


void SPI_RxISR(){
    *SPI_Rx_Buffer = UCA0RXBUF;
    if(SPI_Rx_Size!=0){
        SPI_Rx_Size--;
        SPI_Rx_Buffer++;
        UCB0TXBUF=0XFF;
    }
    IFG2 &=~ UCB0TXIFG;
}


void SPI_TxISR(){
    UCB0RXBUF;//�ȶ�һ�λ����������־λ
    if(SPI_Tx_Size!=0){
        SPI_Tx_Size--;
        SPI_Tx_Buffer++;
        UCB0TXBUF = *SPI_Tx_Buffer;
    }else{
        IFG2&=~UCB0TXIFG;
    }
}


void sendDACData(unsigned int data,unsigned char control_bits){
    unsigned char data_h;
    unsigned char data_l;
    data_l = (unsigned char)(data&0x00FF);
    data_h = (unsigned char)((data&0xFF00)>>8);
    unsigned char i=0;
    do{
        SPI_CS_LOW();
        i=SPI_TxFrame(&control_bits,8);//�����ܲ���һ���Է���16λ����
        //���Ͳ��ɹ��ͷ�������
       // SPI_CS_HIGH();
    }while(!i);
    do{
        SPI_CS_LOW();
        i=SPI_TxFrame(&data_l,8);
        //SPI_CS_HIGH();
    }while(!i);
    do{
        SPI_CS_LOW();
        i=SPI_TxFrame(&data_h,8);
        SPI_CS_HIGH();
    }while(!i);
}
