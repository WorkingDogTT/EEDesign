
#include "global.h"
/*
 * main.c
 */

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    /***************************************************************
    * init SystemClock
    **************************************************************/
    BCSCTL1 |= CALBC1_16MHZ;
    DCOCTL |= CALDCO_16MHZ;
    BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_3;//ACLK Ϊ12MHZ
    BCSCTL2 &= ~SELS;
    BCSCTL2 = DIVS_3;//SMCLK Ϊ 16MHz/8=2MHz
    state=0;
    SPI_init();
    UART_init();
    __bis_SR_register(GIE);//��ȫ���ж�
    _enable_interrupts();
    while(1){
        switch(state){
        case 0x00:
            _nop();
            break;
        case 0x01:
            gain_value=recvBuff[1];
            test_lmh6401();
            break;
        default:
            _nop();
            break;
        }
        _nop();
    }
	return 0;
}


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
    if(IFG2&UCA0TXIFG){
        _nop();
    }else if(IFG2&UCB0TXIFG){
        //-----�����ж��¼����溯��-----
       SPI_TxISR();
       //-----�жϴ˴β����Ƿ���ɣ�������˳��͹���-----
      if(SPI_Tx_Size==0)
          _bic_SR_register_on_exit(LPM0_bits);
    }
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
    _disable_interrupts();
    //���뵽����״̬��ͣ�µ�ǰ���й���
    if (IFG2 & UCA0RXIFG) {
       IFG2&=~UCA0RXIFG;   // �ֶ������־λ
       UART_OnRX();// ����Tx�¼�������

    }
    else if (IFG2 & UCB0RXIFG) {
        //-----�����ж��¼����溯��-----
        SPI_RxISR();
        //-----�жϴ˴β����Ƿ���ɣ�������˳��͹���-----
        if(SPI_Rx_Size==0){
            _bic_SR_register_on_exit(LPM0_bits);
        }
    }
    _enable_interrupts();


}

