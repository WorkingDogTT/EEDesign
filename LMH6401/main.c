
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
    BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_3;//ACLK 为12MHZ
    BCSCTL2 &= ~SELS;
    BCSCTL2 = DIVS_3;//SMCLK 为 16MHz/8=2MHz
    state=0;
    SPI_init();
    UART_init();
    __bis_SR_register(GIE);//打开全局中断
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
 * 名       称：USCI0TX_ISR_HOOK()
 * 功       能：响应Tx中断服务
 * 入口参数：无
 * 出口参数：无
 * 说       明：包含唤醒主循环CPU的代码
 * 范       例：无
 ******************************************************************************************************/
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR_HOOK(void)
{
    if(IFG2&UCA0TXIFG){
        _nop();
    }else if(IFG2&UCB0TXIFG){
        //-----发送中断事件引擎函数-----
       SPI_TxISR();
       //-----判断此次操作是否完成，完成则退出低功耗-----
      if(SPI_Tx_Size==0)
          _bic_SR_register_on_exit(LPM0_bits);
    }
}
/******************************************************************************************************
 * 名       称：USCI0RX_ISR_HOOK()
 * 功       能：响应Rx中断服务
 * 入口参数：无
 * 出口参数：无
 * 说       明：包含唤醒主循环CPU的代码
 * 范       例：无
 ******************************************************************************************************/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    _disable_interrupts();
    //进入到控制状态先停下当前所有工作
    if (IFG2 & UCA0RXIFG) {
       IFG2&=~UCA0RXIFG;   // 手动清除标志位
       UART_OnRX();// 调用Tx事件处理函数

    }
    else if (IFG2 & UCB0RXIFG) {
        //-----接收中断事件引擎函数-----
        SPI_RxISR();
        //-----判断此次操作是否完成，完成则退出低功耗-----
        if(SPI_Rx_Size==0){
            _bic_SR_register_on_exit(LPM0_bits);
        }
    }
    _enable_interrupts();


}

