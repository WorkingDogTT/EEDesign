
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
    BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_3;//ACLK 为2M
    BCSCTL2 &= ~SELS;

    //初始化各变量的值
    state=0x00;
    RecvBuffIndex=0x00;
    stopbitsCount=0;
    SPI_init();
    UART_init();
    init_GPIO();
    _enable_interrupts();
    while(1){
        switch(state){
        case 0x00:
            _nop();
            break;
        case 0x01:
            getFreq();
            state=0x00;
            break;
        case 0x02:
            getDuty();
            state=0x00;
            break;
        case 0x03:
            state=0x01;
            break;
        case 0x04:
            state=0x02;
            break;
        default:
            _nop();
            break;
        }
        _nop();
        getDuty();
        getFreq();
        display();
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
        IFG2&=~UCA0TXIFG;
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


#pragma vector = PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    _disable_interrupt();
    if(P2IFG & BIT0)//判断是否是P1.3产生中断
    {
        P2IFG &= ~BIT0;//清除标志位
        if(state == 0x02)
             state=0x03;
        else
            state = 0x01;
    }else if(P2IFG & BIT1){
        P2IFG &=~ BIT1;
        if(state == 0x01)
            state = 0x04;
        else
            state = 0x02;
    }
    _enable_interrupt();
}
