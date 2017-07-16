#include <msp430.h> 
#include "driverlib.h"
#include "usci_a_spi.h"

/*
 * main.c
 */
unsigned char gain_count;
int main(void)
{
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN1);//设置P2.1为输入并使能上拉电阻
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1);//设置P1.1为输入并使能上拉电阻
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);//设置P1.0为输出q
    GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN7);//设置P4.7为输出q
    //Initialize slave to MSB first, inactive high clock polarity and 3 wire SPI
    EUSCI_A_SPI_initSlaveParam param = {0};
    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    param.spiMode = EUSCI_A_SPI_4PIN;
    EUSCI_A_SPI_initMaster(EUSCI_A0_BASE, &param);
    //Enable SPI Module
    EUSCI_A_SPI_enable(EUSCI_A0_BASE);
    //Enable Receive interrupt
    EUSCI_A_SPI_enableInterrupt(EUSCI_A0_BASE,EUSCI_A_SPI_TRANSMIT_INTERRUPT);
    gain_count=0;
    while(1)
    {
        if(GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN1) == GPIO_INPUT_PIN_LOW)//如果输入低电平（按键被按下）
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);//输出高电平（灯亮）
            gain_count=gain_count-1;

        }
        else//如果输入高电平（按键没被按下）
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);//输出低电平（灯灭）
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN1) == GPIO_INPUT_PIN_LOW)//如果输入低电平（按键被按下）
        {
             GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7);//输出高电平（灯亮）
        }
        else//如果输入高电平（按键没被按下）
        {
             GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7);//输出低电平（灯灭）
        }
    }
    return 0;
}
