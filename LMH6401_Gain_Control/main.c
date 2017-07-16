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
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN1);//����P2.1Ϊ���벢ʹ����������
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1);//����P1.1Ϊ���벢ʹ����������
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);//����P1.0Ϊ���q
    GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN7);//����P4.7Ϊ���q
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
        if(GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN1) == GPIO_INPUT_PIN_LOW)//�������͵�ƽ�����������£�
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);//����ߵ�ƽ��������
            gain_count=gain_count-1;

        }
        else//�������ߵ�ƽ������û�����£�
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);//����͵�ƽ������
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN1) == GPIO_INPUT_PIN_LOW)//�������͵�ƽ�����������£�
        {
             GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7);//����ߵ�ƽ��������
        }
        else//�������ߵ�ƽ������û�����£�
        {
             GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7);//����͵�ƽ������
        }
    }
    return 0;
}
