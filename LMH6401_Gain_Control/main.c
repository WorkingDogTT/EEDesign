#include <msp430.h> 
#include "driverlib.h"

/*
 * main.c
 */
int main(void)
{
	WDT_A_hold(WDT_A_BASE);	// Stop watchdog timer
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN1);//����P2.1Ϊ���벢ʹ����������
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);//����P1.0Ϊ���
	while(1)
	{
		if(GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN1) == GPIO_INPUT_PIN_LOW)//�������͵�ƽ�����������£�
		{
			GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);//����ߵ�ƽ��������
		}
		else//�������ߵ�ƽ������û�����£�
		{
			GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);//����͵�ƽ������
		}
	}
	return 0;
}
