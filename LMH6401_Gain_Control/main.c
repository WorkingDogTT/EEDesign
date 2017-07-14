#include <msp430.h> 
#include "driverlib.h"

/*
 * main.c
 */
int main(void)
{
	WDT_A_hold(WDT_A_BASE);	// Stop watchdog timer
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN1);//设置P2.1为输入并使能上拉电阻
	GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);//设置P1.0为输出
	while(1)
	{
		if(GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN1) == GPIO_INPUT_PIN_LOW)//如果输入低电平（按键被按下）
		{
			GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);//输出高电平（灯亮）
		}
		else//如果输入高电平（按键没被按下）
		{
			GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);//输出低电平（灯灭）
		}
	}
	return 0;
}
