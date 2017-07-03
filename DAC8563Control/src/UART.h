/*
 * UART.h
 *
 *  Created on: 2017Äê5ÔÂ15ÈÕ
 *      Author: tt
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

unsigned char recvBuffIndex;
unsigned char getCmd;

extern void UART_INIT();
extern void UART_OnTX(unsigned char *pbuf, unsigned char length);
extern void UART_OnRX(unsigned char *pbuf, unsigned char length);

#endif /* SRC_UART_H_ */
