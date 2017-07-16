/*
 * uart.h
 *
 *  Created on: 2017Äê7ÔÂ15ÈÕ
 *      Author: tt
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

extern void UART_init();
extern void UART_OnTX(char *pbuf,unsigned char length);
extern void UART_OnRX();

#endif /* SRC_UART_H_ */
