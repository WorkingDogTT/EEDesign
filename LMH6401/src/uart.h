/*
 * uart.h
 *
 *  Created on: 2017��7��15��
 *      Author: tt
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

extern void UART_init();
extern void UART_OnTX(char *pbuf,unsigned char length);
extern void UART_OnRX();

#endif /* SRC_UART_H_ */
