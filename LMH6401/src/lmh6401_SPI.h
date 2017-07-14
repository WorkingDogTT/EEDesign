/*
 * MAX_SPI.h
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */

#ifndef SRC_LMH6401_SPI_H_
#define SRC_LMH6401_SPI_H_

extern unsigned char write_lmh6401(unsigned char *address, unsigned char *data);
extern unsigned char read_lmh6401(unsigned char *address, unsigned char *data);

#endif /* SRC_LMH6401_SPI_H_ */
