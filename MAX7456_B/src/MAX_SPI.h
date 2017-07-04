/*
 * MAX_SPI.h
 *
 *  Created on: 2017Äê7ÔÂ3ÈÕ
 *      Author: tt
 */

#ifndef SRC_MAX_SPI_H_
#define SRC_MAX_SPI_H_

extern unsigned char write_max7456(unsigned char *address, unsigned char *data);
extern unsigned char read_max7456(unsigned char *address, unsigned char *data);

#endif /* SRC_MAX_SPI_H_ */
