/*
 * FPGA_SPI.h
 *
 *  Created on: 2017Äê7ÔÂ21ÈÕ
 *      Author: tt
 */

#ifndef SRC_FPGA_SPI_H_
#define SRC_FPGA_SPI_H_

extern unsigned char read_FPGA(unsigned char *address, unsigned char *data);
extern unsigned char write_FPGA(unsigned char *address, unsigned char *data);

#endif /* SRC_FPGA_SPI_H_ */
