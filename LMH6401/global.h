/*
 * global.h
 *
 *  Created on: 2017年7月3日
 *      Author: tt
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#define DelayMCLK_FREQ          16000000        //用于精确延时函数


#include "msp430g2553.h"
#include "lmh6401_SPI.h"
#include "SPI.h"
#include "src/LMH6401.h"
#include "SystemUtils.h"
#include "Dect.h"

unsigned char address;
unsigned char data;

#endif /* GLOBAL_H_ */
