/*
 * global.h
 *
 *  Created on: 2017年7月3日
 *      Author: tt
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#define DelayMCLK_FREQ          16000000        //用于精确延时函数

#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "msp430g2553.h"
#include "lmh6401_SPI.h"
#include "SPI.h"
#include "src/LMH6401.h"
#include "SystemUtils.h"
#include "uart.h"
#include "USART_HMI.h"
#include "FPGA.h"
#include "FPGA_SPI.h"
#include "GPIO.h"

unsigned char address;
unsigned char data;
unsigned char gain_value;
unsigned char SPI_sendBuff[2];


unsigned char RecvBuffIndex;
unsigned char recvBuff[1];
unsigned char SendBuffIndex;
unsigned char sendBuffLength;
char sendBuff[30];
unsigned char stopbitsCount;
unsigned char temp_data_1;

unsigned long Signal_Rise_Count;
unsigned long Freq_Ref_250_1;
unsigned long Freq_Ref_250_2;
unsigned long Freq_Ref_250_3;
unsigned long Freq_Ref_250_4;
unsigned long Freq_Ref_10_1;
unsigned long Freq_Ref_10_2;
unsigned long Freq_Ref_10_3;
unsigned long Freq_Ref_10_4;
unsigned long Duty_H_counter;
unsigned long Duty_L_counter;
unsigned char temp_data0;
unsigned char temp_data1;
unsigned char temp_data2;
unsigned char temp_data3;


unsigned long long_temp0;
unsigned long long_temp1;
unsigned long long_temp2;

unsigned long Zhenshu;
unsigned long Xiaoshu;

unsigned int High_bits;
unsigned int Low_bits;

int bytes[9];
int xiaoshu_bytes[6];


double Freq_250_result;
double Freq_10_result;
double Duty_Result;


unsigned char state;


#endif /* GLOBAL_H_ */
