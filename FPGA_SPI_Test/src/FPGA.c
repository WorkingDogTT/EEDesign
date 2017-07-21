/*
 * FPGA.c
 *
 *  Created on: 2017年7月21日
 *      Author: tt
 */
#include "global.h"
void getFreq(void){
   address=0x00;
   data=0x10;
   read_FPGA(&address,&data);
   //取外部信号频率计数值
   address=0x10;
   read_FPGA(&address,&data);
   temp_data0=data;
   address=0x11;
   read_FPGA(&address,&data);
   temp_data1=data;
   address=0x12;
   read_FPGA(&address,&data);
   temp_data2=data;
   address=0x13;
   read_FPGA(&address,&data);
   temp_data3=data;
   Signal_Rise_Count=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

   //取参考频率250MHZ_1计数值
   address=0x14;
   read_FPGA(&address,&data);
   temp_data0=data;
   address=0x15;
   read_FPGA(&address,&data);
   temp_data1=data;
   address=0x16;
   read_FPGA(&address,&data);
   temp_data2=data;
   address=0x17;
   read_FPGA(&address,&data);
   temp_data3=data;
   Freq_Ref_250_1=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

   //取参考频率250MHZ_2计数值
   address=0x18;
   read_FPGA(&address,&data);
   temp_data0=data;
   address=0x19;
   read_FPGA(&address,&data);
   temp_data1=data;
   address=0x1A;
   read_FPGA(&address,&data);
   temp_data2=data;
   address=0x1B;
   read_FPGA(&address,&data);
   temp_data3=data;
   Freq_Ref_250_2=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

   //取参考频率250MHZ_3计数值
   address=0x1C;
   read_FPGA(&address,&data);
   temp_data0=data;
   address=0x1D;
   read_FPGA(&address,&data);
   temp_data1=data;
   address=0x1E;
   read_FPGA(&address,&data);
   temp_data2=data;
   address=0x1F;
   read_FPGA(&address,&data);
   temp_data3=data;
   Freq_Ref_250_3=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

   //取参考频率250MHZ_4计数值
   address=0x20;
   read_FPGA(&address,&data);
   temp_data0=data;
   address=0x21;
   read_FPGA(&address,&data);
   temp_data1=data;
   address=0x22;
   read_FPGA(&address,&data);
   temp_data2=data;
   address=0x23;
   read_FPGA(&address,&data);
   temp_data3=data;
   Freq_Ref_250_4=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

   long_temp0 = Freq_Ref_250_1 + Freq_Ref_250_2 + Freq_Ref_250_3 + Freq_Ref_250_4;
   if(long_temp0 != 0){
       long_temp0 = long_temp0 / 4;
       Freq_250_result = (double)Signal_Rise_Count / (double)long_temp0 * 250000000.0;
   }

   //取参考频率10MHZ_1计数值
     address=0x24;
     read_FPGA(&address,&data);
     temp_data0=data;
     address=0x25;
     read_FPGA(&address,&data);
     temp_data1=data;
     address=0x26;
     read_FPGA(&address,&data);
     temp_data2=data;
     address=0x27;
     read_FPGA(&address,&data);
     temp_data3=data;
     Freq_Ref_10_1=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

     //取参考频率10MHZ_2计数值
     address=0x28;
     read_FPGA(&address,&data);
     temp_data0=data;
     address=0x29;
     read_FPGA(&address,&data);
     temp_data1=data;
     address=0x2A;
     read_FPGA(&address,&data);
     temp_data2=data;
     address=0x1B;
     read_FPGA(&address,&data);
     temp_data3=data;
     Freq_Ref_10_2=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

     //取参考频率10MHZ_3计数值
     address=0x2C;
     read_FPGA(&address,&data);
     temp_data0=data;
     address=0x2D;
     read_FPGA(&address,&data);
     temp_data1=data;
     address=0x2E;
     read_FPGA(&address,&data);
     temp_data2=data;
     address=0x2F;
     read_FPGA(&address,&data);
     temp_data3=data;
     Freq_Ref_10_3=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

     //取参考频率10MHZ_4计数值
     address=0x30;
     read_FPGA(&address,&data);
     temp_data0=data;
     address=0x31;
     read_FPGA(&address,&data);
     temp_data1=data;
     address=0x32;
     read_FPGA(&address,&data);
     temp_data2=data;
     address=0x33;
     read_FPGA(&address,&data);
     temp_data3=data;
     Freq_Ref_10_4=temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

     long_temp0 = Freq_Ref_10_1 + Freq_Ref_10_2 + Freq_Ref_10_3 + Freq_Ref_10_4;
     if(long_temp0 != 0){
         long_temp0 = long_temp0 / 4;
         Freq_10_result = (double)Signal_Rise_Count / (double)long_temp0 * 10000000.0;
     }
     _nop();

}

void getDuty(void){
    //取高电平计数值
    address=0x34;
    read_FPGA(&address,&data);
    temp_data0=data;
    address=0x35;
    read_FPGA(&address,&data);
    temp_data1=data;
    address=0x36;
    read_FPGA(&address,&data);
    temp_data2=data;
    address=0x37;
    read_FPGA(&address,&data);
    temp_data3=data;
    Duty_H_counter = temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

    //取低电平计数值
    address=0x38;
    read_FPGA(&address,&data);
    temp_data0=data;
    address=0x39;
    read_FPGA(&address,&data);
    temp_data1=data;
    address=0x3A;
    read_FPGA(&address,&data);
    temp_data2=data;
    address=0x3B;
    read_FPGA(&address,&data);
    temp_data3=data;
    Duty_L_counter = temp_data0+temp_data1*256+temp_data2*65536+temp_data3*16777216;

    long_temp0 = Duty_H_counter + Duty_L_counter;
    if(long_temp0 != 0)
        Duty_Result = (double)Duty_H_counter / (double)long_temp0 *100.0;
}

