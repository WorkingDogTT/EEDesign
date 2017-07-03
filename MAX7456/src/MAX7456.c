/*
 * MAX7456.c
 *
 *  Created on: 2017年7月3日
 *      Author: tt
 */
#include "global.h"

unsigned char write_node_B(void){
    //在关闭显示之前，首先要满足两个条件 STAT[5]=0  以及 DMM[2]=0

    //首先读取STAT[5]
    do{
    address=0xA0;//STAT address
    read_max7456(&address,&data);
    }while(!((data&0B00100000)==0));

    //读取DMM[2]
    do{
        address=0x84;
        read_max7456(&address,&data);
    }while(!((data&0B00000100)==0));

    //写入VM0[3]=0关闭显示
    address=0x00;
    data=0B00100000;//NTSC   External sync    Disable OSD image    Not reset    Video buffer enable
    write_max7456(&address,&data);

    //写入DMAH[1]=0来写入字符的显示位置
    //前提是要求DMM[2]=0当前没有被清空（这里未使用故直接忽视）
    address=0x05;
    data=0B00000000;
    write_max7456(&address,&data);
    //设定支付显示在第32的地址上，32对应020H故DMAH[0]=0  DMAL[7:0]=0x20
    address=0x06;
    data=0x20;
    write_max7456(&address,&data);
    //设定CA[7:0]确定待显示的预置字符
    address=0x09;//写CMAH  没有找到CA，看看是不是这个？
    data=0x0C;
    write_max7456(&address,&data);

    //写入VM1[3]=1打开显示
    address=0x00;
    data=0B00101000;//NTSC   External sync   Enable OSD image    Not reset    Video buffer enable
    write_max7456(&address,&data);

    return 1;
}
