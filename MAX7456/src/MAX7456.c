/*
 * MAX7456.c
 *
 *  Created on: 2017��7��3��
 *      Author: tt
 */
#include "global.h"

unsigned char write_node_B(void){
    //�ڹر���ʾ֮ǰ������Ҫ������������ STAT[5]=0  �Լ� DMM[2]=0

    //���ȶ�ȡSTAT[5]
    do{
    address=0xA0;//STAT address
    read_max7456(&address,&data);
    }while(!((data&0B00100000)==0));

    //��ȡDMM[2]
    do{
        address=0x84;
        read_max7456(&address,&data);
    }while(!((data&0B00000100)==0));

    //д��VM0[3]=0�ر���ʾ
    address=0x00;
    data=0B00100000;//NTSC   External sync    Disable OSD image    Not reset    Video buffer enable
    write_max7456(&address,&data);

    //д��DMAH[1]=0��д���ַ�����ʾλ��
    //ǰ����Ҫ��DMM[2]=0��ǰû�б���գ�����δʹ�ù�ֱ�Ӻ��ӣ�
    address=0x05;
    data=0B00000000;
    write_max7456(&address,&data);
    //�趨֧����ʾ�ڵ�32�ĵ�ַ�ϣ�32��Ӧ020H��DMAH[0]=0  DMAL[7:0]=0x20
    address=0x06;
    data=0x20;
    write_max7456(&address,&data);
    //�趨CA[7:0]ȷ������ʾ��Ԥ���ַ�
    address=0x09;//дCMAH  û���ҵ�CA�������ǲ��������
    data=0x0C;
    write_max7456(&address,&data);

    //д��VM1[3]=1����ʾ
    address=0x00;
    data=0B00101000;//NTSC   External sync   Enable OSD image    Not reset    Video buffer enable
    write_max7456(&address,&data);

    return 1;
}
