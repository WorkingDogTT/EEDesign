/*
 * MAX7456.c
 *
 *  Created on: 2017��7��3��
 *      Author: tt
 */
#include "global.h"
void test_node_B(void){
    address=0x81;
    read_max7456(&address,&data);
}
unsigned char write_node_B(void){
       address=0x00;
       data=0x48;
       write_max7456(&address,&data);
       //д��DMAH[7��0]=0ȷ��λ��
        address=0x05;
        data=0x00;
        write_max7456(&address,&data);
        //д��DMAL[7��0]=xxxȷ��λ��
        address=0x06;
        data=0x21;
        write_max7456(&address,&data);
        //д��DMM[5:3]=xxx���ñ��ر�������
        address=0x04;
        data=0x40;
        write_max7456(&address,&data);
        //д��DMDI[7��0]=0x20
        address=0x07;
        data=0x0C;
        write_max7456(&address,&data);
        //д��DMAH[0]=X
        address=0x05;
        data=0x02;
        write_max7456(&address,&data);
        //д��DMAL[7��0]=xxxȷ��λ��
        address=0x06;
        data=0x21;
        write_max7456(&address,&data);
        //д��DMM[5:3]=xxx���ñ��ر�������
        address=0x04;
        data=0x40;
        write_max7456(&address,&data);
        //д��DMDI[7��0]=0x20
        address=0x07;
        data=0x0C;
        write_max7456(&address,&data);
    return 1;
}

unsigned char write_node_B_clear(void){
           address=0x00;
           data=0x08;
           write_max7456(&address,&data);
           //д��DMAH[7��0]=0ȷ��λ��
            address=0x05;
            data=0x00;
            write_max7456(&address,&data);
            //д��DMAL[7��0]=xxxȷ��λ��
            address=0x06;
            data=0x21;
            write_max7456(&address,&data);
            //д��DMM[5:3]=xxx���ñ��ر�������
            address=0x04;
            data=0x40;
            write_max7456(&address,&data);
            //д��DMDI[7��0]=0x20
            address=0x07;
            data=0x0C;
            write_max7456(&address,&data);
            //д��DMAH[0]=X
            address=0x05;
            data=0x02;
            write_max7456(&address,&data);
            //д��DMAL[7��0]=xxxȷ��λ��
            address=0x06;
            data=0x21;
            write_max7456(&address,&data);
            //д��DMM[5:3]=xxx���ñ��ر�������
            address=0x04;
            data=0x40;
            write_max7456(&address,&data);
            //д��DMDI[7��0]=0x20
            address=0x07;
            data=0x0C;
            write_max7456(&address,&data);
        return 1;
}
