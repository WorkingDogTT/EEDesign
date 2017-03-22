#include <msp430.h>
#include "MSP430G2553.h"
#include "stdio.h"
#include "math.h"
#define maxPoint 360 //这里定义的是点数的最大值
#define sendBuffLength 35 //预留的发送缓存区的大小
#define TA0CCR0_VAL 4  //定义生成sin波形的中断的持续时间
#define INTCOUNT_VAL_LowFreq 25 //定义的是在低频状态下多少次sin中断后触发变频函数
#define INTCOUNT_VAL_HighFreq 25

unsigned char state;//全局的状态机
/*in 0x00 system ready
 * 0xA0  Main Page
 * 0xA1  SetFreq
 * 0xA2  sweepFreq
 * 0xB0  Debug Mode (reversed)
 */
/****************************************************
* In this lib we share a 360 points sin table
*/

const unsigned char sinLib[]={128 ,125 ,123 ,121 ,119 ,116 ,114 ,112 ,110 ,108 ,105 ,103 ,101 ,
                              99 ,97 ,95 ,92 ,90 ,88 ,86 ,84 ,82 ,80 ,78 ,76 ,74 ,72 ,70 ,68 ,
                              66 ,64 ,62 ,60 ,58 ,56 ,54 ,53 ,51 ,49 ,47 ,46 ,44 ,42 ,41 ,39 ,
                              37 ,36 ,34 ,33 ,31 ,30 ,28 ,27 ,26 ,24 ,23 ,22 ,21 ,19 ,18 ,17 ,
                              16 ,15 ,14 ,13 ,12 ,11 ,10 ,9 ,8 ,8 ,7 ,6 ,6 ,5 ,4 ,4 ,3 ,3 ,2 ,
                              2 ,2 ,1 ,1 ,1 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,1 ,1 ,1 ,2 ,2 ,
                              2 ,3 ,3 ,4 ,4 ,5 ,6 ,6 ,7 ,8 ,8 ,9 ,10 ,11 ,12 ,13 ,14 ,15 ,16 ,
                              17 ,18 ,19 ,21 ,22 ,23 ,24 ,26 ,27 ,28 ,30 ,31 ,33 ,34 ,36 ,37 ,
                              39 ,41 ,42 ,44 ,46 ,47 ,49 ,51 ,53 ,54 ,56 ,58 ,60 ,62 ,64 ,66 ,
                              68 ,70 ,72 ,74 ,76 ,78 ,80 ,82 ,84 ,86 ,88 ,90 ,92 ,95 ,97 ,99 ,
                              101 ,103 ,105 ,108 ,110 ,112 ,114 ,116 ,119 ,121 ,123 ,125 ,128 ,
                              130 ,132 ,134 ,136 ,139 ,141 ,143 ,145 ,147 ,150 ,152 ,154 ,156 ,
                              158 ,160 ,163 ,165 ,167 ,169 ,171 ,173 ,175 ,177 ,179 ,181 ,183 ,
                              185 ,187 ,189 ,191 ,193 ,195 ,197 ,199 ,201 ,202 ,204 ,206 ,208 ,
                              209 ,211 ,213 ,214 ,216 ,218 ,219 ,221 ,222 ,224 ,225 ,227 ,228 ,
                              229 ,231 ,232 ,233 ,234 ,236 ,237 ,238 ,239 ,240 ,241 ,242 ,243 ,
                              244 ,245 ,246 ,247 ,247 ,248 ,249 ,249 ,250 ,251 ,251 ,252 ,252 ,
                              253 ,253 ,253 ,254 ,254 ,254 ,255 ,255 ,255 ,255 ,255 ,255 ,255 ,
                              255 ,255 ,255 ,255 ,254 ,254 ,254 ,253 ,253 ,253 ,252 ,252 ,251 ,
                              251 ,250 ,249 ,249 ,248 ,247 ,247 ,246 ,245 ,244 ,243 ,242 ,241 ,
                              240 ,239 ,238 ,237 ,236 ,234 ,233 ,232 ,231 ,229 ,228 ,227 ,225 ,
                              224 ,222 ,221 ,219 ,218 ,216 ,214 ,213 ,211 ,209 ,208 ,206 ,204 ,
                              202 ,201 ,199 ,197 ,195 ,193 ,191 ,189 ,187 ,185 ,183 ,181 ,179 ,
                              177 ,175 ,173 ,171 ,169 ,167 ,165 ,163 ,160 ,158 ,156 ,154 ,152 ,
                              150 ,147 ,145 ,143 ,141 ,139 ,136 ,134 ,132 ,130
};

char recvBuff[10]={0};//初始化接收缓存区
char sendBuff[35]={0};//初始化发送缓存区
unsigned char sendData=0x00;//这里是用来存放即将要发送给DAC的sin数据
int sweep_Step=10;//这里是用来设定扫频时的频率间隔的
unsigned int recvBuffIndex=0x00;//用来设置接收缓存器的指示器
int sinIntInterval=0;//用来标识sin指针的整数间隔
int sinDecInterval=102;//用来标识sin指针的小数间隔
int sinDecSum=0;     //用来标识sin指针的小数计数和
int sinIndex=0;      //sin列表的指针
int Freq_now=10;      //用来标识当前的频率值
int Freq_past=0;      //用来表示上一次触发变频中断的频率值
unsigned int ampADResult=0;  //直接将AD幅度转换的结果值放在这里P1.3
unsigned int maxAmpADResult=0;//比较出来的最大的AD采样值
unsigned int minAmpADResult=1024;//比较出来的最小的AD采样值
unsigned int phaADResult=0;  //直接将AD的相位采样值放在这里
unsigned int avephaADResult=0;//相位的采样使用的是采样5次后取平均值的办法
float ampResult=0.0;          //这里存放的是转换过的幅度最终检测结果
float ampResult_old=0.0;      //这里存放的是上一个频率值采样得到的就得幅度值
float phaResult_V=0.0;        //这里存放的是角度采样的到的电压值
float phaResult_V_old=0.0;    //这里存放的是上一个频率值采样得到的相位的电压值
int x=0,x2=0,y=0,y2=0;        //这里是用来标记绘图时的X,Y坐标值
int INTCount=0;//用来标记当前的中断次数，主要目的是减少定时器的使用，尽量在一个定时器中完成所有功能
unsigned int maxINTCount=3750;
/*使用中断计数的方法来确认当前是否需要变频或者进行其他操作*/
/***这里是一些中间变量***/
unsigned int max_min=0;
unsigned int mul_5=0;
float div_4096=0.0;
float pointInterval=0.0;//临时变量，用来计算当前的sin表间隔
int setSinINTInterval=0;
int setSinDECInterval=0;
unsigned int mul_9=0.0;
float getDec;
int mul_1000=0;
unsigned char tempData=0x00;

inline void genDAC();
void freqChange();
void freqSet();
void UART_OnTX(char *pbuf,unsigned char length);
void UART_OnRX();
void stateDect();
inline void cmdMatch();



/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    DCOCTL |= CALDCO_16MHZ;
    BCSCTL1 |= CALBC1_16MHZ;
    //BCSCTL3 = LFXT1S_3 | XCAP_3;//配置外置晶振
    BCSCTL3 |= LFXT1S_0 | XCAP_1;//使用32.768KHZ
    /*******************************************************************
     * init UART
     *******************************************************************/
    //====启动并配置两个IO口的功能========//
    P1SEL = BIT1 | BIT2;
    P1SEL2 = BIT1 | BIT2;
    //=====设置UART时钟源为外置晶振有更高的准确率======//
    UCA0BR0 = 0x04 ;        //500k/115200=4.34          UCBRx=  INT(4.34)=4
//    UCA0BR0 = 0x06 ;        //32.768k/115200=2.84         UCBRx=  INT(2.84)=2
    UCA0BR1 = 0x00;         //未知时钟源的设定   ACLK？ UCLK？
    UCA0CTL1 |= UCSSEL_1;   //选择外置时钟源作为BRCLK
    //ACLK设置方式为：UCA0BR1 = UCSSEL_1
    UCA0MCTL =UCBRS1 | UCBRS0;  //UCBRSx=round((4.34-4)x8)=round(2.72)=3
    //UCBRSx=round((2.84-2)x8)=round(6.72)=7
    UCA0CTL1 &= ~UCSWRST;          //清除软件复位
    IE2 |= UCA0RXIE ;    //开启接收中断

    /******************************************************************
     * init DAC Pin
     ******************************************************************/
    P1DIR |= BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    P1OUT |= BIT4 | BIT7;
    P1OUT &=~(BIT5 | BIT6);
    P2OUT =0x00;
    //test Pin
    P1DIR|=BIT0;
    P1OUT&=~BIT0;
    //配置晶振的引脚
    P2DIR &= ~BIT6;
    P2SEL |= BIT6 + BIT7;
    P2SEL2 &= ~(BIT6 + BIT7);
    /*****************************************************************
     * init ADC
    *************************************************************/
    /*设置说明
     * SREF_1 内部Vref和VSS为基准源
     * ADC10SHT_2 16XADC10CLK
     * REFON  允许内部2.5V的基准输出
     * REF2_5V 基准定为2.5V
     * ADC10ON 启动ADC模块
     * ADC10CTL1
     * INCH_5通道设置为P1.5的被测网络输出
     * ADC10DIV_ 输入时钟1分频
     * ADC10SSEL_2 以主时钟MCLK作为时钟源 则转换速率为500K
     * CONSEQ_2单通道重复转换模式
     */
    ADC10CTL0 |= SREF_1 | ADC10SHT_2 | REFON | REF2_5V |ADC10ON ;//这里的设置是
    ADC10CTL1 |= INCH_3  | ADC10DIV_0 | ADC10SSEL_2 | CONSEQ_0 ;//这里使用的是主时钟1分频16MHZ，采样保持时间为1/16MHZ * 16 =1us 满足最少3.9us的要求  首先采集的是滤波网络的输出
    /*****************************************************************
    / * init TA without start TA
     ****************************************************************/
    TA0CTL|=TACLR;
    //设置TA0用于固定时长发生中断来产生和发送正弦信号 间隔为10us 理论上时间充裕
    //不在此处配置TA0CCR0以暂停定时器
    TA0CTL |= TASSEL_1 | ID_0 | MC_1;
    TA0CTL |= TAIE;
    //这里设置的是P1.0乘法器输出
    //P1.3为滤波网络的输出
    _enable_interrupts();
    while(1){
        //将所有处理内容放到主函数中进行，目的在于减少中断时的处理强度，考虑这种情况下是否会引发复位问题
        if(INTCount>=maxINTCount){
            /*进入到更改频率的模式时，第一件事就是关闭TA中断*/
            TA0CCR0=0;//关闭TA中断，防止多次中短触发
            _disable_interrupts();
            TA0IV=0;
            TA0CTL&=~MC_1;
            INTCount=0;
            //freqChange();        /*
            switch(state){
            case 0xA1://处于点频输出模式
                freqSet();
                break;
            case 0xA2://处于扫频输出模式
                freqChange();
                break;
            default:
                break;
            }
            //退出时恢复定时器的中断
            TA0CCR0=TA0CCR0_VAL;
            TA0CTL|=MC_1;
            _enable_interrupts();
        }
    }
    return 0;
}

inline void genDAC(){
    P1OUT|=BIT0;//测试管脚置位
    /*转换ADCP1.3端口测量幅值*/
    ADC10CTL1&=~CONSEQ_3;//手动清除单通道重复转换模式
    ADC10CTL0|=ADC10ON | ENC | ADC10SC;//启动ADC的转换
    if(sinIndex>=maxPoint) sinIndex-=maxPoint;//超出的部分直接减掉，保留附加相移
    sendData=sinLib[sinIndex];//取出当前的sin值
    /*
     * @使用位运算将sin值输出给DAC
     */
    tempData=(sendData&0x18)<<2;
    P1OUT=tempData;
//    P2OUT=(((sendData&0x80)>>2|(sendData&0x20)<<2)&0xE0)>>2|((sendData&0x40)>>2|(sendData&0x01)<<2);
    tempData=(sendData&0x01)<<2 | (sendData&0x22) | (sendData&0x44)>>2 | (sendData&0x80)>>4;
    P2OUT=tempData;
    // snprintf(sendBuff,5,"%d",(int)sinIndex);
    //sendBuff[0]=sendData;
    //UART_OnTX(sendBuff,5);
    sinIndex+=sinIntInterval;//做整数部分的直接相加
    sinDecSum+=sinDecInterval;//计算小数部分的和值
    if(sinDecSum>=1000){//当小数部分的和值大于1的时候将在整数部分加1进位，同时自身保留下溢出项
        sinDecSum-=1000;
        sinIndex++;
    }
    /*确认ADC10转换完成的情况下将值取出并比较获得最大最小值*/
    while(ADC10CTL1&ADC10BUSY);
    ampADResult=ADC10MEM;
    if(ampADResult>maxAmpADResult) maxAmpADResult=ampADResult;
    if(ampADResult<minAmpADResult) minAmpADResult=ampADResult;
    /*至此，程序运行时间为1.636us（典型值）*/
    INTCount++;
    P1OUT&=BIT0;
}

void freqChange(){
    /*将AD的通道弄回P1.0用来采集相角的幅度*/
    ADC10CTL0&=~(ADC10ON|ENC|ADC10SC);//先关闭ADC10避免不必要的问题
    ADC10CTL1 &=~INCH_3;//因为这里只是使用了INCH_3 故直接清除位即可
    ampResult_old=ampResult;//保留下过去的幅度AD计算值
    max_min=maxAmpADResult-minAmpADResult;
    mul_5=max_min*5;
    ampResult=mul_5/4096.0;
   // ampResult=((float)(maxAmpADResult-minAmpADResult)/2048.0)*2.5;//利用切换的期间进行复杂的数学运算
    int i=0;

    for(i=0;i<5;i++){
        ADC10CTL1&=~CONSEQ_3;//手动清除单通道重复转换模式
        ADC10CTL0 |= (ENC|ADC10ON|ADC10SC);
        while(ADC10CTL1&ADC10BUSY);
        phaADResult+=ADC10MEM;
    }
    phaResult_V_old=phaResult_V;//保留下过去的角度AD测量值
    phaResult_V=phaADResult/2048.0;//已经除过5的均值了 利用除法得到平均值
//
    /*对频率最处理*/
    Freq_past=Freq_now;//在变频之前先保存当前的频率值
    Freq_now+=sweep_Step;//进行扫频的步进
    if(Freq_now>2000){
        //当频率值超过2000时，将强制其回到开头
        Freq_now=10;
    }

    /*间隔值计算部分*/
    //计算间隔的值
    mul_9=Freq_now*9;
    pointInterval=mul_9/2500.0;
    //temp=maxPoint/((1.0/(float)Freq_now)/0.00001);//包含小数部分的计算值

    setSinINTInterval=(int)pointInterval;//取出temp中的整数部分
    getDec=pointInterval-setSinINTInterval;//取出其中的小数部分
//    setSinDECInterval=(int)(pointInterval*1000.0-setSinINTInterval*1000);//取出temp中的小数部分
    sinIntInterval=setSinINTInterval;
    mul_1000=getDec*1000;//小数部分扩大为整数
    sinDecInterval=mul_1000;//赋值
    /*完成计算值*/
    if(Freq_now>10){
        /*下面发送当前的绘制曲线命令*/
        x=(int)(Freq_past/10+100);
        y=(int)(ampResult_old*20);
        x2=(int)(Freq_now/10+100);
        y2=(int)(ampResult*20);

        snprintf(sendBuff,sendBuffLength,"line %d,%d,%d,%d,31",x,y,x2,y2);
        UART_OnTX(sendBuff,35);
        snprintf(sendBuff,sendBuffLength,"n0.val=%d",Freq_now);
        UART_OnTX(sendBuff,15);
        /*下面计算截止频率并且将其发送出去*/
        int a1=0;//用来存放幅度增益的DB值来寻找3dB点
        int a2=0;
        a1=-20*log10(ampResult_old);
        a2=-20*log10(ampResult);
        if((a1<3&&a2>=3)){
            //这里是低通滤波器的情况
            snprintf(sendBuff,sendBuffLength,"n1.val=%d",Freq_now);
            UART_OnTX(sendBuff,15);
        }else if(a1=3&&a2<3){
            //这里是高通滤波器的情况
            snprintf(sendBuff,sendBuffLength,"n1.val=%d",Freq_past);
            UART_OnTX(sendBuff,15);
        }
    }
    /*下面是结束当前计算时进行的清空处理*/
    maxAmpADResult=0;
    minAmpADResult=1024;
    //phaADResult每次在这个函数里面就会得到重新赋值故无需理会
    ADC10CTL0&=~(ADC10ON|ENC|ADC10SC);//首先完全关闭ADC来进行下一步的操作
    ADC10CTL1|=INCH_3;//将其重新配置到P1.3端口去采集幅度值
    ADC10CTL0|=ADC10ON|ENC;//重新打开ADC，利用空闲时间让其进入就绪状态
    if(Freq_now>=1000) maxINTCount=INTCOUNT_VAL_HighFreq;
    else maxINTCount=INTCOUNT_VAL_LowFreq;
    //恢复定时器的值在外面恢复

}

/*
 * 下面处理的是点频状态输出
 * 点频输出为每65535*4*0.000002 500ms以上进入一次中断发送一次采样数据
 * 这个中断函数只能被genDAC调用
 */
void freqSet(){
    ADC10CTL0&=~(ADC10ON|ADC10SC|ENC);//首先关闭ADC
    ADC10CTL1&=~INCH_3;//重新配置为P1.0端口去采集乘法器的相位信息
    ADC10CTL0|=(ADC10ON);//启动ADC，利用计算时间等待期准备就绪
    int i=0;
    max_min=maxAmpADResult-minAmpADResult;
    mul_5=max_min*5;
    ampResult=mul_5/4096.0;
    for(i=0;i<5;i++){
        ADC10CTL1&=~CONSEQ_3;//手动清除单通道重复转换模式
        ADC10CTL0 |= (ENC|ADC10ON|ADC10SC);
        while(ADC10CTL1&ADC10BUSY);
        phaADResult+=ADC10MEM;
    }
    phaResult_V_old=phaResult_V;//保留下过去的角度AD测量值
    phaResult_V=phaADResult/2048.0;//已经除过5的均值了 利用除法得到平均值

    snprintf(sendBuff,sendBuffLength,"amp.txt=%d",(int)(ampResult*100));
    UART_OnTX(sendBuff,15);
    snprintf(sendBuff,sendBuffLength,"pha.txt=%d",(int)(phaResult_V*100));
    UART_OnTX(sendBuff,15);
    maxINTCount=65535;
    //重新使能ADC10
    ADC10CTL0 &=~ (ADC10ON|ENC);
    ADC10CTL1|=INCH_3;
    ADC10CTL0|=ADC10ON|ENC;

}

/*
 * 此函数用来根据当前的输入检测现在的变化
 */
void stateDect(){
    switch(state){
    case 0x00:
        TA0CCR0=0;
        ADC10CTL0&=~(ENC|ADC10SC);
        break;
    case 0xA1:
        if(recvBuff[0]==0x71){
            unsigned int tempFreq;
            tempFreq=recvBuff[1]+recvBuff[2]*256;
            Freq_now=(int)tempFreq;
            /*间隔值计算部分*/
            //计算间隔的值
            mul_9=Freq_now*9;
            pointInterval=mul_9/2500.0;
            //temp=maxPoint/((1.0/(float)Freq_now)/0.00001);//包含小数部分的计算值

            setSinINTInterval=(int)pointInterval;//取出temp中的整数部分
            getDec=pointInterval-setSinINTInterval;//取出其中的小数部分
        //    setSinDECInterval=(int)(pointInterval*1000.0-setSinINTInterval*1000);//取出temp中的小数部分
            sinIntInterval=setSinINTInterval;
            mul_1000=getDec*1000;//小数部分扩大为整数
            sinDecInterval=mul_1000;//赋值
            maxINTCount=(unsigned int)65535;
        }else{
            snprintf(sendBuff,sendBuffLength,"page 3");
            UART_OnTX(sendBuff,6);
            snprintf(sendBuff,sendBuffLength,"bug.txt=recv %d at 0xA1",(int)recvBuff[1]);
            UART_OnTX(sendBuff,15);
        }

        break;
    case 0xA2:
        if(recvBuff[0]==0x71){
            if(recvBuff[1]==0x01){
                sweep_Step=10;
            }else if(recvBuff[1]==0x02){
                sweep_Step=100;
            }
            Freq_now=10;
            maxINTCount=INTCOUNT_VAL_LowFreq;
        }else{
            snprintf(sendBuff,sendBuffLength,"page 3");
            UART_OnTX(sendBuff,6);
            snprintf(sendBuff,sendBuffLength,"bug.txt=recv %d at 0xA2",(int)recvBuff[1]);
            UART_OnTX(sendBuff,15);
        }
        break;
    default:
        break;
    }
    if(state==0xA1||state==0xA2) {
        TA0R=0;
        TA0CCR0=TA0CCR0_VAL;
        TA0CTL|=MC_1;
    }
}

/*
 * 此函数用于检测当前的指令状态
 */
inline void cmdMatch(){
    switch(recvBuff[0]){
    case 0x00:TA0CCR0=0;state=0x00;break;//关闭唯一的定时器
    case 0xA0:TA0CCR0=0;state=0x00;break;
    case 0xA1:state=0xA1;break;//标志位置到点频输出模式
    case 0xA2:state=0xA2;break;//标志位置到扫频输出模式
    case 0xB0:state=0x00;break;
    default:
        stateDect();
        break;
    }
    recvBuffIndex=0;
}


/*
 *此函数用于接收数据
 */
void UART_OnRX(){
    static int stopBitsCount=0;//用来记录当前停止位的个数
    recvBuff[recvBuffIndex]=UCA0RXBUF;
    if(recvBuff[recvBuffIndex]==0XFF){
        stopBitsCount++;
    }else{
        stopBitsCount=0;
    }

    if(stopBitsCount>=3||recvBuffIndex>9){
        //接收到停止位个数合适或者超出接收缓冲区的大小
        cmdMatch();
        _nop();
    }else{
        recvBuffIndex++;
    }
}

void UART_OnTX(char *pbuf,unsigned char length){
    unsigned char i;
    for(i=0;i<length;i++){
        if(*(pbuf + i)==0x00){
            break;
        }else{
            while(UCA0STAT & UCBUSY);//等待上一个字节发送完毕
            UCA0TXBUF = *(pbuf + i);
            *(pbuf + i)=0x00;//顺带清空缓存区
        }
    }
    for(i=0;i<3;i++){
        /*最后发送3个停止位*/
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = 0xFF;
    }
}
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR_HOOK(void){
    /*首先关闭总中断*/
    _disable_interrupts();
    /*必须有的，读取TA0IV向量促使中断标志位复位*/
    switch(__even_in_range(TA0IV,TA0IV_TAIFG)){
    case TA0IV_TACCR1: break;
    case TA0IV_TACCR2: break;
    case TA0IV_TAIFG: genDAC(); break;
    default:break;
    }
    /*结束中断后开启总中断*/
    _enable_interrupts();
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    _disable_interrupts();
    TA0CTL&=~MC_3;
    TA0CCR0=0;
    ADC10CTL0&=~(ENC|ADC10ON|ADC10SC);
    //进入到控制状态先停下当前所有工作
    if (IFG2 & UCA0RXIFG) {
       IFG2&=~UCA0RXIFG;   // 手动清除标志位
       UART_OnRX();// 调用Tx事件处理函数

    }
    else if (IFG2 & UCB0RXIFG) {
    }
    _enable_interrupts();
}
