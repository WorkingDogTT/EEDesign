
#include "global.h"
/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    /***************************************************************
    * init SystemClock
    **************************************************************/
    BCSCTL1 |= CALBC1_16MHZ;
    DCOCTL |= CALDCO_16MHZ;
    BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_3;//ACLK ÎªÄÚ²¿VLO
    SPI_init();
    Dect_init();
    __delay_cycles(16000000);
    _enable_interrupts();
    while(1){
        Dect();
        _nop();
    }

	return 0;
}
