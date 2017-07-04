#include <msp430.h> 
#include "max7456.h"
#include "spi.h"
/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
        /***************************************************************
        * init SystemClock
        **************************************************************/
        BCSCTL1 |= CALBC1_16MHZ;
        DCOCTL |= CALDCO_16MHZ;
        BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_3;//ACLK ÎªÄÚ²¿VLO
	return 0;
}
