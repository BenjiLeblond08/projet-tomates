
#include "benjamin.h"

void say_hello(void)
{
	FTDI.printf("###############################\r\n");
	FTDI.printf("# Hello World                 #\r\n");
	FTDI.printf("# I'm Teensy 3.2              #\r\n");
	FTDI.printf("# Program by Benjamin LEBLOND #\r\n");
	FTDI.printf("###############################\r\n");
}

void Teensy32_Back2Speed_After_PowerDown(void)
{
	/* SIM->CLKDIV1: OUTDIV1=0,OUTDIV2=1,OUTDIV4=3 Set Prescalers 96MHz cpu, 48MHz bus, 24MHz flash*/
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) |  SIM_CLKDIV1_OUTDIV4(3);
	/* SIM->CLKDIV2: USBDIV=2, Divide 96MHz system clock for USB 48MHz */
	SIM->CLKDIV2 = SIM_CLKDIV2_USBDIV(1);  
	/* OSC0->CR: ERCLKEN=0,EREFSTEN=0,SC2P=1,SC4P=0,SC8P=1,SC16P=0 10pF loading capacitors for 16MHz system oscillator*/
	OSC0->CR = OSC_CR_SC8P_MASK | OSC_CR_SC2P_MASK;
	/* Switch to FBE Mode */
	/* MCG->C7: OSCSEL=0 */
	MCG->C7 = (uint8_t)0x00u;
	/* MCG->C2: LOCKRE0=0,RANGE0=2,HGO=0,EREFS=1,LP=0,IRCS=0 */
	MCG->C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS0_MASK;
	//MCG->C2 = (uint8_t)0x24u;
	/* MCG->C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
	MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3) | MCG_C1_IRCLKEN_MASK;
	/* MCG->C4: DMX32=0,DRST_DRS=0,FCTRIM=0,SCFTRIM=0 */  
	MCG->C4 &= (uint8_t)~(uint8_t)0xE0u; 
	/* MCG->C5: PLLCLKEN=0,PLLSTEN=0,PRDIV0=7 */
	MCG->C5 = MCG_C5_PRDIV0(7);
	/* MCG->C6: LOLIE=0,PLLS=0,CME=0,VDIV0=0 */
	MCG->C6 = (uint8_t)0x00u;
	while((MCG->S & MCG_S_OSCINIT0_MASK) == 0u) { } /* Check that the oscillator is running */
	while((MCG->S & 0x0Cu) != 0x08u); /* Wait until external reference clock is selected as MCG output */
	/* Switch to PBE Mode */
	/* MCG_C5: PLLCLKEN=0,PLLSTEN=0,PRDIV0=5 */
	MCG->C5 = MCG_C5_PRDIV0(3); // config PLL input for 16 MHz Crystal / 4 = 4 MHz
	/* MCG->C6: LOLIE=0,PLLS=1,CME=0,VDIV0=3 */
	MCG->C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0);// config PLL for 96 MHz output
	while((MCG->S & MCG_S_PLLST_MASK) == 0u); /* Wait until the source of the PLLS clock has switched to the PLL */
	while((MCG->S & MCG_S_LOCK0_MASK) == 0u); /* Wait until locked */
	/* Switch to PEE Mode */
	/* MCG->C1: CLKS=0,FRDIV=2,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
	MCG->C1 = MCG_C1_FRDIV(2) | MCG_C1_IRCLKEN_MASK;
	while((MCG->S & 0x0Cu) != 0x0Cu); /* Wait until output of the PLL is selected */
	while((MCG->S & MCG_S_LOCK0_MASK) == 0u); /* Wait until locked */
}

/*
 * Mise a l'heure
 */
void mise_a_l_heure(int wday, int mday, int mon, int year, int hour, int min, int sec)
{
	// Mise à l'heure
	// Gestion de l'heure associée à l'horloge RTC
	struct tm mytime;
	mytime.tm_wday = wday; // int days since Sunday 0-6
	mytime.tm_mday = mday; // int day of the month 1-31
	mytime.tm_mon = mon; // int months since January 0-11
	mytime.tm_year = year - 1900; // int years since 1900 
	mytime.tm_hour = hour; // int hours since midnight 0-23
	mytime.tm_min = min; // int minutes after the hour 0-59
	mytime.tm_sec = sec; // int seconds after the minute 0-59
	// mytime.tm_yday = yday; // int days since January 1 0-365

	// Mise à l'heure de l'horloge RTC
	set_time(mktime(&mytime));
}

