/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * @file main_reception_ok.cpp
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * 
 */

#include "mbed.h"
#include "clk_freqs.h"

#include "tomates.h"
#include "HC12.h"

/**
 * Definition des Variables
 */
int variable_inutile = 42;
bool hc12_data_received = false;

/**
 * Tableau pour envoie HC12
 */
TU_DB data_send_OSV;
sensors_union sensors;

/**
 * Definition des Objets
 */
DigitalOut myled(LED1); // Onboard LED
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
HC12 hc12_1(PTB17, PTB16, PTB19);

/**
 * Prototypes des fonctions
 */
void hc12_rx_it(void);

/******************************************************************************
 * MAIN
 *****************************************************************************/
int main(void)
{
	FTDI.baud(9600);
	FTDI.format(8, SerialBase::None, 1);
	say_hello();
	
	mise_a_l_heure(MERCREDI, 9, MAI, 2018, 0, 0, 0);
	
	hc12_1.initialize();
	hc12_1.clearBuffer();

	hc12_1.attach(&hc12_rx_it, Serial::RxIrq);

	while(true)
	{
		myled = myled^1;
		FTDI.printf("\r\n");
		
		// CODE HERE


		// END CODE HERE

		if (hc12_data_received)
		{
			FTDI.printf("\r\nReceived data:");
			for (int i = 0; i < sizeof(sensors.tab); i=i+4) {
				FTDI.printf("\r\ntab[%-2d]->[%-2d] (hex) = %-2x %-2x %-2x %-2x", i, i+3, sensors.tab[i], sensors.tab[i+1], sensors.tab[i+2], sensors.tab[i+3]);
			}
			print_sensors_data(sensors);
			hc12_data_received = false;
		}

		wait(1);

	} // End While True

} // End MAIN

void hc12_rx_it(void)
{
	int i = hc12_1.read(sensors.tab, sizeof(sensors.tab));
	hc12_data_received = true;
}

