/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * @file main_reception_hc12_envoi_oregon.cpp
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * 
 * water_lvl = PTD6
 * water_lvl_sig_freq = 0.69296
 * 
 */

#include "mbed.h"
#include "clk_freqs.h"

#include "tomates.h"
#include "HC12.h"
#include "OSV3.h"
#include "OSV2.h"

/**
 * Definition des Variables
 */
int variable_inutile = 42;
bool hc12_data_received = false;
sensors_union sensors; // Tableau reception HC12

/**
 * Definition des Objets
 */
DigitalOut myled(LED1); // Onboard LED
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
HC12 hc12_1(PTB17, PTB16, PTB19);

/**
 * Pour Oregon Scientifique
 */
TU_DB data_send_OSV;
DigitalOut Data_433(PTC2, 0);
OSV2 osv2_1(Data_433);
OSV3 osv3_1(Data_433);

/**
 * Prototypes des fonctions
 */
void hc12_rx_it(void);
void envoi_oregon(void);

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
				FTDI.printf("\r\ntab[%2d]->[%2d] = %#x %#x %#x %#x", i, i+3, sensors.tab[i], sensors.tab[i+1], sensors.tab[i+2], sensors.tab[i+3]);
			}
			FTDI.printf("\r\n");
			FTDI.printf("\r\ntemp     = (hex) %#x = (10) %f", sensors.data.temp, sensors.data.temp);
			FTDI.printf("\r\nhumidity = (hex) %#x = (10) %d", sensors.data.humidity, sensors.data.humidity);
			FTDI.printf("\r\nUV       = (hex) %#x = (10) %d", sensors.data.UV, sensors.data.UV);
			FTDI.printf("\r\nUVA      = (hex) %#x = (10) %f", sensors.data.UVA, sensors.data.UVA);
			FTDI.printf("\r\nUVB      = (hex) %#x = (10) %f", sensors.data.UVB, sensors.data.UVB);
			FTDI.printf("\r\npressure = (hex) %#x = (10) %d", sensors.data.pressure, sensors.data.pressure);

			envoi_oregon();

			hc12_data_received = false;
		}

		wait(1);

	} // End While True

} // End MAIN

void hc12_rx_it(void)
{
	FTDI.printf("\r\nHC12 Rx It...");
	hc12_1.read(sensors.tab, sizeof(sensors.tab));
	hc12_data_received = true;
	FTDI.printf(" .:: DONE ::.");
}

void envoi_oregon(void)
{
	
}
