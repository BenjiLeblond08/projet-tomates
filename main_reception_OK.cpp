/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * @file main_reception_ok.cpp
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

/**
 * Variables des données récupérées des capteurs
 */
float VAL_TempC, VAL_UV, VAL_P;
unsigned int VAL_HR;

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
// Serial hc12_serial(PTB17, PTB16);
HC12 hc12_1(PTB17, PTB16, PTB19);

/**
 * Prototypes des fonctions
 */
void hc12_it_rx(void);

/******************************************************************************
 * MAIN
 *****************************************************************************/
int main(void)
{
	FTDI.baud(9600);
	FTDI.format(8, SerialBase::None, 1);
	wait_ms(1);
	say_hello();
	
	mise_a_l_heure(MERCREDI, 9, MAI, 2018, 0, 0, 0);
	
	hc12_1.initialize();
	hc12_1.clearBuffer();

	hc12_1.attach(&hc12_it_rx, Serial::RxIrq);

	FTDI.printf("\r\n # DEBUT BOUCLE #\r\n");
	while(true)
	{
		myled = myled^1;
		FTDI.printf("\r\n");
		
		// CODE HERE

		FTDI.printf("\r\ntemp     = (hex) 0x%#x = (10) %f", sensors.data.temp, sensors.data.temp);
		FTDI.printf("\r\nhumidity = (hex) 0x%#x = (10) %d", sensors.data.humidity, sensors.data.humidity);
		FTDI.printf("\r\nUV       = (hex) 0x%#x = (10) %d", sensors.data.UV, sensors.data.UV);
		FTDI.printf("\r\nUVA      = (hex) 0x%#x = (10) %f", sensors.data.UVA, sensors.data.UVA);
		FTDI.printf("\r\nUVB      = (hex) 0x%#x = (10) %f", sensors.data.UVB, sensors.data.UVB);
		FTDI.printf("\r\npressure = (hex) 0x%#x = (10) %d", sensors.data.pressure, sensors.data.pressure);


		// END CODE HERE

		wait_ms(500);
	}
}

void hc12_it_rx(void)
{
	FTDI.printf("\r\nReception HC12");
	hc12_1.read(sensors.tab, sizeof(sensors.tab));
	// FTDI.printf("\r\nFIN Reception HC12");
	FTDI.printf("\r\nData readed:");
	for (int i = 0; i < sizeof(sensors.tab); ++i)
	{
		FTDI.printf("\r\n %#x", sensors.tab[i]);
	}
}

