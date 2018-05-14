/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * 
 * water_lvl = PTD6
 * water_lvl_sig_freq = 0.69296
 * 
 */

#include "tomates.h"

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
sensors_union sensors;

/**
 * Definition des Objets
 */
DigitalOut myled(LED1); // Onboard LED
DigitalOut hc12_power(PTD7, 0);
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
// Serial HC12(PTB17,PTB16); // Tx0, Rx0
HC12 hc12_1(PTB17, PTB16, PTC7);


/******************************************************************************
 * MAIN
 *****************************************************************************/
int main(void)
{
	FTDI.baud(115200);
	FTDI.format(8, SerialBase::None, 1);
	hc12_1.initialize();

	say_hello();

	while(true)
	{
		myled = myled^1;
		FTDI.printf("\r\n");
		WakeUp::calibrate();
		WakeUp::set_ms(10000); // Reveil dans 10s
		
		// CODE HERE

		// Données des capteurs pour envoi
		VAL_TempC = -27.2;
		VAL_HR = 88;
		VAL_UV = 5;
		VAL_P = 1031;
		sensors.data.temp = 27.2;
		sensors.data.humidity = 88;
		sensors.data.UV = 5;
		sensors.data.UVA = 3;
		sensors.data.UVB = 2;
		sensors.data.pressure = 1031;

		hc12_1.write(sensors.tab);

		// END CODE HERE

		wait_ms(500);
		
		//Apres cette instruction le uC s'arrete
		FTDI.printf("\r\nSLEEP de la Teensy 3.2");
		deepsleep();
		
		Teensy32_Back2Speed_After_PowerDown();
		FTDI.printf("\r\nReveil de la Tennsy 3.2 apres 10s");
	}
}

