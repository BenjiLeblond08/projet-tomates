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

#include "mbed.h"
#include "HC12.h"
#include "OSV3.h"

/**
 * Definition des Variables
 */
int variable_inutile = 42;
float water_lvl_sig_freq = 0.69296;
bool pump_on = false;

/**
 * Definition des Objets
 */
Timer t;
InterruptIn pump_it(PTD6);
Timeout pump_timeout;
DigitalOut myled(LED1); // Onboard LED
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
// Serial HC12(PTB17,PTB16); // Tx0, Rx0
HC12 hc12_1(PTB17, PTB16, PTC7);
OSV3 OSV();

/**
 * Prototypes des fonctions
 */
void say_hello(void);
void turn_pump_on(void);
void turn_pump_off(void);

/******************************************************************************
 * MAIN
 *****************************************************************************/
int main(void)
{
	// t.start();
	FTDI.baud(115200);
	FTDI.format(8, SerialBase::None, 1);
	pump_it.mode(PullNone);
	pump_it.rise(&turn_pump_on);
	hc12_1.initialize();

	hc12_1.initialize();

	say_hello();

	while(true)
	{
		myled = myled^1;
		FTDI.printf("\r\n");

		//Reveil dans 10s
		WakeUp::set_ms(10000);
		
		//Apres cette instruction le uC s'arrete
		FTDI.printf("\r\nSLEEP de la Teensy 3.2");
		deepsleep();
		
		Teensy32_Back2Speed_After_PowerDown();
		FTDI.printf("\r\nReveil de la Tennsy 3.2 apres 10s");


		wait(1);
	}

	return 0;
}

/**
 * Fonction say_hello
 */
void say_hello(void)
{
	FTDI.printf("###############################\r\n");
	FTDI.printf("# Hello World                 #\r\n");
	FTDI.printf("# I'm Teensy 3.2              #\r\n");
	FTDI.printf("# Program by Benjamin LEBLOND #\r\n");
	FTDI.printf("###############################\r\n");
}

/**
 * Fonction pour timeout et interruption pour activer/desactiver la pompe
 */
void turn_pump_on(void)
{
	if (!pump_on)
	{
		FTDI.printf("Demarrage de la pompe\r\n");
		// Envoyer le signal pour DEMARER la pompe
		pump_on = true;
	}
	else
		FTDI.printf("Pompe deja demarree\r\n");

	pump_timeout.attach(&turn_pump_off, 2.0);
}

void turn_pump_off(void)
{
	FTDI.printf("Arret de la pompe\r\n");
	// Envoyer le signal pour ARRETER la pompe
	pump_on = false;
	pump_timeout.detach();
}
