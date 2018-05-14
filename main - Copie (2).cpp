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
float water_lvl_sig_freq = 0.69296;
bool pump_on = false;

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

// data_send_OSV.Data_Capteur.HeureAcqui
// data_send_OSV.Data_Capteur.Temperature
// data_send_OSV.Data_Capteur.HR
// data_send_OSV.Data_Capteur.UV
// data_send_OSV.Data_Capteur.Pression
// data_send_OSV.Data_Capteur.RollingCode
// data_send_OSV.Data_Capteur.type_capteur



/**
 * Definition des Objets
 */
InterruptIn pump_it(PTD6);
Timeout pump_timeout;
DigitalOut myled(LED1); // Onboard LED
DigitalOut hc12_power(PTD7, 1);
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
HC12 hc12(PTB17, PTB16, PTB19);

/**
 * Prototypes des fonctions
 */
void turn_pump_on(void);
void turn_pump_off(void);
void hc12_it_rx(void);

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
	hc12.initialize();
	mise_a_l_heure(MERCREDI, 9, MAI, 2018, 0, 0, 0);

	say_hello();
	
	hc12.attach(&hc12_it_rx, Serial::RxIrq);

	while(true)
	{
		myled = myled^1;
		FTDI.printf("\r\n");
		WakeUp::calibrate();
		WakeUp::set_ms(10000); // Reveil dans 10s
		
		// CODE HERE




		// END CODE HERE

		wait_ms(500);
		
		//Apres cette instruction le uC s'arrete
		// FTDI.printf("\r\nSLEEP de la Teensy 3.2");
		// deepsleep();
		
		// Teensy32_Back2Speed_After_PowerDown();
		wait(1);
		// FTDI.printf("\r\nReveil de la Tennsy 3.2 apres 10s");
	}
}


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

void hc12_it_rx(void)
{
	FTDI.printf("Reception HC12\r\n");	
	// hc12.read(sensors.tab, sizeof(sensors));
	FTDI.printf("FIN Reception HC12\r\n");
}

