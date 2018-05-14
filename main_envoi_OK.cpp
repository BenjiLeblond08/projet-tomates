/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * @file main_envoi_ok.cpp
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * 
 * water_lvl = PTD6
 * water_lvl_sig_freq = 0.69296
 * 
 */

#include "mbed.h"
#include "WakeUp.h"
#include "clk_freqs.h"

#include "tomates.h"
#include "HC12.h"

/**
 * Definition des Variables
 */
int variable_inutile = 42;
float water_lvl_sig_freq = 0.69296;
int water_lvl_it_cpt = 0;
bool pump_to_start = false;
bool pump_to_stop = false;
bool pump_on = false;

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
InterruptIn water_lvl_it(PTD6);
Timeout water_lvl_timeout;
Timeout hc12_send_timeout;
DigitalOut myled(LED1); // Onboard LED
DigitalOut hc12_power(PTD7, 1);
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
// Serial HC12(PTB17,PTB16); // Tx0, Rx0
HC12 hc12_1(PTB17, PTB16, PTB19);

/**
 * Prototypes des fonctions
 */
void water_lvl_it_rise(void);
void water_lvl_it_fall(void);
void water_lvl_timeout_func(void);
void turn_pump_on(void);
void turn_pump_off(void);
void hc12_send_func(void);

/******************************************************************************
 * MAIN
 *****************************************************************************/
int main(void)
{
	FTDI.baud(9600);
	FTDI.format(8, SerialBase::None, 1);
	say_hello();

	hc12_1.initialize();
	hc12_1.clearBuffer();
	hc12_1.sleep();

	water_lvl_it.mode(PullNone);
	water_lvl_it.rise(&water_lvl_it_rise);
	
	
	// hc12_send_timeout.attach(&hc12_send_func, 60);

	while(true)
	{
		myled = myled^1;
		FTDI.printf("\r\n");
		WakeUp::calibrate();
		WakeUp::set_ms(10000); // Reveil dans 10s
		
		// CODE HERE

		// Données des capteurs pour envoi
		sensors.data.temp = 27.2;
		sensors.data.humidity = 88;
		sensors.data.UV = 5;
		sensors.data.UVA = 3.4;
		sensors.data.UVB = 2.6;
		sensors.data.pressure = 1031;


		// END CODE HERE
		
		wait_ms(2000);

		water_lvl_it.disable_irq();
		FTDI.printf("\r\nData to send:");
		for (int i = 0; i < sizeof(sensors.tab); ++i)
		{
			FTDI.printf("\r\n %#x", sensors.tab[i]);
		}
		hc12_1.wakeUp();
		hc12_1.write(sensors.tab, sizeof(sensors.tab));
		wait_ms(500);
		hc12_1.sleep();
		water_lvl_it.enable_irq();
		wait_ms(500);
		
		if (pump_to_start)
			turn_pump_on();
		if (pump_to_stop)
			turn_pump_off();

		// Mise en veille
		FTDI.printf("\r\n - SLEEP -");
		wait_ms(10);
		deepsleep();
		
		Teensy32_Back2Speed_After_PowerDown();
		wait_ms(10);
		FTDI.printf("\r\n - WakeUp -");
	}
}

void water_lvl_it_rise(void) {
	pump_to_start = true;
}

void water_lvl_it_fall(void) {
	water_lvl_timeout.attach(&water_lvl_timeout_func, 2);
}
void water_lvl_timeout_func(void) {
	pump_to_stop = true;
}

void turn_pump_on(void)
{
	pump_to_start = false;
	switch (water_lvl_it_cpt)
	{
		case 0:
			FTDI.printf("\r\nDemarrage de la pompe");
			// Envoyer le signal pour DEMARER la pompe
			pump_on = true;
		break;
		case 2:
			FTDI.printf("\r\nNouveau demarrage de la pompe");
			// Envoyer le signal pour DEMARER la pompe
			water_lvl_it.rise(NULL);
			water_lvl_it.fall(&water_lvl_it_fall);
		break;
	}
	water_lvl_it_cpt++;
}

void turn_pump_off(void)
{
	pump_to_stop = false;
	FTDI.printf("\r\nArret de la pompe");
	// Envoyer le signal pour ARRETER la pompe
	water_lvl_it_cpt = 0;
	pump_on = false;
	water_lvl_timeout.detach();
	water_lvl_it.fall(NULL);
	water_lvl_it.rise(&water_lvl_it_rise);
}

void hc12_send_func(void)
{
	water_lvl_it.disable_irq();
	FTDI.printf("\r\nData to send:");
	for (int i = 0; i < sizeof(sensors.tab); ++i)
	{
		FTDI.printf("\r\n %#x", sensors.tab[i]);
	}
	hc12_1.wakeUp();
	hc12_1.write(sensors.tab, sizeof(sensors.tab));
	wait_ms(500);
	hc12_1.sleep();
	water_lvl_it.enable_irq();
}

