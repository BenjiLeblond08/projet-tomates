/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * @file main_hc12.cpp
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * 
 */

#include "mbed.h"
#include "WakeUp.h"
#include "clk_freqs.h"

#include "tomates.h"
#include "HC12.h"
// #include "BMP280.h"
// #include "HDC1080.h"
// #include "VEML6075.h"

/**
 * Definition des Variables
 */
int water_lvl_it_cpt = 0;
bool pump_to_start = false;
bool pump_to_stop = false;
bool pump_on = false;

/**
 * Tableau pour envoie HC12
 */
sensors_union sensors;

/**
 * Definition des Objets
 */
DigitalOut myled(LED1); // Onboard LED
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
/* hc12 */
DigitalOut hc12_power(PTD7, 1);
HC12 hc12_1(PTB17, PTB16, PTB19);
/* Detection Niveau Eau */
InterruptIn water_lvl_it(PTD6);
Timeout water_lvl_timeout;
Timeout hc12_send_timeout;
/* Capteurs */
I2C i2c0(PTB3,PTB2);
I2C i2c1(PTC11,PTC10);

/**
 * Prototypes des fonctions
 */
void water_lvl_it_rise(void);
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
		WakeUp::set_ms(10000); // WakeUp after 10s

		/* Données des capteurs pour envoi */
		sensors.data.temp_bmp = 26.49;
		sensors.data.pressure = 1000.8100;
		sensors.data.altitude = 44330 * (1.0 - pow((sensors.data.pressure/1013.25), 0.1903));
		sensors.data.temp_hdc = 26.2256;
		sensors.data.humidity = 45.4468;
		sensors.data.UVA      = 3.5;
		sensors.data.UVB      = 2.7;
		sensors.data.UVI      = 5.2;
		sensors.data.pump_on  = (int)pump_on;

		print_sensors_data(sensors);

		wait_ms(1000);

		FTDI.printf("\r\nData to send:");
		for (int i = 0; i < sizeof(sensors.tab); i=i+4) {
			FTDI.printf("\r\ntab[%-2d]->[%-2d] (hex) = %-2x %-2x %-2x %-2x", i, i+3, sensors.tab[i], sensors.tab[i+1], sensors.tab[i+2], sensors.tab[i+3]);
		}
		water_lvl_it.disable_irq();
		hc12_1.wakeUp();
		hc12_1.write(sensors.tab, sizeof(sensors.tab));
		hc12_1.sleep();
		water_lvl_it.enable_irq();
		FTDI.printf("\r\nData sended");

		if (pump_to_start)
			turn_pump_on();
		if (pump_to_stop)
			turn_pump_off();

		wait_ms(1000);

		/* Mise en veille */
		FTDI.printf("\r\n - SLEEP -");
		wait_ms(10);
		deepsleep();

		Teensy32_Back2Speed_After_PowerDown();
		wait_ms(10);
		FTDI.printf("\r\n - WakeUp -");
	}
}

void water_lvl_it_rise(void) {
	if (water_lvl_it_cpt < 3)
		pump_to_start = true;
	water_lvl_timeout.attach(&water_lvl_timeout_func, 5);
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
	water_lvl_it.rise(&water_lvl_it_rise);
}

void hc12_send_func(void)
{
	FTDI.printf("\r\nData to send:");
	for (int i = 0; i < sizeof(sensors.tab); i=i+4) {
		FTDI.printf("\r\ntab[%-2d]->[%-2d] (hex) =", i, i+3);
		FTDI.printf(" %-2x", sensors.tab[i]);
		FTDI.printf(" %-2x", sensors.tab[i+1]);
		FTDI.printf(" %-2x", sensors.tab[i+2]);
		FTDI.printf(" %-2x", sensors.tab[i+3]);
	}
	water_lvl_it.disable_irq();
	hc12_1.wakeUp();
	hc12_1.write(sensors.tab, sizeof(sensors.tab));
	hc12_1.sleep();
	water_lvl_it.enable_irq();
	FTDI.printf("\r\nData sended");
}
