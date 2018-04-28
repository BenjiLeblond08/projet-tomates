/**
 * @file main.cpp
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * 
 * Broches :
 *  BMP280  	SDA = PTC11;
 *  BMP280  	SCL = PTC10;
 *  BMP280  	ADDR = 0b1110111 << 1;
 *  HDC1080 	SDA = PTC11;
 *  HDC1080 	SCL = PTC10;
 *  HDC1080 	ADDR = 0x80; // 0b1000000 << 1
 *  VEML6075	SDA = PTB3;
 *  VEML6075	SCL = PTB2;
 *  VEML6075	ADDR = 0x10;
 *  HC12    	TX = PTB17;
 *  HC12    	RX = PTB16;
 *  FTDI    	TX = PTD3;
 *  FTDI    	RX = PTD2;
 * 
 */

#include "./mbed.h"
#include "BMP280/BMP280.h"
#include "Si7020/Si7020.h"
#include "VEML6075/VEML6075.h"

/**
 * Definition des Variables
 */
int variable_inutile = 42;

/**
 * Definition des Objets
 */
Timer t;
DigitalOut myled(LED1); // Onboard LED
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
Serial HC12(PTB17,PTB16); // Tx0, Rx0
BMP280 BMP280(PTC11, PTC10, 0b1110111 << 1); // SDA1, SCL1, slave_adr
I2C I2C_HDC1080(PTC11, PTC10);
Si7020 HDC1080(&I2C_HDC1080);
VEML6075 VEML6075(PTB3, PTB2, 0x10); // SDA0, SCL0, addr


/**
 * Prototypes des fonctions
 */
void sayHello(void);
void startSequence(void);


/**
 * MAIN
 */
int main(void) 
{
	t.start();
	FTDI.baud(9600);

	float humid;
	float temp;

	// Séquence de démarrage (Regarder la LED)
	startSequence();
	sayHello();

	int i = 0;
	while(i < 4)
	{
		i++;
		// myled = myled^1;
		FTDI.printf("\r\n");
		FTDI.printf("## Execution : %d ##", i);
		FTDI.printf("\r\n");

		// BMP280 
		FTDI.printf("## Test BMP280\r\n");
		FTDI.printf(" Temperature: %f\r\n", BMP280.getTemperature());
		FTDI.printf(" Pressure: %f\r\n", BMP280.getPressure());
		FTDI.printf("\r\n");

		// HDC1080
		FTDI.printf("## Test HDC1080\r\n");
		if(HDC1080.getHumidity(&humid) != 0) {
			FTDI.printf(" Error getting humidity\r\n");
			humid = -1;
		}
		if(HDC1080.getTemperature(&temp) != 0) {
			FTDI.printf(" Error getting temperature\r\n");
			temp = -1;
		} 
		FTDI.printf(" Humidity = %f%% \r\n", humid);
		FTDI.printf(" Temperature = %fC\r\n", temp);
		FTDI.printf("\r\n");

		// VEML6075
		FTDI.printf("## Test VEML6075\r\n");
		FTDI.printf(" UVI = %.4f \r\n", VEML6075.UVI());
		FTDI.printf(" UVA = %.4f \r\n", VEML6075.getUVA());
		FTDI.printf(" UVA_CIE = %.4f \r\n", VEML6075.getUVA_CIE());
		FTDI.printf(" UVB = %.4f \r\n", VEML6075.getUVB());
		FTDI.printf(" UVB_CIE = %.4f \r\n", VEML6075.getUVB_CIE());
		FTDI.printf("\r\n");

		// Fin des test
		FTDI.printf("############\r\n");
		FTDI.printf("# Fin de la boucle\r\n");
		FTDI.printf("# Nouvelle execution dans 1 seconde\r\n");
		FTDI.printf("############\r\n");
		FTDI.printf("\r\n");
		wait(1);
	}

	FTDI.printf("## Fin du programme ##");
	FTDI.printf("   Temps depuis le debut : %f secondes\r\n", t.read());
	FTDI.printf("\r\n");
	FTDI.printf("Faire RESET pour recommencer\r\n");
	
	// Boucle de fin infinie vide
	while(true);

	return 0;
}

/**
 * Fonction sayHello
 */
void sayHello(void)
{
	FTDI.printf("###############################\r\n");
	FTDI.printf("# Hello World                 #\r\n");
	FTDI.printf("# I'm Teensy 3.2              #\r\n");
	FTDI.printf("# Program by Benjamin LEBLOND #\r\n");
	FTDI.printf("###############################\r\n");
}

void startSequence(void)
{
	float endTime = t.read() + 5.0;
	while(t.read() < endTime) {
		wait_ms(200);
		myled = 1;
		wait_ms(800);
		myled = 0;
	}
	myled = 1;
	wait(1);
	myled = 0;
	wait_ms(100);
	myled = 1;
	wait_ms(50);
}