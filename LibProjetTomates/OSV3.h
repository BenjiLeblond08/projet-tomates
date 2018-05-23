/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * Oregon Scientific V2 and V3 Library
 */

#ifndef _OSV3_H
#define _OSV3_H

#include "mbed.h"
#include "WakeUp.h"
#include "clk_freqs.h" // include to check/display clock rates

#define _DEBUG

#ifdef _DEBUG
extern Serial FTDI;
#define DEBUG_PRINT(...) FTDI.printf("\r\n"__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

/** 
 * Gestion du 433MHz en Manchester
 */
#define OSV3_CRC8_M433_GP 0x107 //x^8+x^2+x+1
#define OSV3_CRC8_M433_DI 0x07
//Taille du tableau pour les données du capteur 
//Le capteur est temperature en °C et HR en % -> protocole Oregon Scientific V3 : THGR810
#define OSV3_TAILLE_DATA_CAPTEUR_THR 13
//Le capteur d'UV  -> protocole Oregon Scientific V3 : UVN800
#define OSV3_TAILLE_DATA_CAPTEUR_UV 12
//Possibilité de definir un canal de 1 à 15 : Oregon de 1 à 3
#define OSV3_CANAL_TEMP_HR 2
#define OSV3_CANAL_HAUTEUR_EAU 3
#define OSV3_CANAL_UV 9
#define DUREE_ENTRE_CHAQUE_MESURE_OSV3 9

/** 
 * OSV3 class
 */
class OSV3
{
public:
	
	/**
	 * OSV3 Class constructor
	 */
	OSV3(PinName out, PinName ain);
	OSV3(DigitalOut *out, PinName ain);

	/**
	 * Class Destructor
	 */
	~OSV3();


	unsigned char calcCrc(bool *InitFait);
	
	unsigned char calcCrcUv(bool *InitFait);
	
	void initCrc(bool *InitFait);
	
	unsigned char calcChecksumThr(void);
	
	unsigned char calcChecksumUv(void);
	
	void construireTrameThr(float Temp_f, int HumiHR);
		
	unsigned char rollingCode(void);
	
	void manchesterEncode(unsigned char Octet_Encode, bool Fin);

	void sendData(unsigned char* data);
	void manchesterSendDataThr(void);
	void manchesterSendDataUv(void);
	
	static unsigned char TAB_DATA_UV[OSV3_TAILLE_DATA_CAPTEUR_UV]; // = {0xFF, 0xFF, 0xFF, 0xAD, 0x87, 0x41, 0x57, 0x80, 0x0D, 0x60, 0x64, 0x11};

private:

	/**
	 * Sortie pour l'emission de la data en 433.92MHz
	 */
	DigitalOut* m_data_out;
	// Entrée analogique pour le CAN, utilisé pour RollingCode
	AnalogIn m_AIN;

	Timer TimeManchester;

	unsigned char TAB_DATA_HR[OSV3_TAILLE_DATA_CAPTEUR_THR];
	unsigned char TAB_CRC_INIT[256];
	unsigned char VAL_ROLLING_CODE;
	bool Fait_Init_TAB_CRC;

};

#endif
