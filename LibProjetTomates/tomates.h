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

#ifndef TOMATES_H
#define TOMATES_H

#define TEST_PROF

#include "mbed.h"

#define _DEBUG

#ifdef _DEBUG
extern Serial FTDI;
#define DEBUG_PRINT(...) FTDI.printf("\r\n"__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif


/**
 * Utilisation de l'horloge RTC
 */
#define DIMANCHE 0
#define LUNDI 1
#define MARDI 2
#define MERCREDI 3
#define JEUDI 4
#define VENDREDI 5
#define SAMEDI 6

#define JANVIER 0
#define FEVRIER 1
#define MARS 2
#define AVRIL 3
#define MAI 4
#define JUIN 5
#define JUILLET 6
#define AOUT 7
#define SEPTEMBRE 8
#define OCTOBRE 9
#define NOVEMBRE 10
#define DECEMBRE 11


typedef enum {
	THGR810   = 1, // Thermo-Hygro - OSV3
	THGR2228N = 2, // Thermo-Hygro - OSV2
	UVN800    = 3, // UV - OSV3
	BTHR918N  = 4, // Thermo-hygro-baromètre - OSV2
	THN132N   = 5  // Thermo - OSV2
} OSV23_sensors;
							 
struct T_DB {
	time_t time;               // 4 octets
	float temp;                // 4 octets
	unsigned int HR;           // 4 octets
	unsigned int UV;           // 4 octets
	int pressure;              // 4 octets
	int rolling_code;          // 4 octets
	OSV23_sensors sensor_type; // 4 octets
}; // size = 28 octets

typedef union {
	T_DB Data_Capteur;
	char Tab_TU[sizeof(T_DB)]; // meme taille que T_DB et au même emplacement mémoire.
} TU_DB;

struct data_sensors {
	float temp;            // 4 octets
	unsigned int humidity; // 4 octets
	unsigned int UV;       // 4 octets
	float UVA;             // 4 octets
	float UVB;             // 4 octets
	int pressure;          // 4 octets
}; // size = 24 octets

typedef union {
	struct data_sensors data;
	char tab[sizeof(struct data_sensors)];
} sensors_union;


/**
 * Definition des Variables
 */


/**
 * Fonction say_hello
 */
void say_hello(void);

#ifndef TEST_PROF
/**
 * Fonction pour obtenir la configuration initiale des frequences de la Teensy 3.2
 * Suite au mode PowerDown, la Teensy ne fonctionne qu'à 72MHz
 */
void Teensy32_Back2Speed_After_PowerDown(void);
#endif

void mise_a_l_heure(int wday = LUNDI, int mday = 7, int mon = MAI, int year = 2018, int hour = 0, int min = 0, int sec = 0);

#endif
