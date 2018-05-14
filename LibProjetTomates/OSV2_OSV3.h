/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * Oregon Scientific V2 and V3 Library
 */

#ifndef OSV2_OSV3_H
#define OSV2_OSV3_H

#include "mbed.h"
#include "WakeUp.h"
#include "clk_freqs.h" // include to check/display clock rates

#define _DEBUG

#ifdef _DEBUG
extern Serial FTDI;
#define DEBUG_PRINT(...) FTDI.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

/** 
 * HC12 class
 */
class OSV2_OSV3
{

/**
 * Prototype des fonctions
 */

void IT_LED(void);

// Variables modifiées par la fonction d'IT
// Pour l'IT toutes les 30s
volatile bool FLAG_IT_MESURE = true;

// FLAG indique si un traitement est a effectuer par le programme principal
volatile bool FLAG = true;

// Prototype des fonctions pour OSV3
void Teensy32_Back2Speed_After_PowerDown(void);
void OSV3_MANCHESTER_SEND_DATA(void);
unsigned char OSV3_CALC_CRC(bool *InitFait);
unsigned char OSV3_CALC_CRC_UV(bool *InitFait);
unsigned char OSV3_CALC_CHECKSUM_THR(void);
unsigned char OSV3_CALC_CHECKSUM_UV(void);
void OSV3_CONSTRUIRE_TRAME_THR(float Temp_f, int HumiHR);
void OSV3_CONSTRUIRE_TRAME_THRP(float Temp_f, int HumiHR, float Pression);
unsigned char OSV3_ROLLING_CODE(void);
void OSV3_MANCHESTER_ENCODE(unsigned char Octet_Encode, bool Fin);
void OSV3_MANCHESTER_SEND_DATA_THR(void);
void OSV3_MANCHESTER_SEND_DATA_UV(void);
void OSV3_INIT_CRC(bool *InitFait);

// Prototype des fonctions pour OSV2
void SendZero(void);
void SendOne(void);
void SendQuarterMSB(unsigned char data);
void SendQuarterLSB(unsigned char data);
void SendData(unsigned char *data, unsigned char size);
void SendOregon(unsigned char *data, unsigned char size);
void SendPreamble(void);
void SendPostamble(void);
void SendSync(void);
void SetType(unsigned char *data, unsigned char *type);
void SetChannel(unsigned char *data, unsigned char channel);
void SetId(unsigned char *data, unsigned char Id);
void SetBatteryLevel(unsigned char *data, unsigned char level);
void SetTemperature(unsigned char *data, float Temp);
void SetHumidity(unsigned char *data, unsigned char Hum);
void SetPressure(unsigned char *data, float pres);
unsigned char Sum(unsigned char count, unsigned char *data);
void calculateAndSetChecksum_T(unsigned char *data);
void calculateAndSetChecksum_THR(unsigned char *data);
void calculateAndSetChecksum_THRP(unsigned char *data);


#endif
