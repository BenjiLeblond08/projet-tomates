/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * Oregon Scientific V2 and V3 Library
 */

#ifndef _OSV2_H
#define _OSV2_H

#include "mbed.h"
#include "WakeUp.h"
#include "clk_freqs.h" // include to check/display clock rates

#define _DEBUG

#ifdef _DEBUG
extern Serial FTDI;
#define debugPrint(...) FTDI.printf(__VA_ARGS__)
#else
#define debugPrint(...)
#endif

#define OSV2_TIME 512
#define OSV2_TWOTIME OSV2_TIME*2
#define SEND_HIGH() m_data_out->write(1);
#define SEND_LOW() m_data_out->write(0);
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

#define OSV2_CANAL_TEMP_HR_P 6
#define OSV2_CANAL_TEMP 3
#define OSV2_CANAL_TEMP_HR 7


/** 
 * OSV2 class
 */
class OSV2
{
public:
	
	/**
	 * OSV2 Class constructor
	 */
	OSV2(PinName out);
	OSV2(DigitalOut *out);

	/**
	 * Class Destructor
	 */
	~OSV2();

	void sendZero(void);
	void sendOne(void);
	void sendQuarterMSB(unsigned char data);
	void sendQuarterLSB(unsigned char data);
	void sendData(unsigned char *data, unsigned char size);
	void sendOregon(unsigned char *data, unsigned char size);
	void sendPreamble(void);
	void sendPostamble(void);
	void sendSync(void);
	void setType(unsigned char *data, unsigned char *type);
	void setChannel(unsigned char *data, unsigned char channel);
	void setID(unsigned char *data, unsigned char Id);
	void setBatteryLevel(unsigned char *data, unsigned char level);
	void setTemperature(unsigned char *data,float Temp);
	void setHumidity(unsigned char *data,unsigned char Hum);
	void setPressure(unsigned char *data, float pres);
	unsigned char sum(unsigned char count, unsigned char *data);
	void calculateAndSetChecksumT(unsigned char *data);
	void calculateAndSetChecksumTHR(unsigned char *data);
	void calculateAndSetChecksumTHRP(unsigned char *data);

private:

	/**
	 * Sortie pour l'emission de la data en 433.92MHz
	 */
	DigitalOut* m_data_out;

	Timer TimeManchester;
};

#endif


