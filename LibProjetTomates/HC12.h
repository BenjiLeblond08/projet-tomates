/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * HC12 Library
 * 
 * @author  Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * @version 1.0
 * @date    21-May-2018
 * 
 */

#ifndef _HC12_H
#define _HC12_H

#include "mbed.h"

#define _DEBUG

#ifndef DEBUG_PRINT
#ifdef _DEBUG
extern Serial FTDI;
#define DEBUG_PRINT(...) FTDI.printf("\r\n"__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

// Gestion des commandes AT
#define HC12_END_CMD ("\r")
#define HC12_CR (0x0D)
#define HC12_RESP_OK ("OK")
#define HC12_AT	"AT"
#define HC12_ERROR	"ERROR"
// Taille maximale de 60 octets en mode FU4 pour envoyer des données
// Ici buffer pour la configuration : min de 8 (AT+Bxxxx) et en plus CR (HC12_END_CMD) en fin de commande
// 41 octets au minimum pour recevoir la reponse à AT+RX
#define HC12_TAILLE_BUFFER 60
#define HC12_TIMEOUT 1
// FU4 -> AT+FU4
#define HC12_FU4 "AT+FU4"
#define HC12_FU4_Rep "OK+FU4"
// 100mW -> AT+P8
#define HC12_100mW "AT+P8"
#define HC12_100mW_Rep "OK+P8"
// 433.4MHz -> AT+C001
#define HC12_CANAL1 "AT+C001"
#define HC12_CANAL1_Rep "OK+C001"
// 441.4MHz -> AT+C021
#define HC12_CANAL21 "AT+C021"
#define HC12_CANAL21_Rep "OK+C021"
// 1200bauds -> AT+B1200
#define HC12_B1200 "AT+B1200"
#define HC12_B1200_Rep "OK+B1200"
// Verification Configuration
#define HC12_Config "AT+RX"
// Mode SLEEP
#define HC12_SLEEP "AT+SLEEP"
#define HC12_SLEEP_Rep "OK+SLEEP"
// Mode DEFAULT - Reset Factory - 9600bit/s - 8bits - pas parité - 1 bit stop - communication canal 001 (433.4MHz) - Power 20dBm - FU3 
#define HC12_DEFAULT "AT+DEFAULT"
#define HC12_DEFAULT_Rep "OK+DEFAULT"

/** 
 * HC12 class
 */
class HC12
{

public:

	/**
	 * HC12 Constructor
	 * 
	 * @param PinName tx
	 * @param PinName rx
	 * @param PinName cs
	 */
	HC12(PinName tx, PinName rx, PinName cs);
	HC12(Serial* hc12, DigitalOut* cs);
	HC12(Serial* hc12, PinName cs);

	/**
	 * HC12 Destructor
	 */
	~HC12();

	void initialize(void);

	void clearBuffer(void);

    /** Attach a function to call whenever a serial interrupt is generated
     *
     *  @param fptr A pointer to a void function, or 0 to set as none
     *  @param type Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    void attach(void (*fptr)(void), Serial::IrqType type = Serial::RxIrq);

	bool writeable(void);

	bool readable(void);

	/**
	 * 
	 * 
	 * @param char* data Byte(s) to write
	 * 
	 * @return int Number of writed bytes
	 */
	int write(char *data, int size);

	/**
	 * 
	 * 
	 * @param char* data array to put readed bytes
	 * @param int ndata Number of bytes to read
	 * 
	 * @return int Number of readed bytes
	 */
	int read(char* data, int ndata = 1);

	void sleep(void);
	void wakeUp(void);


private:

	/**
	 * sendATcommand
	 * 
	 * @param char* ATcommand
	 * @param char* expected_answer
	 * @param float timeout
	 * 
	 * @return bool true if command sent successfully, false otherwise
	 */
	bool sendATcommand(char *ATcommand, char *expected_answer, float timeout);

	/**
	 * enterCMDMode
	 * Logical level HIGH during more than 40ms on pin CS_HC12
	 * 
	 * @param none
	 * 
	 * @return none
	 */
	void enterCMDMode(void);

	/**
	 * leaveCMDMode
	 * Logical level LOW during more than 80ms on pin CS_HC12
	 * 
	 * @param none
	 * 
	 * @return none
	 */
	void leaveCMDMode(void);

	void enterModeAT(void);

private:

	Serial* m_hc12;
	DigitalOut* m_cs;

	bool m_cmd_mode;

	char m_CMD_AT[HC12_TAILLE_BUFFER];
	char m_reponse_AT[HC12_TAILLE_BUFFER];
};

#endif
