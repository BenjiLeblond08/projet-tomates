/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * HC12 Library
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * @version 1.0
 * @date    21-May-2018
 * 
 */

#include "mbed.h"
#include "HC12.h"


HC12::HC12(PinName tx, PinName rx, PinName cs)
	:
	m_hc12(new Serial(tx, rx)), // Liaison serie via TX0(PTB17) RX0(PTB16)
	m_cs(new DigitalOut(cs, 1)),
	m_cmd_mode(false)
{
	m_hc12->baud(9600);
	m_hc12->format(8, SerialBase::None, 1);
}

HC12::HC12(Serial* hc12, DigitalOut* cs)
	:
	m_hc12(hc12),
	m_cs(cs),
	m_cmd_mode(false)
{
	m_hc12->baud(9600);
	m_hc12->format(8, SerialBase::None, 1);
}

HC12::HC12(Serial* hc12, PinName cs)
	:
	m_hc12(hc12),
	m_cs(new DigitalOut(cs, 1)),
	m_cmd_mode(false)
{	
	m_hc12->baud(9600);
	m_hc12->format(8, SerialBase::None, 1);
}

HC12::~HC12() { }


void HC12::initialize(void)
{
	DEBUG_PRINT("HC12 init...");
	enterCMDMode();

	enterModeAT();

	// Config mode FU4 -> "AT+FU4"
	sprintf(m_CMD_AT, HC12_FU4);
	sprintf(m_reponse_AT, HC12_FU4_Rep);
	if(sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT))
		DEBUG_PRINT("sendATcommand=AT+FU4");
	else
		DEBUG_PRINT("sendATcommand=ERROR");

	// Config mode 100mW -> "AT+P8"
	sprintf(m_CMD_AT, HC12_100mW);
	sprintf(m_reponse_AT, HC12_100mW_Rep);
	if(sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT))
		DEBUG_PRINT("sendATcommand=AT+P8");
	else
		DEBUG_PRINT("sendATcommand=ERROR");

	/**
	 * Canal 1 - 433.1MHz
	 * Config mode 433.1MHz -> "AT+C001"
	 */
	//sprintf(m_CMD_AT, HC12_CANAL1);
	//sprintf(m_reponse_AT, HC12_CANAL1_Rep);
	//if(sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=AT+C001");
	//else DEBUG_PRINT("sendATcommand=ERROR");

	/**
	 * Canal 21 - 441.4MHz
	 * Config mode 441.4MHz -> "AT+C021"
	 */
	sprintf(m_CMD_AT, HC12_CANAL21);
	sprintf(m_reponse_AT, HC12_CANAL21_Rep);
	if(sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT))
		DEBUG_PRINT("sendATcommand=AT+C021");
	else
		DEBUG_PRINT("sendATcommand=ERROR");

	// Config mode 1200bauds -> "AT+B1200"
	sprintf(m_CMD_AT, HC12_B1200);
	sprintf(m_reponse_AT, HC12_B1200_Rep);
	if(sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT))
		DEBUG_PRINT("sendATcommand=AT+B1200");
	else
		DEBUG_PRINT("sendATcommand=ERROR");

	leaveCMDMode();

}

void HC12::clearBuffer(void)
{
	while(m_hc12->readable()) {
	    m_hc12->getc();
	}
}

void HC12::attach(void (*fptr)(void), Serial::IrqType type)
{
	m_hc12->attach(*fptr, type);
}

bool HC12::writeable(void)
{
	return m_hc12->writeable();
}

bool HC12::readable(void)
{
	return m_hc12->readable();
}

int HC12::write(char *data, int size)
{
	int i;

	// m_hc12->putc(0xaa);
	// m_hc12->putc(0xaa);

	for(i = 0; i < size; i++)
	{
		while(!m_hc12->writeable());
		m_hc12->putc(data[i]);
	}
	wait_ms(size*20); // Wait for end of transmission
	return i;
}

int HC12::read(char* data, int ndata)
{
	int i = 0;

	// m_hc12->getc();

	do
	{
		data[i++] = m_hc12->getc();
	} while (m_hc12->readable() || i < ndata);

	return i;
}

void HC12::sleep(void)
{
	enterCMDMode();

	enterModeAT();

	// Enter mode SLEEP -> "AT+SLEEP"
	sprintf(m_CMD_AT, HC12_SLEEP);
	sprintf(m_reponse_AT, HC12_SLEEP_Rep);
	if(sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=AT+SLEEP");
	else DEBUG_PRINT("sendATcommand=ERROR");

	leaveCMDMode();
}

void HC12::wakeUp(void)
{
	enterCMDMode();
	// wait_us(5);
	leaveCMDMode();
}


bool HC12::sendATcommand(char* ATcommand, char* expected_answer, float timeout)
{
	int i = 0;
	float debut;  
	bool answer = false;
	
	static char buffer[HC12_TAILLE_BUFFER];
	static char reponse[HC12_TAILLE_BUFFER];

	Timer t;
	// Initialisation par le '\0' sur la totalité
	memset(reponse, '\0', HC12_TAILLE_BUFFER);
	//memset(buffer, '\0', HC12_TAILLE_BUFFER);

	// Attente si ecriture impossible via RS232 pour configurer HC12
	while(!m_hc12->writeable());
	// Preparation de la commande AT et terminée par CR non necessaire ici
	sprintf(buffer, "%s%s", ATcommand, HC12_END_CMD);
	
	m_hc12->printf("%s", buffer);

	// Preparation du Timeout si la reponse du HC12 ne vient pas...
	t.reset();
	t.start();
	
	debut = t.read();

	// Gestion de la reponse du HC12
	do
	{
		// if there are data in the UART input buffer, reads it and checks for the asnwer
		if(m_hc12->readable())
		{
			reponse[i++] = (char)m_hc12->getc();
			// test si la reponse est bien OK
			if (strstr(reponse, expected_answer) != NULL)
				answer = true;
		}
	} // Test si la reponse OK est bien presente ou le Timeout est terminé
	while((answer == false) && ((t.read() - debut) < timeout));

	// Arret du Timer
	t.stop();

	// DEBUG
	//DEBUG_PRINT("\r\nTX = %s | RX = %s",buffer,reponse);
	//DEBUG_PRINT("\r\nTimeout = %fs",t.read() - debut);

	// true ou false -> true si OK bien present et false dans l'autre cas
	return (answer);
}


void HC12::enterCMDMode(void)
{
	// NL0 pendant >40ms
	m_cs->write(0);
	m_cmd_mode = true;
	wait_ms(41);
}

void HC12::leaveCMDMode(void)
{
	// NL1 et attente MAJ pendant >80ms avant retout mode transparent
	m_cs->write(1);
	m_cmd_mode = false;
	wait_ms(81);
}

void HC12::enterModeAT(void)
{
	if(!m_cmd_mode)
		return;

	// Enter mode AT -> "AT"
	sprintf(m_CMD_AT, HC12_AT);
	sprintf(m_reponse_AT, HC12_RESP_OK);
	if(sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT))
		DEBUG_PRINT("sendATcommand=OK");
	else
	{
		DEBUG_PRINT("sendATcommand=ERROR");
		DEBUG_PRINT("Changement vitesse en 1200bauds");
		// Changement vitesse transmission RS232 suite nouvelle configuration.
		// Configuration liaison serie
		// reglage usine : 1200 bauds
		m_hc12->baud(1200);
		// Tempo pour mise en service de la nouvelle vitesse de la RS232.
		wait_ms(200);
	}
}
