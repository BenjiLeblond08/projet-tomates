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

#include "mbed.h"
#include "clk_freqs.h"

#include "tomates.h"
#include "HC12.h"
#include "OSV3.h"
#include "OSV2.h"

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
Serial FTDI(PTD3,PTD2); // FTDI_Tx, FTDI_Rx
Serial hc12_serial(PTB17, PTB16);
DigitalOut hc12_cs(PTB19);
// HC12 hc12_1(&hc12_serial, PTB19);

/**
 * Prototypes des fonctions
 */
void turn_pump_on(void);
void turn_pump_off(void);
void hc12_it_rx(void);
void hc12_initialize(void);
bool hc12_sendATcommand(char* ATcommand, char* expected_answer, float timeout);
void hc12_enterCMDMode(void);
void hc12_leaveCMDMode(void);

/******************************************************************************
 * MAIN
 *****************************************************************************/
int main(void)
{
	// t.start();
	FTDI.baud(9600);
	FTDI.format(8, SerialBase::None, 1);
	hc12_serial.baud(9600);
	hc12_serial.format(8, SerialBase::None, 1);
	wait_ms(1);
	say_hello();

	pump_it.mode(PullNone);
	pump_it.rise(&turn_pump_on);
	
	mise_a_l_heure(MERCREDI, 9, MAI, 2018, 0, 0, 0);
	
	hc12_initialize();

	hc12_serial.attach(&hc12_it_rx, Serial::RxIrq);

	FTDI.printf("\r\n # DEBUT BOUCLE #\r\n");
	while(true)
	{
		myled = myled^1;
		// FTDI.printf("\r\n");
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

	pump_timeout.attach(&turn_pump_off, 2.5);
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
	int i = 0;
	char readed_char;
	FTDI.printf("Reception 2 HC12\r\n");

	// readed_char = hc12_serial.getc();
	// DEBUG_PRINT("readed_char: %c", hc12_serial.getc());

	FTDI.printf("Reception 3 HC12\r\n");
	while(hc12_serial.readable() && i < sizeof(sensors))
	{
		DEBUG_PRINT("HC12->read: i=%d\r\n", i);
		DEBUG_PRINT("HC12->readable: %d\r\n", hc12_serial.readable());
		// readed_char = hc12_serial.getc();
		// DEBUG_PRINT("readed_char: %c", hc12_serial.getc());
		// &data[i++] = hc12_serial.getc();
		i = i + 1;
	}
	// hc12_1.read(sensors.tab, sizeof(sensors));
	FTDI.printf("FIN Reception HC12\r\n");
}

void hc12_initialize(void)
{
	char m_CMD_AT[HC12_TAILLE_BUFFER];
	char m_reponse_AT[HC12_TAILLE_BUFFER];

	DEBUG_PRINT("HC12 initialize\r\n");
	hc12_enterCMDMode();

	//Entrer en mode AT -> "AT"
	sprintf(m_CMD_AT, HC12_AT);
	sprintf(m_reponse_AT, HC12_RESP_OK);
	if(hc12_sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=OK\r\n");
	else 
	{
		DEBUG_PRINT("sendATcommand=ERROR\r\n");
		DEBUG_PRINT("Changement vitesse en 1200bauds\r\n");
		// Changement vitesse transmission RS232 suite nouvelle configuration.
		// Configuration liaison serie
		// reglage usine : 1200 bauds
		hc12_serial.baud(1200);
		// Tempo pour mise en service de la nouvelle vitesse de la RS232.
		wait_ms(200);
	}

	// Configuration mode FU4 -> "AT+FU4"
	sprintf(m_CMD_AT, HC12_FU4);
	sprintf(m_reponse_AT, HC12_FU4_Rep);
	if(hc12_sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=AT+FU4\r\n");
	else DEBUG_PRINT("sendATcommand=ERROR\r\n");

	//Configuration mode 100mW -> "AT+P8"
	sprintf(m_CMD_AT, HC12_100mW);
	sprintf(m_reponse_AT, HC12_100mW_Rep);
	if(hc12_sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=AT+P8\r\n");
	else DEBUG_PRINT("sendATcommand=ERROR\r\n");

	/**
	 * Canal 1 - 433.1MHz
	 * Configuration mode 433.1MHz -> "AT+C001"
	 */
	//sprintf(m_CMD_AT, HC12_CANAL1);
	//sprintf(m_reponse_AT, HC12_CANAL1_Rep);
	//if(hc12_sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=AT+C001\r\n");
	//else DEBUG_PRINT("sendATcommand=ERROR\r\n");

	/**
	 * Canal 21 - 441.4MHz
	 * Configuration mode 441.4MHz -> "AT+C021"
	 */
	sprintf(m_CMD_AT, HC12_CANAL21);
	sprintf(m_reponse_AT, HC12_CANAL21_Rep);
	if(hc12_sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=AT+C021\r\n");
	else DEBUG_PRINT("sendATcommand=ERROR\r\n");

	// Configuration mode 1200bauds -> "AT+B1200"
	sprintf(m_CMD_AT, HC12_B1200);
	sprintf(m_reponse_AT, HC12_B1200_Rep);
	if(hc12_sendATcommand(m_CMD_AT, m_reponse_AT , HC12_TIMEOUT)) DEBUG_PRINT("sendATcommand=AT+B1200\r\n");
	else DEBUG_PRINT("sendATcommand=ERROR\r\n");

	hc12_leaveCMDMode();
}

bool hc12_sendATcommand(char* ATcommand, char* expected_answer, float timeout)
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
	while(!hc12_serial.writeable());
	// Preparation de la commande AT et terminée par CR non necessaire ici
	sprintf(buffer, "%s%s", ATcommand, HC12_END_CMD);
	
	hc12_serial.printf("%s", buffer);

	// Preparation du Timeout si la reponse du HC12 ne vient pas...
	t.reset();
	t.start();
	
	debut = t.read();

	// Gestion de la reponse du HC12
	do
	{
		// if there are data in the UART input buffer, reads it and checks for the asnwer
		if(hc12_serial.readable())
		{
			reponse[i++] = (char)hc12_serial.getc();
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


void hc12_enterCMDMode(void)
{
	// NL0 pendant >40ms
	hc12_cs.write(0);
	wait_ms(41);
}

void hc12_leaveCMDMode(void)
{
	// NL1 et attente MAJ pendant >80ms avant retout mode transparent
	hc12_cs.write(1);
	wait_ms(81);
}

