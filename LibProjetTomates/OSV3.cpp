/**
 * Teensy 3.2 USB Serial, test IT par Ticker
 * 
 * Unplug then plug back in the Teensy after programing to reactivate the Teensy USB serial port.
 * if your terminal program can't see the Teensy
 */

#include "OSV3.h"

unsigned char OSV3::TAB_DATA_UV[OSV3_TAILLE_DATA_CAPTEUR_UV] = {0xFF, 0xFF, 0xFF, 0xAD, 0x87, 0x41, 0x57, 0x80, 0x0D, 0x60, 0x64, 0x11};

OSV3::OSV3(PinName out, PinName ain) 
	:
	m_AIN(ain)
{
	m_data_out = new DigitalOut(out, 0);
	Fait_Init_TAB_CRC = false;
}

OSV3::OSV3(DigitalOut *out, PinName ain) 
	:
	m_data_out(out),
	m_AIN(ain)
{
	Fait_Init_TAB_CRC = false;
}

OSV3::~OSV3() { }

unsigned char OSV3::calcCrc(bool *InitFait)
{
	unsigned char CRC = 0;

	// Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	// Inclure les octets de l'indice 4 à 10. Ne pas inclure le CHECKSUM à l'indice 11

	initCrc(InitFait);

	CRC = TAB_CRC_INIT[TAB_DATA_HR[3] & 0x0F];
	// ne sert à rien
	//CRC = CRC & 0xFF;

	for (int i = 4 ; i<11 ; i++)
	{ 
		CRC = TAB_CRC_INIT[CRC ^ TAB_DATA_HR[i]];
		// ne sert à rien
		//CRC = CRC & 0xFF;
	}
	// Permutation des NIBBLES
	return( ((CRC & 0x0F)<<4) | ((CRC & 0xF0)>>4) );
}

unsigned char OSV3::calcCrcUv(bool *InitFait)
{
	unsigned char CRC = 0;

	// Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	// Inclure les octets de l'indice 4 à 9. Ne pas inclure le CHECKSUM à l'indice 10

	initCrc(InitFait);

	CRC = TAB_CRC_INIT[TAB_DATA_UV[3] & 0x0F];
	// ne sert à rien
	//CRC = CRC & 0xFF;

	for (int i = 4 ; i<10 ; i++)
	{ 
		CRC = TAB_CRC_INIT[CRC ^ TAB_DATA_UV[i]];
	// ne sert à rien
	//CRC = CRC & 0xFF;
	}
	// Permutation des NIBBLES
	return( ((CRC & 0x0F)<<4) | ((CRC & 0xF0)>>4) );
}

void OSV3::initCrc(bool *InitFait)
{
// *********************************************************************************************************************************************************
}

unsigned char OSV3::calcChecksumThr(void)
{
	unsigned char CheckSum;
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	
	CheckSum = (TAB_DATA_HR[3] & 0x0F);

	//Inclure les octets de 4 à 10
	for(int i=4 ; i <= OSV3_TAILLE_DATA_CAPTEUR_THR-3 ; i++)
	{
		CheckSum = CheckSum + (TAB_DATA_HR[i] & 0x0F) + ((TAB_DATA_HR[i]>>4) & 0x0F);
	}
	
	//Permutation des NIBBLES
	return( ((CheckSum & 0x0F)<<4) | ((CheckSum & 0xF0)>>4) );
	
}

//Capteur UVN800
unsigned char OSV3::calcChecksumUv(void)
{
	unsigned char CheckSum;
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	
	CheckSum = ( TAB_DATA_UV[3] & 0x0F );

	//Inclure les octets de 4 à 10
	for(int i=4 ; i <= OSV3_TAILLE_DATA_CAPTEUR_UV-3 ; i++)
	{
		CheckSum = CheckSum + (TAB_DATA_UV[i] & 0x0F) + ((TAB_DATA_UV[i]>>4) & 0x0F);
	}
	
	//Permutation des NIBBLES
	return( ((CheckSum & 0x0F)<<4) | ((CheckSum & 0xF0)>>4) );
	
}

void OSV3::construireTrameThr(float Temp_f, int HumiHR)
{
	//Les nibbles sont envoyés LSB first
	
	//Preambule du protocole OSV3
	//24 bits à 1 -> 6 nibbles
	TAB_DATA_HR[0] = 0xFF;
	TAB_DATA_HR[1] = 0xFF;
	TAB_DATA_HR[2] = 0xFF;
	//nibble de synchro -> 0101 -> LSB en 1er soit 0xA0
	TAB_DATA_HR[3] = 0xA0;
	
	//Trame de données du capteur THGR810 -> payload
	//les nibbles 0..3 sont l'ID du capteur qui est unique pour chaque capteur ou commun pour
	//un groupe de capteur.
	//Ici ID du capteur est F824 dans l'ordre de reception
	TAB_DATA_HR[3] = TAB_DATA_HR[3] | 0x0F;
	TAB_DATA_HR[4] = 0x82;
	TAB_DATA_HR[5] = 0x40;
	
	//le nibble 4 pour le CANAL de 1 à 15  
	//Insertion du CANAL
	TAB_DATA_HR[5] = TAB_DATA_HR[5] | OSV3_CANAL_TEMP_HR;

	//Les nibbles 5..6 pour le code tournant dont la valeur est aleatoire
	//à chaque reset du capteur : exemple changement de piles.
	//OSV3_TAB_DATA[6]=OSV3_ROLLING_CODE();
	TAB_DATA_HR[6] = VAL_ROLLING_CODE;
	//Capteur avec bit d'etat de la batterie -> toujours à 0 pour batterie chargée
	//valeur à 1 lorsque la batterie est à changer
	//A changer par une variable pour evolution.
	TAB_DATA_HR[7] = 0x80;
	
	//Les nibbles 8..[N-5] sont les données du capteur
	//Les nibbles 10..8 sont la temperature avec 1 LSB qui represente 0.1 °C
	//exemple : un float de 23.5°C est à transformer en entier de 235
	int temp = (int)(Temp_f*10);
	//Extraction de 5 de 23.5°C
	TAB_DATA_HR[7] = TAB_DATA_HR[7] | ( (abs(temp) % 10) & 0x0F );
	//Extraction de 3 de 23.5°C
	TAB_DATA_HR[8] = ((abs(temp)/10) % 10) << 4;
	//Extraction de 2 de 23.5°C
	TAB_DATA_HR[8] = TAB_DATA_HR[8] | ((abs(temp)/100) & 0x0F);
	//Le nibble 11 represente le signe de la temperature -> une valeur differente de 0 est 
	//une temperature negative
	TAB_DATA_HR[9] = (Temp_f <0) ? 0x80 : 0;
	//Extraction de HD en %
	TAB_DATA_HR[9] = TAB_DATA_HR[9] | ((HumiHR % 10) & 0x0F);
	TAB_DATA_HR[10] = ((HumiHR /10) % 10) <<4 ;
	//Placement du CHECKSUM
	//Le resultat de la somme sur 8 bits des nibbles 0..[N-5]
	//Le CHECKSUM est placé dans [N-3]..[N-4]
	TAB_DATA_HR[11] = calcChecksumThr();
	//Placement du CRC
	TAB_DATA_HR[12] = calcCrc(&Fait_Init_TAB_CRC);
}

unsigned char OSV3::rollingCode(void)
{
	// Lecture d'une entrée analogique sur 16 bits sur la teensy 3.2
	// Ici l'entrée analogique est PTC0 soit A1.
	unsigned short VCAN = m_AIN.read_u16();
	// Initialisation du generateur aléatoire
	srand(VCAN);

	// Nombre aleatoire entre 1 et 254.
	return((rand() % 253) + 1);
}

void OSV3::manchesterEncode(unsigned char Octet_Encode, bool Fin)
{
	unsigned char MASK = 0x10;
	// Timer pour la gestion du temps.
	// Lecture de la durée
	static int TimeBase = TimeManchester.read_us();

	// OSV3 emet à la frequence de 1024Hz ou 1024bit/s
	// Prevoir un ajustement en fonction du temp de traitement par le uC
	// En mesure : 1020Hz et donc 490us
	const unsigned int DureeDesire = 490;
	// Valeur ajustemet du au temps de traitement
	// Reduction de DureeDesiree
	const unsigned int ReduireDe = 32;
	// Mode PowerDown -> plus lent et on mesure 764, 5Hz au lieu de 1020Hz
	// Au reveil, la Teensy est à 72MHz au lieu de 96MHz -> 96/72 = 1.33
	//const unsigned int DureeDesire = 367;
	//const unsigned int ReduireDe = 24;

	// Les bits sont transmis de 4 à 7 puis de 0 à 3
	for(int i = 0 ; i<8 ; i++)
	{
		TimeBase = TimeBase + DureeDesire;

		unsigned int CalculRetard = TimeBase-TimeManchester.read_us();

		//FTDI.printf("\r\n\n CalculRetard =%d | TimeBase =%d", CalculRetard, TimeBase);

		if(CalculRetard > 2*DureeDesire)  {
			// Retard trop grand indique un break entre la transmission : reset de TimeBase
			TimeBase = TimeManchester.read_us();
			//FTDI.printf("\r\n\n CalculRetard > 2*DureeDesire ->TimeBase =%d", TimeBase);
		}
		else {
			if(CalculRetard > 0) wait_us(CalculRetard);
			//FTDI.printf("\r\n\n CalculRetard > 0 ->TimeBase =%d", CalculRetard);
		}


		if((Octet_Encode & MASK)== 0) {
			// Un 0 est représenté par une transition de 0 à 1 dans le signal RF
			// Mise à 0 de la broche
			m_data_out->write(0);
			wait_us(DureeDesire-ReduireDe);
			m_data_out->write(1);

			// Test si dernier bit
			// Dans ce cas pas de retard long apres la transition de 0 à 1 
			// pour indiquer que plus de donnée à suivre
			if(Fin) wait_us(DureeDesire);

		}
		else {
			m_data_out->write(1);
			wait_us(DureeDesire-ReduireDe);
			m_data_out->write(0);
		}

		if(MASK == 0x80) MASK = 0x01;
		else MASK = MASK<<1;

		TimeBase = TimeBase + DureeDesire;
	}
}


void OSV3::sendData(unsigned char* data)
{
	int size = sizeof(data);
	TimeManchester.reset();
	TimeManchester.start();

	for(int i = 0 ; i < size ; i++)
	{
		manchesterEncode(data[i], size == i+1);
	}

	TimeManchester.stop();
}

void manchesterSendDataThr(void)
{
// ***********************************************************************************************************************************************
}

void manchesterSendDataUv(void)
{
// ***********************************************************************************************************************************************
}
