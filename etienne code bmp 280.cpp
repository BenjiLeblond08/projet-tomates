// Teensy 3.2 USB Serial

//A lire


// Unplug then plug back in the Teensy after programing to reactivate the Teensy USB serial port.
// if your terminal program can't see the Teensy

//A lire pour SPI Hardware
//https://riot-os.org/api/group__cpu__kinetis__common__spi.html
//https://mcuoneclipse.com/2016/01/09/how-to-add-bluetooth-low-energy-ble-connection-to-arm-cortex-m/

//A lire pour carte SD
//https://developer.mbed.org/cookbook/SD-Card-File-System
//http://langster1980.blogspot.com/2014/01/mbed-sd-card-tutorial.html

//Lien SD LIB plus recente
//Recherche : https://developer.mbed.org/search/?q=sdfile
//https://developer.mbed.org/questions/77810/OS5-K64F-and-SDFileSystem-gets-online-co/#answer12519
//https://developer.mbed.org/users/infinnovation/code/SDFileSystem/file/cf988cb4fad9/SDFileSystem.h
//https://developer.mbed.org/users/infinnovation/code/sd_test_5/
//Import -> https://developer.mbed.org/users/infinnovation/code/SDFileSystem/#cf988cb4fad9
//https://developer.mbed.org/users/infinnovation/code/SDFileSystem/file/cf988cb4fad9/SDFileSystem.h

//Modification adaptateur SD
//https://forum.arduino.cc/index.php?topic=360325.0
//http://forum.arduino.cc/index.php?topic=360718.msg2942160#msg2942160
//https://os.mbed.com/forum/mbed/topic/101/

//A lire MOS
//http://www.edaboard.com/thread54696.html

//A lire pour gestion des string en C par string.h
//https://www.tutorialspoint.com/c_standard_library/string_h.htm

//A lire sur XBee simple
//https://os.mbed.com/users/tristanjph/code/xbee_lib/file/6455a079bdb3/xbee.cpp
//https://os.mbed.com/users/vcazan/notebook/mbed--xbee/

//A lire mbed  1 wire DigitalInOut
//https://os.mbed.com/forum/mbed/topic/101/
//https://os.mbed.com/users/simon/code/OneWireDriver/file/d0b35e0b4294/main.cpp
//Exemple Arduino
//https://www.carnetdumaker.net/articles/mesurer-une-temperature-avec-un-capteur-1-wire-ds18b20-et-une-carte-arduino-genuino/
//Exemple Arduino par Maxim
//https://www.maximintegrated.com/en/app-notes/index.mvp/id/162
//demo par Maxim
//https://os.mbed.com/teams/Maxim-Integrated/code/OneWire_DS18B20_Demo/file/343505f6da8d/main.cpp
//Exemple de code
//https://os.mbed.com/users/alpov/code/OneWire/docs/445fe6e6bd68/1wire_8cpp_source.html
//https://os.mbed.com/users/alpov/code/OneWire/docs/445fe6e6bd68/1wire_8h_source.html
//Code PJRC pour 1 wire
//Exemple d'utilisation :
//https://www.pjrc.com/teensy/td_libs_OneWire.html
//https://github.com/PaulStoffregen/OneWire/blob/master/OneWire.cpp
//https://github.com/PaulStoffregen/OneWire/blob/master/OneWire.h

//A lre sur RJ45
//http://www.frank-zhao.com/thingspeak_mbed_tut1/

//A lire Bus CAN Teensy 3.2
//https://github.com/collin80/FlexCAN_Library

//A lire portabilite arduino vers mbed
//https://os.mbed.com/questions/61104/how-to-convert-arduino-code-to-mbed-libr/

//A lire Arduino millis()
//https://os.mbed.com/questions/61002/Equivalent-to-Arduino-millis/

//A lire - VEML6075
//https://github.com/Jan--Henrik/VEML6075
//https://forum.mysensors.org/topic/6952/veml6070-and-veml6075-uv-sensors
//https://os.mbed.com/teams/MSS/code/VEML6075/file/9518befb9fd3/VEML6075.cpp
//https://os.mbed.com/teams/MSS/code/VEML6075/file/9518befb9fd3/VEML6075.h
//https://forum.mysensors.org/topic/6952/veml6070-and-veml6075-uv-sensors/25

//A lire fingerprint GT-511 sur mbed
//https://os.mbed.com/users/beanmachine44/notebook/fingerprint-scanner1/

//XBee
//center frequency=2.405+(CH-11d)x5MHz
//Ici : canal C -> f=2.405+(12-11)*5MHz -> 2.41GHz

//HC-12
////Configuration mode 441.4MHz -> "AT+C021"

//Tutoriel Nextion en Allemand :
//https://www.boecker-systemelektronik.de/epages/63381271.sf/de_DE/?ObjectPath=/Shops/63381271/Categories/Tutorials/Nextion_Tutorials
//Tutoriel de début
//https://www.boecker-systemelektronik.de/epages/63381271.sf/de_DE/?ObjectPath=/Shops/63381271/Categories/Tutorials/%22Grafische+Oberfl%C3%A4chen+f%C3%BCr+Mikrocontroller-Anwendungen%22

//Tutoriel Russe sur LSM6DS33
//https://www.youtube.com/watch?v=UzXH_3b_Cis
//Cours
//http://narodstream.ru/stm-urok-47-podklyuchaem-giroskop-lsm6ds3-chast-2/

//A lire LSM6DS33
//https://electronics.stackexchange.com/questions/358713/interrupt-and-fifo-settings-in-lsm6ds3
//https://community.st.com/thread/47814-interrupt-and-fifo-settings-in-lsm6ds3

#include "mbed.h"
//#include "USBSerial.h"
#include "SDFileSystem.h"
//-----------------------------------------------------------------
//1 wire
//-----------------------------------------------------------------
//#include "OneWire.h"
#define MAX_SENSORS	10

//-----------------------------------------------------------------
//Arret programmble du ARM M4
//-----------------------------------------------------------------
#include "WakeUp.h"

//-----------------------------------------------------------------
//Bus CAN à partir PJRC.COM
//-----------------------------------------------------------------






// include to check/display clock rates
#include "clk_freqs.h"
//Utilisation avec printf et USBSerial
//PC.printf("\r\n core %d",SystemCoreClock);
//PC.printf("\n Bus  %d",bus_frequency());
//PC.printf("\n Osc  %d",extosc_frequency());
//-----------------------------------------------------------------
//1 wire
//-----------------------------------------------------------------
//using namespace OneWire;
//using namespace RomCommands;
//broche pour signal DQ (Input/Output)
#define DS18B20_PIN	PTC4



//Definition de la durée entre chaque lecture
#define DUREE_ENTRE_IT	30

//Definition pour I2C
#define I2C_ACK			1
#define I2C_NACK		0
#define I2C_TIMEOUT	2

//Adresse du capteur I2C
//Ici 8bits car attente de 8bits pour l'adresse en I2C
#define ADDR_LUX 0b01000110
//Resolution de 0.5lx et attente de 120ms au minimum
#define Resolution_05LUX 0b00100001

//-----------------------------------------------------------------------
//Memoire EEPROM 24LC256
//-----------------------------------------------------------------------

//Adresse EEPROM : 1010 A2(=0) A1(=0) A0(=0) R/W(0)
#define ADRESSE_SELECTION_EEPROM        0b1010000<<1 //0x50
//Taille buffer ECRITURE (page write) 24LC256 : 64 octets
#define TAILLE_BUFFER        64
//Consigne pour LIRE et ECRIRE dans EEPROM 24LC256
#define LIRE 			ADRESSE_SELECTION_EEPROM
#define ECRIRE 		ADRESSE_SELECTION_EEPROM|1
//Consigne pour 1ere adresse interne (15 bits) EEPROM 24LC256
#define VAL_ADRESSE_INTERNE 0x0000
//Masque adresse interne (15 bits stockés dans int) EEPROM 24LC256
#define MASK_ADRESSE_INTERNE 0x7FFF
//Adresse EEPROM : 1010 A2(=0) A1(=0) A0(=0) R/W(0)

//-----------------------------------------------------------------------
//Capteur I2C TC74A0
//-----------------------------------------------------------------------

//Adresse fixe de selection du TC74. Cf. page 6
//#define ADRESSE_SELECTION_TC74        0b10011010
//Adresse fixe de sélection du TC74. Cf. carte STI2D ou BTS, étiquette TC74A0
#define ADRESSE_SELECTION_TC74        0b1001000<<1
//Registres de configuration du TC74
//TC74_TEMP, 8 bits en complément à 2 stockant la température (lecture seul)
#define TC74_TEMP 0x00
//TC74_CONFIG, 8 bits pour configurer le TC74 (lecture et écriture)
//b7: standby (0: normal; 1: standby) b6: data ready (1: ready, 0 not ready)
//b5...b0 : toujours à 0 lors d'une lecture
#define TC74_CONFIG 0x01

//-----------------------------------------------------------------------
//Capteur humidité HH10D - la sortie est une frequence. 
//-----------------------------------------------------------------------

//Gestion du HH10D - Selection nombre de bits - Adresse
#define ADRESSE_SELECTION_HH10D 0b1010001<<1 //0x51
//Valeur de l'adresse pour SENS (10 pour MSB et 11 pour LSB)
#define ADRESSE_SENS_HH10D        10
//Valeur de l'adresse pour OFFSET (12 pour le MSb et 13 pour le LSB)
#define ADRESSE_OFFSET_HH10D        12
//Duree de mesure de la période 
#define DUREE_MESURE_T_HH10D	1.0

//-----------------------------------------------------------------------
//Capteur humidité AM2322 - I2C
//-----------------------------------------------------------------------
//Gestion du HH10D - Selection nombre de bits - Adresse
#define ADRESSE_SELECTION_AM2322 0x5C<<1 //0x5C sur 7 bits

//-----------------------------------------------------------------------
//Emetteur - Recepteur - 433 - HC-12
//-----------------------------------------------------------------------
//Gestion des commandes AT
#define HC12_END_CMD ("\r")
#define HC12_CR (0x0D)
#define HC12_RESP_OK ("OK")
#define HC12_AT	"AT"
#define HC12_ERROR	"ERROR"
//Taille maximale de 60 octets en mode FU4 pour envoyer des données
//Ici buffer pour la configuration : min de 8 (AT+Bxxxx) et en plus CR (HC12_END_CMD) en fin de commande
//41 octets au minimum pour recevoir la reponse à AT+RX
#define HC12_TAILLE_BUFFER 60	
#define HC12_TIMEOUT	1
//FU4 -> AT+FU4
#define HC12_FU4 "AT+FU4"
#define HC12_FU4_Rep "OK+FU4"
//100mW -> AT+P8
#define HC12_100mW "AT+P8"
#define HC12_100mW_Rep "OK+P8"
//433.4MHz -> AT+C001
#define HC12_CANAL1 "AT+C001"
#define HC12_CANAL1_Rep "OK+C001"
//441.4MHz -> AT+C021
#define HC12_CANAL21 "AT+C021"
#define HC12_CANAL21_Rep "OK+C021"
//1200bauds -> AT+B1200
#define HC12_B1200 "AT+B1200"
#define HC12_B1200_Rep "OK+B1200"
//Verification Configuration
#define HC12_Config "AT+RX"
//Mode SLEEP
#define HC12_SLEEP "AT+SLEEP"
#define HC12_SLEEP_Rep "OK+SLEEP"
//Mode DEFAULT - Reset Factory - 9600bit/s - 8bits - pas parité - 1 bit stop - communication canal 001 (433.4MHz) - Power 20dBm - FU3 
#define HC12_DEFAULT "AT+DEFAULT"
#define HC12_DEFAULT_Rep "OK+DEFAULT"

//-----------------------------------------------------------------------
//Commande servo-moteur : par PWM
//-----------------------------------------------------------------------
//Test pour OPTIMA
//PwmOut PWM_HB(PTC1);
//PwmOut PWM_DG(PTD6);

//-----------------------------------------------------------------------
//Emetteur - Recepteur - XBee
//-----------------------------------------------------------------------
//Gestion des commandes AT
#define XBee_Mode_AT "+++"
#define XBee_Mode_AT_RESP	"OK\r"
#define XBee_Quitte_Mode_AT	"ATCN\r"
#define XBee_Quitte_Mode_AT_RESP	"OK\r"
#define XBee_No_Command_AT	" "
#define XBee_CR "\r"
#define XBee_RESP_OK "OK\r"
#define XBee_ERROR	"ERROR"
#define XBee_TAILLE_BUFFER 60	
#define XBee_TIMEOUT	2


#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE		0
#endif

#ifndef LED1_OFF
#define LED1_OFF		0
#endif

#ifndef LED1_ON
#define LED1_ON		1
#endif

#define uSD_OFF 0
#define uSD_ON 1

//-----------------------------------------------------------------------
//Utilisation de l'horloge RTC
//-----------------------------------------------------------------------
//Gestion de la RTC
#define DIMANCHE 0
#define LUNDI 1
#define MARDI 2
#define MERCREDI 3
#define JEUDI 4
#define VENDREDI 5
#define SAMEDI 6

#define JANVIER 1
#define FEVRIER 2
#define MARS 3
#define AVRIL 4
#define MAI 5
#define JUIN 6
#define JUILLET 7
#define AOUT 8
#define SEPTEMBRE 9
#define OCTOBRE 10
#define NOVEMBRE 11
#define DECEMBRE 12

#define ANNEE 2017
#define JOUR_SEMAINE_1_31	8
#define MOIS_ANNEE OCTOBRE

/////////////////////////////////////////////////////////////////////////////
// Gestion du 433MHz en Manchester pour OSV3
/////////////////////////////////////////////////////////////////////////////
#define OSV3_CRC8_M433_GP	0x107	//x^8+x^2+x+1
#define OSV3_CRC8_M433_DI	0x07
//Taille du tableau pour les données du capteur 
//Le capteur est temperature en °C et HR en % -> protocole Oregon Scientific V3 : THGR810
#define OSV3_TAILLE_DATA_CAPTEUR	13
//Possibilité de definir un canal de 1 à 15 : Oregon de 1 à 3
#define OSV3_CANAL_TEMP_HR	7
#define OSV3_CANAL_HAUTEUR_EAU 3
#define DUREE_ENTRE_CHAQUE_MESURE_OSV3	9

/////////////////////////////////////////////////////////////////////////////
// Gestion du VEML6075 pour UVA - UVB et UVC en I2C0
/////////////////////////////////////////////////////////////////////////////
#define ADDRESSE_VEML6075 0x10<<1
#define VEML6075_DEVID 0x26
/* VEML6075 SLAVE ADDRESS AND FUNCTION DESCRIPTION */
#define VEML6075_REG_UV_CONF  0x00 			// Configuration register (options below)
#define VEML6075_REG_Reserved01  0x01
#define VEML6075_REG_Reserved02  0x02
#define VEML6075_REG_Reserved03  0x03
#define VEML6075_REG_Reserved04  0x04
#define VEML6075_REG_Reserved05  0x05
#define VEML6075_REG_Reserved06  0x06
#define VEML6075_REG_UVA_Data    0x07		// UVA register
#define VEML6075_REG_UVD_Data    0x08 	// Dark current register
#define VEML6075_REG_UVB_Data    0x09 	// UVB register
#define VEML6075_REG_UVCOMP1_Data  0x0A // Visible compensation register
#define VEML6075_REG_UVCOMP2_Data  0x0B // IR compensation register
#define VEML6075_REG_DEVID  0x0C 				// Device ID register

#define VEML6075_CONF_IT_50MS    0x00 // Integration time = 50ms (default)
#define VEML6075_CONF_IT_100MS   0x10 // Integration time = 100ms
#define VEML6075_CONF_IT_200MS   0x20 // Integration time = 200ms
#define VEML6075_CONF_IT_400MS   0x30 // Integration time = 400ms
#define VEML6075_CONF_IT_800MS   0x40 // Integration time = 800ms
#define VEML6075_CONF_IT_MASK    0x8F // Mask off other config bits

#define VEML6075_CONF_HD_NORM    0x00 // Normal dynamic seetting (default)
#define VEML6075_CONF_HD_HIGH    0x08 // High dynamic seetting

#define VEML6075_CONF_TRIG       0x04 // Trigger measurement, clears by itself

#define VEML6075_CONF_AF_OFF     0x00 // Active force mode disabled (default)
#define VEML6075_CONF_AF_ON      0x02 // Active force mode enabled (?)

#define VEML6075_CONF_SD_OFF     0x00 // Power up
#define VEML6075_CONF_SD_ON    0x01 // Power down
 
// Following magic numbers are from 
// VISHAY veml6075 Application Note 84339
// Page 6 
#define VEML6075_UVA_A_COEF  (2.22)
#define VEML6075_UVA_B_COEF  (1.33)
#define VEML6075_UVB_C_COEF  (2.95)
#define VEML6075_UVB_D_COEF  (1.74)
#define VEML6075_UVA_sensitivity (0.93)
#define VEML6075_UVA_CIE_sensitivity (0.093)
#define VEML6075_UVB_sensitivity (2.1)
#define VEML6075_UVB_CIE_sensitivity (0.21)
// from page 15
#define VEML6075_UVA_RESPONSIVITY (0.001461)
#define VEML6075_UVB_RESPONSIVITY (0.002591)

//https://github.com/Jan--Henrik/VEML6075/blob/master/VEML6075.h

// To calculate the UV Index, a bunch of empirical/magical coefficients need to
// be applied to UVA and UVB readings to get a proper composite index value.
// Seems pretty hand wavey, though not nearly as annoying as the dark current
// not being subtracted out by default.

#define VEML6075_UVI_UVA_VIS_COEFF (3.33)
#define VEML6075_UVI_UVA_IR_COEFF  (2.5)
#define VEML6075_UVI_UVB_VIS_COEFF (3.66)
#define VEML6075_UVI_UVB_IR_COEFF  (2.75)

// Once the above offsets and crunching is done, there's a last weighting
// function to convert the ADC counts into the UV index values. This handles
// both the conversion into irradiance (W/m^2) and the skin erythema weighting
// by wavelength--UVB is way more dangerous than UVA, due to shorter
// wavelengths and thus more energy per photon. These values convert the compensated values 

#define VEML6075_UVI_UVA_RESPONSE (1.0 / 909.0)
#define VEML6075_UVI_UVB_RESPONSE (1.0 / 800.0)

/////////////////////////////////////////////////////////////////////////////
// Gestion du LCD NEXTION par IT avec buffer en emission et reception.
/////////////////////////////////////////////////////////////////////////////
//Taille maximale de 85 octets
#define MAX_DATA_NEXTION 85

 
//Virtual serial port over USB
//USBSerial PC;

//FTDI sur UART0 -> taille buffer augmente avec la vitesse ici 5 ou 6 pour 921600 baud
//					  TX		RX	
//Serial FTDI(PTB17,PTB16);

//FTDI sur UART2 -> taille buffer augmente avec la vitesse ici 5 ou 6 pour 921600 baud
//					TX		RX	
//Serial FTDI(PTD3,PTD2);

//FTDI sur UART1 -> taille buffer augmente avec la vitesse ici 5 ou 6 pour 921600 baud
//					TX		RX	
//Serial FTDI(PTC4,PTC3);

//FTDI sur HMI -> taille buffer augmente avec la vitesse ici 5 ou 6 pour 921600 baud
//					  TX		RX	
Serial FTDI(PTD3,PTD2);

/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion module XBee par commande AT
/////////////////////////////////////////////////////////////////////////////
Serial XBee_SX(PTB17,PTB16);

/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion de l'addicheur LCD Nextion - commun avec XBee
/////////////////////////////////////////////////////////////////////////////
//Serial LCD_HMI(PTB17,PTB16);

/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////
DigitalOut CS_HC12(PTB19,1);
//HC12-433
Serial HC_12(PTB17,PTB16);

//Commande de la LED1 en sortie
//DigitalOut LED1_ON_Off(PTC5,LED1_OFF);
//Commande alimentation carte SD
DigitalOut COM_SD_POWER(PTC9,uSD_OFF);
//Entree pour le BP sur PTA4
DigitalIn BP(PTA4,PullUp);

//Entrée analogique pour le CAN
AnalogIn AIN(PTB0);

//Utilisation d'un Ticker pour changer periodiquement l'etat de la LED et lire la luminosité en I2C
//Suppression pour deepsleep
Ticker Ma_Duree;

//Configuration du bus I2C sur les broches : SCL0 et SDA0
//Configuration I2C
//					 SDA0  SCL0
I2C Capt_I2C0(PTB3,PTB2);

//Configuration du bus I2C sur les broches : SCL1 et SDA1
//Configuration I2C
//					 SDA1  SCL1
I2C Capt_I2C1(PTC11,PTC10);

//Mise en service du CNA 12bits.
AnalogOut DAC_12bits(DAC0_OUT);

/////////////////////////////////////////////////////////////////////////////
// Declaration pour commander la carte uSD
/////////////////////////////////////////////////////////////////////////////

//Déclaration de l'objet SD CARD
// SDFileSystem(PinName mosi, PinName miso, PinName sclk, PinName cs, const char* name);
//								MOSI	MISO	SCLK	CS	name
//SDFileSystem sd(PTC6,PTC7,PTD1,PTC4,"sd",NC,SDFileSystem::SWITCH_NONE, 100000);
//SDFileSystem sd(PTC6,PTC7,PTD1,PTC4,"sd");
//Configuration carte uSD carte audio TEENSY 3.2
//SDFileSystem sd(PTC6,PTC7,PTD1,PTC5,"sd");
//SDFileSystem::CardType TypeuSD;
typedef struct {
								 time_t HeureAcqui;
								 float Temperature;
							 } T_DATA_SD;

/////////////////////////////////////////////////////////////////////////////
// Declaration objet pour SPI Optima
/////////////////////////////////////////////////////////////////////////////
							 
//Test pour LSM6DS33							 

/*							 
//Liste des signaux :
//CSLCD_3_3 sur PTC0 -> sortie
DigitalOut CSLCD(PTC0,1); 
//CSAFCL_3_3 sur PTB18 -> sortie
DigitalOut CSAFCL(PTB18,1); 			
//RESET_3_3 sur PTB1 -> sortie
DigitalOut RESET(PTB1,1); 					 
//SCK_3_3 sur PTD1 -> sortie
//MOSi, MISO, SCK
SPI Optima(PTC6, PTC7, PTD1);		 
//MOSI_3_3 sur PTC6 -> sortie
//MISO_3_3 sur PTC7 -> entrée
						 
//A0AFF_3_3 sur PTA5 -> sortie
DigitalOut A0AFF(PTA5,1); 					 
//TON/OFF_3_3 sur PTC3 -> entrée
DigitalIn TON_OFF(PTC3, PullNone);
*/

/////////////////////////////////////////////////////////////////////////////
// Declaration objet pour 1 wire
/////////////////////////////////////////////////////////////////////////////
DigitalInOut Capt_1_Wire(DS18B20_PIN,PIN_INPUT,PullNone,1);
//Mesure du temps de conversion du capteur DS18B20
Timer t;

/////////////////////////////////////////////////////////////////////////////
// Objets pour l'emission de la data en 433.92MHz pour OSV3
/////////////////////////////////////////////////////////////////////////////
DigitalOut POWER_433(PTD7,0);
DigitalOut Data_433(PTC2,0);							 
Timer TimeManchester;				

//Test transfert en binaire par XBee ou HC-12
//Par un struct dans un union
enum list_OSV3 { THGR810, UVN800, BTHR968, THN132N};
							 
typedef struct {
								 time_t HeureAcqui;	//4 octets
								 float Temperature;	//4 octets
								 float HR;					//4 octets
								 float UV;					//4 octets
								 float UVA;					//4 octets
								 float UVB;					//4 octets
								 float Pression;		//4 octets
								 int numero;				//4 octets
								 enum list_OSV3 type_capteur; //4 octets
								 
							 } T_DB; //sizeof donne 36 octets

typedef union {
								T_DB Data_Capteur;
								char Tab_TU[36]; //meme taille que T_DB et au même emplacement mémoire.
	
							} TU_DB;

//Trame transmise pour mettre à l'heure en HC-12
//Transmission de : time_t (4 octets pour un int) et TACQ en s (4 octets pour un int)
							

							
							
//Trame pour le pont :
//
/*
//Transmission des capteurs
typedef struct {
								 time_t HeureAcqui;	//4 octets
								 float Force;				//4 octets
								 float AX;					//4 octets
								 float AY;					//4 octets
								 float AZ;					//4 octets
								 bool Test;					//1 octet
								 int numero;				//4 octets
								 enum list_Type type_capteur; //4 octets
								 
							 } T_PDB; //sizeof donne 29 octets

//Transmission de TX/RX
typedef struct {
								 time_t HeureAcqui;	//4 octets
								 float Force;				//4 octets
								 float AX;					//4 octets
								 float AY;					//4 octets
								 float AZ;					//4 octets
								 bool Test;					//1 octet
								 int TACQ;					//4 octets
								 int numero;				//4 octets
								 enum list_Type type_capteur; //4 octets
								 
							 } T_PDB; //sizeof donne 33 octets			
							 
//Trame serrure bio
typedef struct {
								 time_t HeureAcqui;	//4 octets
								 char nomTab[20];		//20 octets
								 
								 enum list_Type type_Auto_Refu; //4 octets
								 
							 } T_PDB; //sizeof donne 28 octets		

							
							
*/



//Prototype des fonctions
void IT_LED(void);

/////////////////////////////////////////////////////////////////////////////
// Variables globales
/////////////////////////////////////////////////////////////////////////////

volatile bool FLAG=true;


//Utilisation d'une broche en entrée pour compter les fronts d'horloge en sortie du 555.
//Fout du HH10D
//Sur broche PTD0(D2) -> allumage de la LED (PTC5)
//Ici sur un front montant (importe peu le front)
InterruptIn IT_Fout_HH10D(PTD0);
//TimeOut pour une durée de 1s
Timeout Mesure_T_HH10D_1s;
//Compteur IT du HH10D
volatile int CPT_IT_HH10D=0,CPT_IT_HH10D_Res;
volatile bool FLAG_HH10D=true;

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du HH10D en I2C
/////////////////////////////////////////////////////////////////////////////

void i2c_lecture_OFFSET_SENS_HH10D(int ADRESSE_HH10D,int *SENS, int *OFFSET, bool *TimeOut_Ended);

//IT sur front montant de HH10D
void IT_HH10D(void);
void IT_HH10D_CPT(void);

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion d'une EEPROM en I2C
/////////////////////////////////////////////////////////////////////////////
char i2c_lecture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE, bool *TimeOut_Ended);
void i2c_lecture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE,  int NOMBRE_A_LIRE, char *dest, bool *TimeOut_Ended);
void i2c_ecriture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char DATA, bool *TimeOut_Ended);
void i2c_ecriture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char *t, bool *TimeOut_Ended);

////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du capteur de temperature TC74 en I2C
/////////////////////////////////////////////////////////////////////////////

signed char i2c_lecture_temp_tc74(int adresse_TC74, bool *TimeOut_Ended);
void i2c_init_tc_74(int adresse_TC74, bool *TimeOut_Ended);
bool i2c_test_ready_tc_74(int adresse_TC74, bool *TimeOut_Ended);
void i2c_standby_tc_74(int adresse_TC74, bool *TimeOut_Ended);

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////
bool sendATcommand(char* ATcommand, char* expected_answer, float timeout);
void Enter_CMD_mode(void);//NL0 pendant >40ms
void Leave_CMD_mode(void);//NL1 et attente MAJ pendant >80ms avant retour mode transparent

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du module XBee par commande AT
/////////////////////////////////////////////////////////////////////////////
bool Entree_Mode_ATcommand_XBee(char *ATcommand, char *expected_answer, float timeout);
bool sendATcommand_XBee(char *ATcommand, char *Param, char *expected_answer, float timeout);
bool Lecture_ResultATcommand_XBee(char *ATcommand, char *Param, char *answer_XBee, float timeout);
bool Lecture_ATND_Nom_command_XBee(char *ATcommand, char *Param, char *answer_XBee, float timeout);
void Extraire_ATND(char *data, unsigned int *MY, unsigned int *SH, unsigned int *SL, unsigned int *DB, char *NI);

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du capteur 1 Wire DS18B20
/////////////////////////////////////////////////////////////////////////////
bool OneWireReset(void);
void OneWireOutByte(unsigned char d);
void OneWireOut_1_Bit(unsigned char d);
unsigned char OneWireInByte(void);
unsigned char OneWireIn_1_Bit(void) ;
void OneWireIn_N_Bytes(unsigned char *buf, unsigned short N);
void OneWireOut_N_Byte(const unsigned char *buf, unsigned int N);
void OneWire_SelectRom(const unsigned char rom[8]);
void OneWire_SkipRom(void);
void OneWire_ResetSearch(uint8_t *OW_LastDiscrepancy, uint8_t *OW_LastDevice, uint8_t *OW_LastFamilyDiscrepancy, unsigned char *ROM_NO);
void OneWire_TargetSearch(uint8_t *OW_LastDiscrepancy, uint8_t *OW_LastDevice, uint8_t *OW_LastFamilyDiscrepancy, uint8_t Family_Code, unsigned char *ROM_NO);
bool OneWiresearch(uint8_t *OW_LastDiscrepancy, uint8_t *OW_LastDevice, uint8_t *OW_LastFamilyDiscrepancy, uint8_t *newAddr, bool search_mode /* = true */,unsigned char *ROM_NO);
unsigned char OneWireCRC8(const unsigned char *addr, unsigned char len);

int OneWireNext(unsigned char *ROMID,unsigned char *OW_LastDiscrepancy, bool *OW_LastDevice, unsigned char *OW_LastFamilyDiscrepancy);
int OneWireFirst(uint8_t *ROMID, uint8_t *OW_LastDiscrepancy, bool *OW_LastDevice, uint8_t *OW_LastFamilyDiscrepancy);
int OneWireReadTemperature(uint8_t *ROMID, int *result);
void OneWireConvertAll(bool wait);
void OneWireSendCmd(uint8_t *ROMID, uint8_t cmd);

//Fonctions pour la recherche des esclaves
void FindDevices(void);
unsigned char First(void);
bool Next(void);
unsigned char ow_crc( unsigned char x);
float read_one_DS18B20(unsigned char index);

//Variables 1 WIRE pour fonction Find, Next et Search
//En variables globale pour tests
unsigned char ROM[8];
unsigned char lastDiscrep;
bool doneFlag;
unsigned char numROMs;
unsigned char dowcrc;
unsigned char FoundROM[MAX_SENSORS][8];

static const unsigned char dscrc_table[] = { 
0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
157,195, 33,127,252,162, 64, 30, 95, 1,227,189, 62, 96,130,220,
35,125,159,193, 66, 28,254,160,225,191, 93, 3,128,222, 60, 98,
190,224, 2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89, 7,
219,133,103, 57,186,228, 6, 88, 25, 71,165,251,120, 38,196,154,
101, 59,217,135, 4, 90,184,230,167,249, 27, 69,198,152,122, 36,
248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91, 5,231,185,
140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
202,148,118, 40,171,245, 23, 73, 8, 86,180,234,105, 55,213,139,
87, 9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};


/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du protocole OSV3
/////////////////////////////////////////////////////////////////////////////
void Teensy32_Back2Speed_After_PowerDown(void);
void OSV3_MANCHESTER_SEND_DATA(void);
unsigned char OSV3_CALC_CRC(bool *InitFait);
unsigned char OSV3_CALC_CHECKSUM(void);
void OSV3_CONSTRUIRE_TRAME(float Temp_f, int HumiHR);
unsigned char OSV3_ROLLING_CODE(void);
void OSV3_MANCHESTER_ENCODE(unsigned char Octet_Encode, bool Fin);
void OSV3_MANCHESTER_SEND_DATA(void);
void OSV3_INIT_CRC(bool *InitFait);

/////////////////////////////////////////////////////////////////////////////
// Gestion du 433MHz en Manchester pour OSV3-> variables globales
/////////////////////////////////////////////////////////////////////////////
//Tableau pour stocker tout le protole OSV3
unsigned char OSV3_TAB_DATA[OSV3_TAILLE_DATA_CAPTEUR];
unsigned char OSV3_TAB_CRC_INIT[256];
unsigned char VAL_ROLLING_CODE;
bool Fait_Init_TAB_CRC=false;
//Pour tester l'emission en Manchester et 433MHz
float VAL_TempC;
unsigned int VAL_HR;
bool Fait_ROOLING_CODE=false;
//Test pour transmettre en binaire :
T_DB TransferBinaireOSV3;
TU_DB UnionOSV3;

/////////////////////////////////////////////////////////////////////////////
// Gestion du VEML6075 pour UVA - UVB et UVC en I2C0
/////////////////////////////////////////////////////////////////////////////
int VEML6075_GetID(int ADDR_VEML6075, bool *TimeOut_Ended);
int VEML6075_GetRauUVABD(int ADDR_VEML6075, char SelectABD, bool *TimeOut_Ended);
void VEML6075_WriteConfig(int ADDR_VEML6075, char Config, bool *TimeOut_Ended);
float VEML6075_GetUVA(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uva, unsigned int raw_dark);
float VEML6075_GetUVB(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uvb, unsigned int raw_dark);
float VEML6075_GetUVIndex(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uva,unsigned int raw_uvb, unsigned int raw_dark);


/////////////////////////////////////////////////////////////////////////////
// Gestion du LCD NEXTION par IT et RS232
/////////////////////////////////////////////////////////////////////////////

void LCD_NEXTION_IT_RX(void);
void Copie_de_Tableau(volatile char *s, char *d);

//tableau pour stocker le message à emettre et pour stocker le message reçu
//Le buffer de réception et de transmission sont séparés
volatile char data_RX_LCD_Nextion[MAX_DATA_NEXTION];
				 char data_TX_LCD_Nextion[MAX_DATA_NEXTION];
				 char Copie_data_RX_LCD_Nextion[MAX_DATA_NEXTION];
volatile bool IT_RX_LCD_Nextion_OK=false;

//Variable dans la fonction IT -> ne fonctionne pas.
volatile unsigned int CPT_RX_LCD_Nextion=0;

/////////////////////////////////////////////////////////////////////////////
// Fonction pour gestion du LSM6DS33TR
/////////////////////////////////////////////////////////////////////////////
unsigned char LSM_spi_read(unsigned char target);
short LSM_spi_read_16bits(unsigned char target);
void LSM_spi_write(unsigned char target, unsigned char data2write);
void LSM_spi_init(void);
float CalcAcc(int acc);
float CalcGyro(int gyro);
void LSM_ISR_INT1(void);
void LSM_ISR_INT2(void);
void LSM_Vidage_FIFO(void);
unsigned short LSM_FIFO_Status(void);
short LSM_Read_FIFO(void);
unsigned short LSM_FIFO_Pattern(void);
void LSM_bypass_FIFO(void);


/////////////////////////////////////////////////////////////////////////////
// SPI pour LSM6DS33TR
/////////////////////////////////////////////////////////////////////////////

//SPI pour le LSM6DS33TR
//       mosi miso sclk
SPI LSM(PTC6, PTC7, PTD1);

//CS pour LSM6DS33TR
DigitalOut LSM_CS(PTC5,1);

//INT2 et INT1 en entrée interruption
//Le flag FIFO_STATUS2 est redirigé vers INT1 par INT1_CTRL
//INT1 sur PTD0
InterruptIn LSM_INT1(PTD0);
//INT2 sur PTB19
InterruptIn LSM_INT2(PTB19);

/////////////////////////////////////////////////////////////////////////////
// #define pour LSM6DS33TR
/////////////////////////////////////////////////////////////////////////////
#define LSM_READ		0x80
#define LSM_WRITE 	0x7F

#define LSM_accelX	0x28
#define LSM_accelY	0x2A
#define LSM_accelZ	0x2C

#define LSM_gyroX		0x22
#define LSM_gyroY		0x24
#define LSM_gyroZ		0x26

#define LSM_FIFO_Data_L 0x3E
#define LSM_FIFO_Data_H 0x3F

#define LSM_FIFO_STATUS1	0x3A
#define LSM_FIFO_STATUS2	0x3B

#define LSM_FIFO_STATUS3	0x3C
#define LSM_FIFO_STATUS4	0x3D

//AN page 9
#define WHO_I_AM		0b01101001
//Taille FIFO -> 8ko soit 4096 mots de 16bits - page 41 de la doc.
//#define TAILLE_FIFO	4095
#define TAILLE_FIFO	4040
//#define TAILLE_FIFO	210
//#define TAILLE_FIFO	3990
//#define TAILLE_FIFO	1000
//Taille max de la FIFO pour tableaux
#define TAILLE_FIFO_MAX 4095

/////////////////////////////////////////////////////////////////////////////
// Variable globale pour LSM6DS33TR
/////////////////////////////////////////////////////////////////////////////
volatile bool Flag_Fifo=true;
volatile bool Flag_Wake=true;

/////////////////////////////////////////////////////////////////////////////
// I2C pour HDC1080
/////////////////////////////////////////////////////////////////////////////
//Adress HDC1080 sur 8bits (Cf. page 10 de la doc).
#define I2C_HDC1080		0b1000000 <<1

#define Temp_14bits		6.35
#define Temp_11bits		3.35

#define RH_14bits			6.50
#define RH_11bits			3.85
#define RH_8bits			2.50

#define Reg_Temp			0x00
#define Reg_HR				0x01
#define Reg_Config		0x02

#define Reg_ID_HBytes			0xFB
#define Reg_ID_MBytes			0xFC
#define Reg_ID_LBytes			0xFD

#define Reg_ID_Manu				0xFE
#define Reg_ID_Device			0xFF

#define VAL_ID_Manu			0x5449
#define VAL_ID_Device		0x1050

#define Precision_Temp_demandee 14
//#define Precision_Temp_demandee 11


#define Precision_HR_demandee 14
//#define Precision_HR_demande 11
//#define Precision_HR_demandee 8

/////////////////////////////////////////////////////////////////////////////
// Variable globale pour I2C - Gestion du timeout
/////////////////////////////////////////////////////////////////////////////
bool TimeOut_Ended;


/////////////////////////////////////////////////////////////////////////////
// Fonction pour gestion du HDC1080
/////////////////////////////////////////////////////////////////////////////
unsigned short HDC1080_Read_Temp(int addr, unsigned char Precision, bool *TimeOut_Ended);
unsigned short HDC1080_Read_HR(int addr, unsigned char Precision, bool *TimeOut_Ended);
bool HDC1080_Read_Temp_HR(int addr, unsigned char PrecisionTemp, unsigned char PrecisionHR, unsigned short *Temp, unsigned short *HR, bool *TimeOut_Ended);
bool HDC1080_Test_ID(int addr, bool *TimeOut_Ended);
float Calc_Temp(unsigned short Temp);
float Calc_HR(unsigned short HR);

//Declaration des objets pour le HDC1080
//           SDA SCL
I2C HDC1080(PTC11,PTC10);
//Vitesse de l'IC 400kHz max (Cf. page 6 de la doc et vitesse minimale 10kHz).


/////////////////////////////////////////////////////////////////////////////
// Fonction pour gestion du BMP280
/////////////////////////////////////////////////////////////////////////////

//typedef pour les parametres du BMP280
typedef struct
	{
		unsigned short    dig_T1;
    short     				dig_T2, dig_T3;
    unsigned short    dig_P1;
    short     				dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	} T_BMP280;



//Autre solution de codage - BMP280
int BMP280_init(int addr, T_BMP280 *BMP280_Data , char ctrl_meas , char config , bool *TimeOut_Ended);
int BMP280_readData(int addr, T_BMP280 BMP280_Data , float *tempC, float *pressPa, bool *TimeOut_Ended);
void BMP_280_write(int addr, char reg, char ctrl, bool *TimeOut_Ended);
void BMP280_read(int addr, char reg, char *data, int length, bool *TimeOut_Ended);
bool BMP280_Test_Presence(int addr, bool *TimeOut_Ended);
bool BMP280_Fin_Mesure(int addr, bool *TimeOut_Ended);
float BMP280_readAltitude(float seaLevelhPa, float PressPa);
bool BMP280_Soft_Reset(int addr, bool *TimeOut_Ended);
	
//Variable globale pour le BMP280
T_BMP280 BMP280_Data;
	
//https://developer-sjc-indigo-border.mbed.org/teams/MtM/code/BMP280/docs/tip/BMP280_8h_source.html
//https://learn.adafruit.com/pages/6077/elements/2957962/download
//CODE BOSCH
//https://github.com/BoschSensortec/BMP280_driver/blob/master/bmp280_defs.h
	
//Adresse I2C du BMP280
#define ADRESSE_SELECTION_BMP280 0x76<<1 
	
// Register define 
#define BMP280_REG_CALIBRATION  0x88    // calibration register start
#define BMP280_REG_ID           0xD0    // read out is 0x58
#define BMP280_CHIPID           0x58
#define BMP280_REG_RESET        0xE0    // write value 0xB6 will process power-on-reset other value is not work
#define BMP280_REG_STATUS       0xF3    // indicate status of the device
#define BMP280_REG_CTRL_MEAS    0xF4    // config options of the device
#define BMP280_REG_CONFIG       0xF5    // set rate, filted and interface options of device
#define BMP280_REG_PRESS        0xF7    // pressure measurement out data 0xF7~0xF9
#define BMP280_REG_TMEP         0xFA    // temperature measurment out data 0xFA~0xFC
 


//Valeur pour filter
#define BMP280_FILTER_OFF       0b000    	// filter off
#define BMP280_FILTER_2       	0b001    	// filter 2
#define BMP280_FILTER_5       	0b010   	// filter 5
#define BMP280_FILTER_11       	0b011   	// filter 11
#define BMP280_FILTER_22        0b100    	// filter 22


#define BMP280_OSSR_SKIP        0x00    // Skipped output set to 0x80000
#define BMP280_OSSR_OV1         0x01    // oversampling x1
#define BMP280_OSSR_OV2         0x02    // oversampling x2
#define BMP280_OSSR_OV4         0x03    // oversampling x4
#define BMP280_OSSR_OV8         0x04    // oversampling x8
#define BMP280_OSSR_OV16        0x05    // oversampling x16
 
// wait to read out time
// T_SB
#define BMP280_T_SB0            0x00    // 0.5ms
#define BMP280_T_SB62           0x01    // 62.5ms
#define BMP280_T_SB125          0x02    // 125ms
#define BMP280_T_SB250          0x03    // 250ms
#define BMP280_T_SB500          0x04    // 500ms
#define BMP280_T_SB1000         0x05    // 1000ms
#define BMP280_T_SB2000         0x06    // 2000ms
#define BMP280_T_SB4000         0x07    // 4000ms
 
// Power Mode
#define BMP280_POWER_SLEEP      0b00
#define BMP280_POWER_FORCE      0b01
#define BMP280_POWER_NORMAL     0b11

// OSSR Data soit pour osrs_p (3 bits) et osrs_t (3 bits)
//osrs_t :
//000 -> pas de mesure de T
//001 -> 16bits / 0.0050°C
//010 -> 17bits / 0.0025°C
//011 -> 18bits / 0.0012°C
//100 -> 19bits / 0.0005°C
//101 -> 20bits / 0.0003°C
#define OSRS_Pas_Mesure_T	0b000
#define OSRS_T_16bits			0b001
#define OSRS_T_17bits			0b010
#define OSRS_T_18bits			0b011
#define OSRS_T_19bits			0b100
#define OSRS_T_20bits			0b101

//osrs_p :
//000 -> pas de mesure de P
//001 -> 16bits / 2.62 Pa
//010 -> 17bits / 1.31 Pa
//011 -> 18bits / 0.66 Pa
//100 -> 19bits / 0.33 Pa
//101 -> 20bits / 0.16 Pa
#define OSRS_Pas_Mesure_P	0b000
#define OSRS_P_16bits			0b001
#define OSRS_P_17bits			0b010
#define OSRS_P_18bits			0b011
#define OSRS_P_19bits			0b100
#define OSRS_P_20bits			0b101

//Detail du registre ctrl_meas
// bit  7 6 5   4 3 2    1 0   
//      osrs_t  osrs_p   mode

//Detail du registre config
// bit  7 6 5   4 3 2    1 0   
//      t_sb    filter   X spi3w_en

//Configuration du BMP280
//mode FORCE
//Mesure de T et P avec maximum de precision : ULTRA HIGH RESOLUTION (table 13)
//T avec : oversamling x16 -> 20bits / 0.0003°C
//P avec : oversamling x16 -> 20bits / 0.16 hPa

#define CTRL_MEAS		(OSRS_T_20bits <<5) | (OSRS_P_20bits << 2) | (BMP280_POWER_FORCE)
#define CONFIG			(BMP280_OSSR_OV16 <<5) | (BMP280_FILTER_22 <<2) | (0b00)

#define SEALEVELHPA 1013.25

//Ce qui donne :
//#define CTRL_MEAS		0b101 101 01 soit 0xb5
//#define CONFIG			0b101 100 00 soit 0xb0

//---------------------------------------------------------------------------

int main(void) 
{
	
	//Vitesse FTDI
	//FTDI.baud(250000);
	FTDI.baud(921600);
	//FTDI.baud(512000);
	FTDI.format(8,SerialBase::None,1);
	
	//Initialisation de l'I2C - vitesse 400kHz
	Capt_I2C1.frequency(400000);
	
	//Affichage
	FTDI.printf("\r\n Test BMP280 - Presser une touche pour continuer.");
	int c=FTDI.getc();
	
	//declaration des variables
	float BMP280_Temp;
  float BMP280_Pressure;
	

	
	//Recherche des adresses des composants en I2C1 par un scan I2C
		FTDI.printf("\r\n\rDebut Scanner I2C1\r\n");
		
		for (int i=0 ; i<128 ; i++)
		{
			if(!Capt_I2C1.write(i<<1, NULL, 0)) FTDI.printf("%#x ACK \r\n",i);
		}
		
		FTDI.printf("Fin Scanner I2C1\r\n");
		
		FTDI.printf("Debut Soft Reset\r\n");
		BMP280_Soft_Reset(ADRESSE_SELECTION_BMP280, &TimeOut_Ended);
		FTDI.printf("Fin Soft Reset\r\n");
		
		//Test presense BMP280
		if(!BMP280_Test_Presence(ADRESSE_SELECTION_BMP280, &TimeOut_Ended)) FTDI.printf("Erreur BMP280 non detecte.");
		else 
			{	
				FTDI.printf("BMP280 detecte.");
	
				while(true)
				{
					
					//2eme solution
					FTDI.printf("\r\n\rInitialisation BMP280\r\n");
					FTDI.printf("\r\nCTRL_MEAS=%#x|CONFIG=%#x",CTRL_MEAS, CONFIG);
					BMP280_init(ADRESSE_SELECTION_BMP280, &BMP280_Data, CTRL_MEAS, CONFIG, &TimeOut_Ended);
					FTDI.printf("\r\n\rFin Initialisation BMP280\r\n");
					
					
					//Attente fin mesure
					while(!BMP280_Fin_Mesure(ADRESSE_SELECTION_BMP280, &TimeOut_Ended));
					
					//init(ADRESSE_SELECTION_BMP280, &BMP280_Data, 0x6F, 0x70);
					BMP280_readData(ADRESSE_SELECTION_BMP280, BMP280_Data, &BMP280_Temp, &BMP280_Pressure, &TimeOut_Ended);
					FTDI.printf("Temp: %4.4f °C, Pression: %4.2f hPa\r\n",BMP280_Temp, BMP280_Pressure/100.0);
					
					FTDI.printf("\r\nAltitude=%8.4fm",BMP280_readAltitude(SEALEVELHPA, BMP280_Pressure));
		
		
					wait(1);
				
			}
		
		
		}
	
	
	
	/*
	//Vitesse FTDI
	//FTDI.baud(250000);
	FTDI.baud(921600);
	//FTDI.baud(512000);
	FTDI.format(8,SerialBase::None,1);
	
	//Initialisation de l'I2C - vitesse 100kHz
	HDC1080.frequency(400000);
	
	//Affichage
	FTDI.printf("\r\n Test HDC1080 - Presser une touche pour continuer.");
	int c=FTDI.getc();
	
	//declaration des variables
	unsigned short Temp,HR;
	float myTemp,myHR;
	bool Fait;
	
	//Attention - attente de 15ms avant une demande d'acquisition apres PowerUp du HDC1080
	
	
	while(true)
	{
		FTDI.printf("\r\nHDC1080 detecte : %s\r\n",HDC1080_Test_ID(I2C_HDC1080, &TimeOut_Ended) ? "OUI" : "NON");
		//Acquisition de Temp
		Temp=HDC1080_Read_Temp(I2C_HDC1080, Precision_Temp_demandee, &TimeOut_Ended);
		
		if(TimeOut_Ended) FTDI.printf("\r\n Erreur timeout sur HDC1080_Read_Temp");
		else {
					if(Temp) FTDI.printf("\r\nTemp sur %dbits=%d",Precision_Temp_demandee,Temp);
					else FTDI.printf("\r\n Erreur lecture Temp HDC1080_Read_Temp");
				 }
		
		
		//Acquisition de HR
		HR=HDC1080_Read_HR(I2C_HDC1080, Precision_HR_demandee, &TimeOut_Ended);
		
		if(TimeOut_Ended) FTDI.printf("\r\n Erreur timeout sur HDC1080_Read_HR");
		else {
					if(HR) FTDI.printf("\r\nHR sur %dbits=%d",Precision_HR_demandee,HR);
					else FTDI.printf("\r\n Erreur lecture HR HDC1080_Read_HR");
				 }
		
		
		//Calcul de Temp et HR
		FTDI.printf("\r\nTemp=%3.1f°C",Calc_Temp(Temp));
		FTDI.printf("\r\nHR=%3.1f%%\r\n",Calc_HR(HR));
		
		//Acquisition de Temp et HR en même temps
		FTDI.printf("\r\nAcquisition sur %dbits de Temp et %dbits pour HR",Precision_Temp_demandee,Precision_HR_demandee);
		Fait=HDC1080_Read_Temp_HR(I2C_HDC1080, Precision_Temp_demandee, Precision_HR_demandee, &Temp, &HR, &TimeOut_Ended);
		
		if(TimeOut_Ended) FTDI.printf("\r\n Erreur timeout sur HDC1080_Read_Temp_HR");
		else {
					if(Fait) FTDI.printf("\r\nTemp=%3.1f°C| HR=%3.1f%%",Calc_Temp(Temp),Calc_HR(HR));
					else FTDI.printf("\r\n Erreur lecture HR et Temp dans HDC1080_Read_Temp_HR");
				 }
		
		wait(1);
	}
	
	*/
	
	/*
	
	//Affectation de la fonction à appeler toutes les DUREE_ENTRE_IT
	//Suppression pour deepsleep
	Ma_Duree.attach(&IT_LED,DUREE_ENTRE_IT);
	
	//Variables pour CAN 16bits
	float Res_CAN;
	unsigned short Res_us;
	
	//Variable pour HH10D
	int HH10D_Sens, HH10D_Offset;
	
	//Variable pour I2C
	bool I2C_Duree_Time_Out_Ecoulee=false;
	
	//Vitesse FTDI
	FTDI.baud(115200);
	FTDI.format(8,SerialBase::None,1);
	
	//Vitesse HC12
	//Configuration liaison serie
	//reglage usine : 9600 bauds
	HC_12.baud(9600);
	HC_12.format(8,SerialBase::None,1);
	
	//HC-12 en 433.4MHz
	char tab_HC_12_CMD_AT[HC12_TAILLE_BUFFER];
	char tab_HC_12_reponse_AT[HC12_TAILLE_BUFFER];
	
	//Variable configuration faite HC12
	bool HC12_CONFIG_DONE=false;
	
	//Vitesse LCD Nextion
	//Configuration liaison serie
	//reglage usine : 9600 bauds modifie en 115200 par commande entrée au clavier.
	//LCD_HMI.baud(115200);
	//LCD_HMI.format(8,SerialBase::None,1);
	
	//La reception est realisée par interruption -> traitement à realiser rapidement.
	//IT en reception par RS232 pour le LCD Nextion
	//LCD_HMI.attach(&LCD_NEXTION_IT_RX, Serial::RxIrq);
	
	
	
	//Vitesse XBee SX
	//Configuration liaison serie
	//reglage usine : 115200 bauds - chnagement par XCTU
	XBee_SX.baud(115200);
	XBee_SX.format(8,SerialBase::None,1);
	static char XBee_Reponse[XBee_TAILLE_BUFFER];
	unsigned int MY;
	unsigned int SH,SL;
	unsigned int DB;
	char NI[21]={'\0'};
	
	//Vitesse du bus I2C -> 100kbit/s
	Capt_I2C0.frequency(400000);
	Capt_I2C1.frequency(400000);
	
	//Tension en sortie du DAC de 0,22V
	FTDI.printf("\r\nDAC d.d.p de 0,22V");
	DAC_12bits.write_u16(273<<4);
	
	//1 wire broche associée à la DATA
	//D sur PTC4
	unsigned char HighByte, LowByte;
	//Autre solution pour stoker le resultat sur 16bits : int16_t Result;
	short Result; 
	Capt_1_Wire.output();
  Capt_1_Wire = 0;
	// Impose un NL1 sur la broche en la declarant en entrée par la presence de R de 4.7k.
  Capt_1_Wire.input(); 
	
	*/
	
	
	/*
	
	// Reset state 
  uint8_t OW_LastDiscrepancy;
  uint8_t OW_LastDevice;
  uint8_t OW_LastFamilyDiscrepancy;
	unsigned char ROM_NO[8];
	
	//Variables pour 1 Wire
	bool present;
	unsigned char data[12];
	unsigned char addr[8];
	
	//Variables 1 WIRE pour fonction Find, Next et Search
	//En variables globale pour tests
	//unsigned char ROM[8];
	//unsigned char lastDiscrep;
	//unsigned char doneFlag;
	//unsigned char numROMs;
	//unsigned char dowcrc;
	//unsigned char FoundROM[MAX_SENSORS][8];
	
	
	//Variables bpour la gestion de l'horloge RTC
	//Extraction HMS
	unsigned int HMS;
	//Gestion de l'heure associée à l'horloge RTC
	struct tm mytime;
	//Pré configuration de l'horloge -> Year Month Day
	mytime.tm_year=ANNEE-1900;
	mytime.tm_mon=MOIS_ANNEE-1;
	mytime.tm_mday=JOUR_SEMAINE_1_31;
	//reglage de heure, minute et seconde
	mytime.tm_hour=15;
	mytime.tm_min=50;
	mytime.tm_sec=30;
	//Variable pour obtenir l'heure
	time_t seconds;
	//Acquisition de l'heure sous forme d'une chaîne de caractéres HMS
	char Heure_UC[6];
	//Variable pour l'acquisition avec l'heure
	T_DATA_SD Acqui_Heure_SD;
	//Mise à l'heure de l'horloge RTC
	set_time(mktime(&mytime));
	//Compteur pour la position dans le fichier:
	int CPT=0;
	
	
	*/
	
//-----------------------------------------------------------------------
//Commande servo-moteur : par PWM
//-----------------------------------------------------------------------
	//Test pour OPTIMA
	//PWM_HB.period_ms(20);
	//int HB=1500,DG=1500;
	
/*
	//false, on ne traite pas le test du LCD.
	if(false)
	{
		
	//Affichage message pour LCD_HMI
	FTDI.printf("\r\nAttente reception message par TX0/RX0 du LCD HMI.\r\n");
	
	//Codage de la reception de l'afficheur LCD HMI
	//Affichage par scrutation puis
	//Par interruption remplissage d'un buffer
	int LCDi=69999;
	
		while(true)
		{
		
			FTDI.printf("\r\nEmission vers LCD de LCDi=%d\r\n",LCDi);
		
			//while(!LCD_HMI.writeable());
			//LCD_HMI.printf("page 0ÿÿÿ");
			//wait_ms(200);
			//while(!LCD_HMI.writeable());
			//LCD_HMI.printf("page 0ÿÿÿ");
			//while(!LCD_HMI.writeable());
			//LCD_HMI.printf("n0.val=%dÿÿÿ",LCDi++);
		
			//t2.txt=Page_Intro.va0.txt
			while(!LCD_HMI.writeable());
			LCD_HMI.printf("va0.txt=\"%d\"ÿÿÿ",LCDi++);
		
			wait(1);
		
			if(IT_RX_LCD_Nextion_OK) {
																//FTDI.printf("Reception du LCD TX=%s",data_RX_LCD_Nextion);
																for(int i=0 ; i<8 ; i++)
																	{
																		FTDI.printf("\r\nReception du LCD TX=%#x",(int)data_RX_LCD_Nextion[i]);
																		FTDI.printf("\r\nReception du LCD TX=%d",(int)data_RX_LCD_Nextion[i]);
																	}
																	
																Copie_de_Tableau(data_RX_LCD_Nextion, Copie_data_RX_LCD_Nextion);
																if(strcmp(Copie_data_RX_LCD_Nextion,"")==0) FTDI.printf("\r\nEgalite avec 65030FFFFFF");
																else FTDI.printf("\r\nDifferent de 65030FFFFFF");
																	
																CPT_RX_LCD_Nextion=0;
																IT_RX_LCD_Nextion_OK=false;
															}
		
		
		}
		
	}
	
*/
	
	/*
	//Gestion de l'affichage de la pompe OPTIMA
	//Configuration de la SPI
	
	
	Optima.format(8,0);
	Optima.frequency(1000000);
	
	CSLCD=0;
	CSAFCL=0;
	
	FTDI.printf("\r\n Reset Pompe OPTIMA");
	
	
	RESET=0;
	wait_ms(10);
	RESET=1;
	
	wait(1);
	//Variable pour changer l'etat de la LED_ALARME
	bool FlipFlop=true;
	
	while(true)
	{
		//FTDI.printf("\r\n CSAFCL=1");
		//Selection des 7 segemnts et DEL
		CSAFCL=1;
		//Dernier registre U12
		//NC NC NC QE Q/=LED_ALARME COL2 COL1 COL0
		
		if (FlipFlop) Optima.write(0b00001111);
		else Optima.write(0b00000111);
		FlipFlop=!FlipFlop;
		//FTDI.printf("\r\n FlipFlop=%d",FlipFlop);
		
		//SEG7 SEG6 SEG5 SEG4 SEG3 SEG2 SEG1 SEG0
		Optima.write(0b01101101);
		//DIG...premier registre de puissance
		//CD_BACK_LIGHT NC DIG5 DIG4 DIG3 DIG2 DIG1 DIG0
		int LectureSPI=Optima.write(0b10000001);
		
		//if(TON_OFF) FTDI.printf("\r\n Touche TON pressee");
		//else FTDI.printf("\r\n Touche TON relachee");
		
		//Fin selection 7 segments et DEL et actualisation de l'affichage
		//FTDI.printf("\r\n CSAFCL=0");
		CSAFCL=0;
		//FTDI.printf("\r\n Lecture clavier de : %#x",LectureSPI);
		
		wait_ms(200);
	}
	
	*/
	

/*
	while(true)
	{
		
		//Recherche des adresses des composants en I2C0 par un scan I2C
		FTDI.printf("\r\nDebut Scanner I2C0\r\n");
		
		for (int i=0 ; i<128 ; i++)
		{
			if(!Capt_I2C0.write(i<<1, NULL, 0)) FTDI.printf("%#x ACK \r\n",i);
		}
		
		FTDI.printf("Fin Scanner I2C0\r\n");
		
		//Recherche des adresses des composants en I2C1 par un scan I2C
		FTDI.printf("\r\n\rDebut Scanner I2C1\r\n");
		
		for (int i=0 ; i<128 ; i++)
		{
			if(!Capt_I2C1.write(i<<1, NULL, 0)) FTDI.printf("%#x ACK \r\n",i);
		}
		
		FTDI.printf("Fin Scanner I2C1\r\n");
		
		FTDI.printf("\r\nAttente de %ds",DUREE_ENTRE_IT);
		//Attente demande de mesure par Ticker
		//Suppression pour deepsleep
		while(FLAG);
		FLAG=true;
		
		//FTDI.printf("\r\n\n 1ere-Calibration avant PowerDown Teensy 3.2");
		//Mode ECO pour la Teensy 3.2
		//WakeUp::calibrate();
		
		//Reveil dans 5mn
		//WakeUp::set_ms(299000);
		
		//FTDI.printf("\r\n\n DeepSleep de 5mn");
		//wait(1);
		//Apres cette instruction le uC s'arrete
		//deepsleep();
		
		//Teensy32_Back2Speed_After_PowerDown();
		//WakeUp::calibrate();
		
		//FTDI.printf("\r\n\n 2eme-Calibration suite PowerDown Teensy 3.2");
		
		//wait(1);
		//FTDI.printf("\r\n\n Reveil et de nouveau 96MHz pour Teensy 3.2");
		
		
	//Utilisation avec printf et USBSerial
	//PC.printf("\r\n core %d",SystemCoreClock);
	//PC.printf("\r\n Bus  %d",bus_frequency());
	//PC.printf("\r\n Osc  %d\r\n",extosc_frequency());
	
	
	*/
	
	/*
	
	if(VEML6075_GetID(ADDRESSE_VEML6075, &I2C_Duree_Time_Out_Ecoulee)!=VEML6075_DEVID) FTDI.printf("\r\nErreur VEML6075 non detecte.");
	else 
	{
		FTDI.printf("\r\nVEML6075 detecte.");
		//Realisation de la mesure ensuite par le VEML6075
		FTDI.printf("\r\nVEML6075 Config=%#x.",VEML6075_CONF_SD_OFF | VEML6075_CONF_IT_100MS);
		//Configuration du VEML6075
		VEML6075_WriteConfig(ADDRESSE_VEML6075, VEML6075_CONF_SD_OFF | VEML6075_CONF_IT_100MS, &I2C_Duree_Time_Out_Ecoulee);
		//raw_uva
		unsigned int raw_uva=VEML6075_GetRauUVABD(ADDRESSE_VEML6075, VEML6075_REG_UVA_Data, &I2C_Duree_Time_Out_Ecoulee);
		//raw_uvb
		unsigned int raw_uvb=VEML6075_GetRauUVABD(ADDRESSE_VEML6075, VEML6075_REG_UVB_Data, &I2C_Duree_Time_Out_Ecoulee);
		//raw_dark
		unsigned int raw_dark=VEML6075_GetRauUVABD(ADDRESSE_VEML6075, VEML6075_REG_UVD_Data, &I2C_Duree_Time_Out_Ecoulee);
		//raw_vis
		unsigned int raw_vis=VEML6075_GetRauUVABD(ADDRESSE_VEML6075, VEML6075_REG_UVCOMP1_Data, &I2C_Duree_Time_Out_Ecoulee);
		//raw_ir
		unsigned int raw_ir=VEML6075_GetRauUVABD(ADDRESSE_VEML6075, VEML6075_REG_UVCOMP2_Data, &I2C_Duree_Time_Out_Ecoulee);
		
		FTDI.printf("\r\nUVA=%3.3f",VEML6075_GetUVA(raw_vis, raw_ir, raw_uva, raw_dark));
		FTDI.printf("\r\nUVB=%3.3f",VEML6075_GetUVB(raw_vis, raw_ir, raw_uvb, raw_dark));
		FTDI.printf("\r\nIndice UV=%3.3f",VEML6075_GetUVIndex(raw_vis, raw_ir, raw_uva, raw_uvb, raw_dark));
		
	}
		
				
				
		*/
	
	/*
		
	FTDI.printf("\r\n\r\r\ncore %d",SystemCoreClock);
	FTDI.printf("\r\nBus  %d",bus_frequency());
	FTDI.printf("\r\nOsc  %d\r\n",extosc_frequency());
		
	Res_CAN=AIN*3.3;
	Res_us=AIN.read_u16();
		
	//PC.printf("\r\n\nCAN sur 16bits");
	//PC.printf("\r\nRes_CAN=%5.5fV",Res_CAN);
	//PC.printf("\r\nRes_us(10)=%d | Res_us(16)=%0x",Res_us,Res_us);
	//PC.printf("\r\nTa=%5.5f C",(Res_CAN-0.5)/0.01);
	
	FTDI.printf("\r\r\n\nCAN sur 16bits");
	FTDI.printf("\r\nRes_CAN=%5.5fV",Res_CAN);
	FTDI.printf("\r\nRes_us(10)=%d | Res_us(16)=%0x",Res_us,Res_us);
	FTDI.printf("\r\nTa=%5.5f C",(Res_CAN-0.5)/0.01);
	
	if(!BP) FTDI.printf("\r\nBP appuye.");
	else FTDI.printf("\r\nBP relache.");
		
		
	//Commande alimentation carte uSD
	COM_SD_POWER=uSD_ON;
	
	*/
	
	/*
	
	//Mount...
		sd.mount();
		
		FTDI.printf("\r\n\nTest de la presence de la carte uSD.");
		
		if(!sd.card_present()) FTDI.printf("\r\n\nAbsence de carte uSD.");
		else
		{
			FTDI.printf("\r\n\nPresence de la carte uSD.");
			FTDI.printf("\r\n\nDebut Ecriture sur carte SD apres mount.");
			//Acquisition de l'heure interne
			seconds=time(NULL);
			FTDI.printf("\r\n\nAcquisition de l'heure : %s",ctime(&seconds));
			
		
			//Creation fu fichier en ajout -> Si il existe il est ouvert en ecriture, sinon cree.
			FILE *fp=fopen("/sd/sdtestV2.txt","a+b");
			if(fp==NULL) {
										FTDI.printf("\r\nImpossible d'ouvrir le fichier en ecriture.");
									 }
			else {
						//Ecriture d'une chaine de caracyeres
						//fprintf(fp, "Hello fun SD Card World!");
						//Ecriture en binaire d'un struct -> date/heure/Temperature
						//Rangement de la temperature et acquisition de l'heure.
						seconds=time(NULL);
						Acqui_Heure_SD.HeureAcqui=seconds;
						Acqui_Heure_SD.Temperature=(Res_CAN-0.5)/0.01;
						fwrite(&Acqui_Heure_SD,sizeof(T_DATA_SD),1,fp);
				
						//Affichage de la position dans le fichier
						FTDI.printf("\r\n\nPosition dans le fichier : %d",ftell(fp));
						FTDI.printf("\r\n\nTaille du struct en octet dans le fichier : %d",sizeof(T_DATA_SD));
						//Affichage du contenu d'un struct enregistre dans le fichier
						fseek(fp,(99+CPT++)*sizeof(T_DATA_SD),0);
						fread(&Acqui_Heure_SD,sizeof(T_DATA_SD),1,fp);
						FTDI.printf("\r\n\nPosition dans le fichier pour lecture : %d",ftell(fp));
						FTDI.printf("\r\n\nLecture Temperature enregistree T=%5.3f°C",Acqui_Heure_SD.Temperature);
						FTDI.printf("\r\n\nHeure d'acquisition enregistree :%s",ctime(&Acqui_Heure_SD.HeureAcqui));
		
						fclose(fp);
						FTDI.printf("\r\n\nFin test carte SD avec ecriture terminee.");
				   }
		
		//unmount...
		sd.unmount();
		}
		
		
	 */
	
	
	/*
	
	//Commande alimentation carte uSD
	COM_SD_POWER=uSD_OFF;
	

	
	FTDI.printf("\r\nTC74 en I2C:");
	
	//Reveil du TC74
	i2c_init_tc_74(ADRESSE_SELECTION_TC74, &I2C_Duree_Time_Out_Ecoulee);
		
	//Affichage message d'erreur si durée de TIME OUT depassée.
	if(!I2C_Duree_Time_Out_Ecoulee) FTDI.printf("\r\nFait -> i2c_init_tc_74.");
	else FTDI.printf("\r\n\ni2c_init_tc_74 TIMEOUT en I2C.");
		
	//Attente fin reveil et acquisition temperature
	while(!i2c_test_ready_tc_74(ADRESSE_SELECTION_TC74, &I2C_Duree_Time_Out_Ecoulee));
	
	//Affichage message d'erreur si durée de TIME OUT depassée.
	if(!I2C_Duree_Time_Out_Ecoulee) FTDI.printf("\r\nFait -> i2c_test_ready_tc_74.");
	else FTDI.printf("\r\n\ni2c_test_ready_tc_74 TIMEOUT en I2C.");
	
	//Lecture de la temperature
	FTDI.printf("\r\nTC74 lecture Temperature=%d C",i2c_lecture_temp_tc74(ADRESSE_SELECTION_TC74, &I2C_Duree_Time_Out_Ecoulee));
	
	//Affichage message d'erreur si durée de TIME OUT depassée.
	if(!I2C_Duree_Time_Out_Ecoulee) FTDI.printf("\r\nFait -> i2c_lecture_temp_tc74.");
	else FTDI.printf("\r\n\ni2c_lecture_temp_tc74 TIMEOUT en I2C.");
	
	//Mise en sommeil du TC74
	i2c_standby_tc_74(ADRESSE_SELECTION_TC74, &I2C_Duree_Time_Out_Ecoulee);
	
	//Affichage message d'erreur si durée de TIME OUT depassée.
	if(!I2C_Duree_Time_Out_Ecoulee) FTDI.printf("\r\nFait -> i2c_standby_tc_74.");
	else FTDI.printf("\r\n\ni2c_standby_tc_74 TIMEOUT en I2C.");
	
	FTDI.printf("\r\nHH10D parametres en I2C:");
	//Lectues des parametres SENSet OFFSET du HH10D
	i2c_lecture_OFFSET_SENS_HH10D(ADRESSE_SELECTION_HH10D, &HH10D_Sens, &HH10D_Offset, &I2C_Duree_Time_Out_Ecoulee);
	//Affichage des 2 parametres si pas d'erreur de TIME OUT.
	if(!I2C_Duree_Time_Out_Ecoulee) FTDI.printf("\r\n\nHH10D parametres SENS=%d OFFSET=%d en I2C.",HH10D_Sens,HH10D_Offset);
	else FTDI.printf("\r\n\nHH10D TIMEOUT en I2C.");
	
	
	//Mesure de la période en sortie de Fout
	//Principe : une mesure sur 1s
	//On compte le nombre d'impulsions et on calcul la valeur moyenne de la période sur 1s
	//Un front sur PTD0 incremente un compteur sur 32bits
	//Un Timer est lancé pour que 1s aprés on arrete le comptage et on lit la valeur du compteur
	//Synthese : PTD0 est une entrée qui déclenche une IT sur un front
	//Synthese : la base de temps est realisée par un TimeOut de 1s
	//Synthese : On dettache ensuite l'IT sur la broche PTD0 -> par NULL
			
	CPT_IT_HH10D=0;
			
	//Desactivation de toutes les IT.
	
	//Association fonction à executer lors du front montant sur Fout du HH10D
	IT_Fout_HH10D.rise(&IT_HH10D);
	//L'entrée du signal Fout est sans resistance interne -> MODE=PullNone
	IT_Fout_HH10D.mode(PullNone);
	
	//Association du Timeout
	Mesure_T_HH10D_1s.attach(&IT_HH10D_CPT,DUREE_MESURE_T_HH10D);
	//Lancement de la mesure
	FTDI.printf("\r\nLancement de la mesure de la periode sur Fout. Attente de %2.0fs",DUREE_MESURE_T_HH10D);
			
	while(FLAG_HH10D);
	FLAG_HH10D=true;		
			
	FTDI.printf("\r\nMesure de %d fronts montants sur %fs.",CPT_IT_HH10D,DUREE_MESURE_T_HH10D);
	FTDI.printf("\r\nSoit une periode sans ajustement de T=%fs | f=%fHz.",DUREE_MESURE_T_HH10D/(CPT_IT_HH10D-1),(CPT_IT_HH10D-1)/DUREE_MESURE_T_HH10D);	
	//Pour OSV3 - unsigned int VAL_HR;
	VAL_HR=(unsigned int) ((HH10D_Offset-((CPT_IT_HH10D-1)/DUREE_MESURE_T_HH10D))*HH10D_Sens)/4096.0;
	FTDI.printf("\r\nCalcul de RH%=%2.2f%%",((HH10D_Offset-((CPT_IT_HH10D-1)/DUREE_MESURE_T_HH10D))*HH10D_Sens)/4096.0);
			
	
	
	FTDI.printf("\r\nPresser une touche pour continuer - AT HC12 - 9600bauds.\r\n");	
	//FTDI.getc();
	
	if(!HC12_CONFIG_DONE) 
{		
	HC12_CONFIG_DONE=true;
	
	Enter_CMD_mode();
			
	//Entrer en mode AT -> "AT"
	sprintf(tab_HC_12_CMD_AT, HC12_AT);
	sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
	if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=OK");
	else 
			{
				FTDI.printf("\r\n sendATcommand=ERROR");
				FTDI.printf("\r\n Changement vitesse en 1200bauds");
				//Changement vitesse transmission RS232 suite nouvelle configuration.
				//Configuration liaison serie
				//reglage usine : 1200 bauds
				HC_12.baud(1200);
				//Tempo pour mise en service de la nouvelle vitesse de la RS232.
				wait_ms(200);
				
			}
			
*/	
		
/*			
	//Configuration mode DEFAULT -> "AT+DEFAULT"
			sprintf(tab_HC_12_CMD_AT, HC12_DEFAULT);
			sprintf(tab_HC_12_reponse_AT, HC12_DEFAULT_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) 
			{
				FTDI.printf("\r\n sendATcommand=AT+DEFAULT");
				//Changement vitesse transmission RS232 suite nouvelle configuration.
				//Configuration liaison serie
				//reglage usine : 9600 bauds
				FTDI.printf("\r\n Changement vitesse en 9600bauds");
				HC_12.baud(9600);
				//Tempo pour mise en service de la nouvelle vitesse de la RS232.
				wait_ms(200);
			}
			else FTDI.printf("\r\n sendATcommand=ERROR");
*/			
			
			
/*
			
		
			//Configuration mode FU4 -> "AT+FU4"
			sprintf(tab_HC_12_CMD_AT, HC12_FU4);
			sprintf(tab_HC_12_reponse_AT, HC12_FU4_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+FU4");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 100mW -> "AT+P8"
			sprintf(tab_HC_12_CMD_AT, HC12_100mW);
			sprintf(tab_HC_12_reponse_AT, HC12_100mW_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+P8");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 441.4MHz -> "AT+C021"
			sprintf(tab_HC_12_CMD_AT, HC12_CANAL21);
			sprintf(tab_HC_12_reponse_AT, HC12_CANAL21_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+C021");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 1200bauds -> "AT+B1200"
			sprintf(tab_HC_12_CMD_AT, HC12_B1200);
			sprintf(tab_HC_12_reponse_AT, HC12_B1200_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+B1200");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			Leave_CMD_mode();
}			
else
{	
			//Mis en commentaire pour tester le LCD HMI
	
	
			//Entrer dans le mode AT
			Entree_Mode_ATcommand_XBee("+++", "OK\r", XBee_TIMEOUT);
			//Lecture ATNI avant changement de nom
			//Lecture_ResultATcommand_XBee("ATNI", "", XBee_Reponse, XBee_TIMEOUT);
			//Changement du nom par ATNI
			sendATcommand_XBee("ATNI", "Test2", "OK\r", XBee_TIMEOUT);
			//Lecture ATAI pour tester si association realisée
			//test par ATAI -> la réponse est 0 si association reussie sinon 0xFF lorsqu'il scan toutes les frequences
			Lecture_ResultATcommand_XBee("ATAI", "", XBee_Reponse, XBee_TIMEOUT);
			Lecture_ResultATcommand_XBee("ATNI", "", XBee_Reponse, XBee_TIMEOUT);
			//Recuperation adresse SH et SL par ATND Nom_Noeud
			Lecture_ATND_Nom_command_XBee("ATND", "S1_N3", XBee_Reponse, XBee_TIMEOUT);
			//Extraction des champs MY'\r'SH'\r'SL'\r'DB'\r'NI'\r'
			//sscanf ne fonctionne pas.
			//sscanf(XBee_Reponse,"%d'\r'%d'\r'%d'\r'%d'\r'%s'\r'",&MY,&SH,&SL,&DB,NI);
			//Test avec strtok :
			//void Extraire_ATND(char *data, unsigned int *MY, unsigned int *SH, unsigned int *SL, unsigned int *DB, char *NI);
			Extraire_ATND(XBee_Reponse, &MY, &SH, &SL, &DB, NI);
			FTDI.printf("\r\nMY=%0x|SH=%0x|SL=%0x|DB=%0x|NI=%s",MY,SH,SL,DB,NI);
			//Detection association reseau
			Lecture_ResultATcommand_XBee("ATAI", "", XBee_Reponse, XBee_TIMEOUT);
			//Ecriture nouvelle configuration - TimeOut de 5s
			sendATcommand_XBee("ATWR", "", "OK\r", 5);
			//Quitter le mode AT
			sendATcommand_XBee("ATCN", "", "OK\r", XBee_TIMEOUT);
			
	
	
			//Affichage information sur taille en octet du type T_DATA_BINAIRE
			FTDI.printf("\r\nTaille en octet de T_DB=%d octets",sizeof(T_DB));
			FTDI.printf("\r\nTaille en octet de TU_DB=%d octets",sizeof(TU_DB));
			//Entrée des données dans TransferBinaireOSV3 de type T_DATA_BINAIRE
			//TransferBinaireOSV3.type_capteur=THGR810;
			//TransferBinaireOSV3.HeureAcqui=time(NULL);
			//TransferBinaireOSV3.Temperature=12.5;
			//TransferBinaireOSV3.HR=33;
			//TransferBinaireOSV3.numero=1;
			//Entrée des données dans l'union.
			UnionOSV3.Data_Capteur.HeureAcqui=time(NULL);
			UnionOSV3.Data_Capteur.type_capteur=THN132N;	//valeur 3 dans enum
			UnionOSV3.Data_Capteur.Temperature=12.5;
			UnionOSV3.Data_Capteur.HR=33;
			UnionOSV3.Data_Capteur.numero=1;
			
			
			//Visualisation des données pour l'ordre.
			//https://os.mbed.com/forum/helloworld/topic/2053/
			//Astuce : float demarre de la position 3
			//char my_array[10];
			//float my_float;
			//my_float = *( (float*)(my_array + 3) );           // from array to float
			
			float OSV3_Temperature = *( (float*)(UnionOSV3.Tab_TU + 4) );
			FTDI.printf("\r\nOSV3_Temperature=%f",OSV3_Temperature);
			
			float OSV3_HR = *( (float*)(UnionOSV3.Tab_TU + 8) );
			FTDI.printf("\r\nOSV3_HR=%f",OSV3_HR);
			
			int OSV3_numero = *( (int*)(UnionOSV3.Tab_TU + 28) );
			FTDI.printf("\r\nOSV3_numero=%d",OSV3_numero);
			
			int OSV3_type_capteur = *( (int*)(UnionOSV3.Tab_TU + 32) );
			FTDI.printf("\r\nOSV3_type_capteur=%d",OSV3_type_capteur);
			
			
*/
			
			
		/*
			//transmission par HC12 en mode DEFAULT.
			int TX_Buffer[]={'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W'};
			
			for(int i=0;i<59;i++)
			{
				//Attente Buffer libre
				while(!HC_12.writeable());
				//Emission d'un caratere
				HC_12.putc((char)TX_Buffer[i]);
				//Affichage par le maitre de l'emission
				FTDI.printf("\r\nEmission de %c",(char)TX_Buffer[i]);
				
				//Emission vers le XBee
				//Attente Buffer libre
				while(!XBee_SX.writeable());
				//Emission d'un caratere
				XBee_SX.putc((char)TX_Buffer[i]);
				
			}
		*/
		
		
		/*
		
		//Test transmission binaire par XBee et HC-12
			for(int i=0;i<36;i++)
			{
				//Attente Buffer libre
				while(!HC_12.writeable());
				//Emission d'un caratere
				HC_12.putc(UnionOSV3.Tab_TU[i]);
				//Affichage par le maitre de l'emission
				FTDI.printf("\r\nEmission de %c",UnionOSV3.Tab_TU[i]);
				
				//Emission vers le XBee
				//Attente Buffer libre
				while(!XBee_SX.writeable());
				//Emission d'un caratere
				XBee_SX.putc(UnionOSV3.Tab_TU[i]);
				
			}
		
		
		*/
		
		/*
			
			FTDI.printf("\r\n\r\nTest communication avec 1 Wire -> DS18B20");
			//Test communication par 1 wire sans CLASS de chez MAXIM
			if(OneWireReset()) FTDI.printf("\r\nPas d'escalve detecte en 1 WIRE.");
			else
			{
				
				FTDI.printf("\r\nEsclave detecte en 1 WIRE.");
				FTDI.printf("\r\nRecherche des adresses en 1 WIRE.");
				
				
				FTDI.printf("\r\nOneWire_ResetSearch en 1 WIRE.");
				OneWire_ResetSearch(&OW_LastDiscrepancy, &OW_LastDevice, &OW_LastFamilyDiscrepancy, ROM_NO);
				
				//FTDI.printf("\r\nOneWire_TargetSearch 0x28 en 1 WIRE.");
				//OneWire_TargetSearch(&OW_LastDiscrepancy, &OW_LastDevice, &OW_LastFamilyDiscrepancy, 0x28, ROM_NO);
				
				
				FTDI.printf("\r\nOneWiresearch en 1 WIRE.");
				bool Rep=OneWiresearch(&OW_LastDiscrepancy, &OW_LastDevice, &OW_LastFamilyDiscrepancy, addr, true, ROM_NO);
				//bool Rep=OneWiresearch(&OW_LastDiscrepancy, &OW_LastDevice, &OW_LastFamilyDiscrepancy, &ROM_NO[0], true, ROM_NO);
				if(Rep)
				{
					FTDI.printf("\r\nAdresse(s) trouvee(s) en 1 WIRE.");
					
				}
				else
				{
					FTDI.printf("\r\nAucun composant trouve en 1 WIRE.");
				}
				
				
				//Recherche de tous les esclaves
				
				FindDevices();
				FTDI.printf("\r\nNombre d'esclave(s) detecte(s) : %d",numROMs);
				
				//Affichage de la valeur de la ROM sur 8 octets
				for(int i=0 ; i<numROMs ; i++)
				{
					FTDI.printf("\r\n");
					for(int j=0 ; j<8 ; j++)
					{
						//FTDI.printf("FoundROM[%d][%d]=%x ",i,j,FoundROM[i][j]);
						FTDI.printf("%#x ",FoundROM[i][j]);
					}
					
				}
				
				FTDI.printf("\r\n");
				
				//lecture Temp pour chaque capteur trouvé et individuelement.
				for(int x=0 ; x<numROMs ; x++)
				{
					FTDI.printf("\r\nEsclave numero :%d -> T=%3.4f°C",x+1,read_one_DS18B20(x));
				}
				
				
				
				if(numROMs==1) {
														FTDI.printf("\r\n\r\n Un seul Esclave detecte en 1 WIRE.");
														FTDI.printf("\r\nLecture des 64bits propre au capteur en 1 WIRE.");
														OneWireOutByte(0x33);  //Skip ROM command
														//Lecture des 8 octects
														for(int i=0 ;i<8;i++)
														{
															data[i]=OneWireInByte();
														}
														//Affichage CRC, n° serie sur 6 octets et code famille
														FTDI.printf("\r\nCRC=%#x | Serie=%x %x %x %x %x %x | Famille=%#x",data[7],data[6],data[5],data[4],data[3],data[2],data[1],data[0]);
														FTDI.printf("\r\nCRC lu=%#x.",data[7]);
														FTDI.printf("\r\nCRC calcule=0x%x.",OneWireCRC8(data,7));
				
														OneWireReset();
				
														OneWireOutByte(0xCC);  //Skip ROM command
														OneWireOutByte(0x44); // perform temperature conversion
														//wait_ms(750);
														//Attente fin de conversion par lecture NL retourné par le capteur
														//1-> mesure terminée 0-> mesure en cours.
														//t.reset();
														//t.start();
														//Capt_1_Wire.input();
														//wait_us(5);
														//while(!Capt_1_Wire);
														//t.stop();
														t.reset();
														t.start();
														//while(!OneWireIn_1_Bit());
														while(!OneWireInByte());
														t.stop();
														FTDI.printf("\r\nMesure sur 12bits terminee en 1 WIRE.");
														FTDI.printf("\r\nMesure sur 12bits terminee en : %fs.",t.read());
				
														OneWireReset();
														OneWireOutByte(0xCC);
				
				
														OneWireOutByte(0xBE);   //Read Scratchpad
				
														for(int i=0 ;i<9;i++)
														{
															data[i]=OneWireInByte();
														}
				
														//verification du CRC
														//Lecture de 9 octets et CRC calculé sur 8 octets.
														if( OneWireCRC8(data, 8) != data[8]) FTDI.printf("\r\nErreur de CRC Scratchpad en 1 WIRE.");
														else 
														{
															FTDI.printf("\r\nCRC OK Scratchpad en 1 WIRE.");
															FTDI.printf("\r\nCRC lu=%#x.",data[8]);
															FTDI.printf("\r\nCRC calcule=0x%x.",OneWireCRC8(data, 8));
															FTDI.printf("\r\nConfig register=0x%x",data[4]);
															FTDI.printf("\r\nAlarme TH=%d",data[2]);
															FTDI.printf("\r\nAlarme TL=%d",data[3]);
						
															//LowByte = data[0];
															//HighByte = data[1];
						
															switch(data[4]>>5)
																{
																	case 0b11 : FTDI.printf("\r\nMesure sur 12bits demandee.");
																							Result = (data[1] << 8) + data[0];
																							//Pour OSV3 - VAL_TempC 
																							VAL_TempC=(float)(Result*0.0625);
																							FTDI.printf("\r\nTemperature sur 12bits DS18B20 : %0.3f°C\r\n",(float)(Result*0.0625));
																							break;
							
																	case 0b00 : FTDI.printf("\r\nMesure sur 9bits demandee.");
																							Result = (data[1] << 8) + data[0];
																							Result=Result >>3;
																							//Pour OSV3 - VAL_TempC 
																							VAL_TempC=(float)(Result*0.5);
																							FTDI.printf("\r\nTemperature sur 9bits DS18B20 : %0.3f°C\r\n",(float)(Result*0.5));
																							break;
							
																	case 0b01 : FTDI.printf("\r\nMesure sur 10bits demandee.");
																							Result = (data[1] << 8) + data[0];
																							Result=Result >>2;
																							//Pour OSV3 - VAL_TempC 
																							VAL_TempC=(float)(Result*0.25);
																							FTDI.printf("\r\nTemperature sur 10bits DS18B20 : %0.3f°C\r\n",(float)(Result*0.25));
																							break;
							
																	case 0b10 : FTDI.printf("\r\nMesure sur 11bits demandee.");
																							Result = (data[1] << 8) + data[0];
																							Result=Result >>1;
																							//Pour OSV3 - VAL_TempC 
																							VAL_TempC=(float)(Result*0.125);
																							FTDI.printf("\r\nTemperature DS18B20 : %0.3f°C\r\n",(float)(Result*0.125));
																							break;
																}
						
						
						
						
						
															}
				
					
					
															FTDI.printf("\r\nChangement resolution mesure sur 12,11,10,9 bits en 1 WIRE.");
															//Changement résolution capteur
															OneWireReset();
															OneWireOutByte(0xCC);  //Skip ROM command
															OneWireOutByte(0x4E);  //ROM Write Scratchpad 
															//Lecture de TH=0x4B ->75°C
															//Lecture de TL=0x46 ->70°C
															//Ecriture de 3 octets
															//Lecture des 8 octects
															//data[0]=0x4B;//TH
															//data[1]=0x46;//TL
															data[0]=30;//TH=30°C
															data[1]=-20;//TL=-20°C
															//data[2]=0b01011111;//Config Register ici mesure sur R1R0=10->11bits
															data[2]=0b01111111;//Config Register ici mesure sur R1R0=11->12bits
															//data[2]=0b00111111;//Config Register ici mesure sur R1R0=01->10bits
															//data[2]=0b00011111;//Config Register ici mesure sur R1R0=00->9bits
															for(int i=0 ;i<3;i++)
															{
																OneWireOutByte(data[i]);
															}
															//Memorisation de la configuration
															OneWireReset();
					
															//Lecture du mode d'alimentation du DS18B20
															OneWireOutByte(0xCC);  //Skip ROM command
															OneWireOutByte(0xB4);  //ROM Read Power Supply
															//lecture du resultat par 1->Externe Power 0->Parasite Power
															FTDI.printf("\r\nAnalyse du type d'alimentation du capteur en 1 WIRE.");
															if(OneWireIn_1_Bit()) FTDI.printf("\r\nAlimentation en externe du capteur en 1 WIRE.");
															else FTDI.printf("\r\nAlimentation en Parasite Power du capteur en 1 WIRE.");
				
													}
					
					
			}
				
				
			//Transmission en 433.92MHz par OSV3
			//Codage du protocole OSV3 - On transmet la T°C du 1 WIRE et HR du HH10D
			//Dans DOMOTICSZ, le rolling code est de 65(10)
			//VAL_TempC=-3.2;
			//VAL_HR=32;
			//Les valeurs de VAL_TempC et VAL_HR sont plus haut dans le code : 1WIRE et HH10D
			
			//Calcul d'un seul ROLLING CODE pour simuler changement de piles
			if(!Fait_ROOLING_CODE) 
			{
				//VAL_ROLLING_CODE=OSV3_ROLLING_CODE();
				VAL_ROLLING_CODE=65;
				Fait_ROOLING_CODE=true;
			}
			
			FTDI.printf("\r\n\n Calcul du ROLLING Code ->%d",VAL_ROLLING_CODE);
	
			
		
			FTDI.printf("\r\n\n Power ON du module 433.92MHz pour OSV3 et attente de 1s :");
			POWER_433=1;
			wait(1);
						
	
			FTDI.printf("\r\n\n Construction de la trame OSV3 :");
			VAL_HR=99;
			OSV3_CONSTRUIRE_TRAME(VAL_TempC, VAL_HR);
			
			FTDI.printf("\r\n\n Transmission de la trame en OSV3 :T=%3.2f°C | HR=%d%%",VAL_TempC,VAL_HR);
			OSV3_MANCHESTER_SEND_DATA();
			FTDI.printf("\r\n\n Fin transmission de la trame en OSV3 :T=%3.2f°C | HR=%d%%",VAL_TempC,VAL_HR);
		
		
			//wait_ms(30);
			POWER_433=1;
			FTDI.printf("\r\n\n Power OFF du module 433.92MHz pour OSV3 :");
		
		
		
		//LED1_ON_Off=!LED1_ON_Off;
		
		//Boucle attente
		//while(FLAG);
		
		//Pour attendre dans le while(FLAG);
		//FLAG=TRUE;
		
		//Traitement
		//if(LED1_ON_Off) PC.printf("LED1 allumée.\n\r");
		//else PC.printf("LED1 éteinte.\n\r");
			
 }		
			
		
*/
 

//-----------------------------------------------------------------------
//Commande servo-moteur : par PWM
//-----------------------------------------------------------------------
			//Test pour OPTIMA
			//PWM_HB.pulsewidth_us(HB);
			//PWM_DG.pulsewidth_us(DG);

			//DG=DG+0.1;
			//HB=HB+0.1;

			//if(DG>1900) DG=1500;
			//if(HB>1900) HB=1500;


	/*	
		
			//Attente fin de transmission avant entrer en mode SLEEP
			//wait(0.5);

			Enter_CMD_mode();
			
			//Entrer en mode AT -> "AT"
			sprintf(tab_HC_12_CMD_AT, HC12_AT);
			sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand en 9600bauds = OK");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Leave_CMD_mode();
			
			//Enter_CMD_mode();
			
			//Entrer en mode SLEEP -> "AT+SLEEP"
			sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
			sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+SLEEP");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			Leave_CMD_mode();
			
	*/
	
	
	
/*	
	Enter_CMD_mode();
			
	//Entrer en mode AT -> "AT"
	sprintf(tab_HC_12_CMD_AT, HC12_AT);
	sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
	if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=OK");
	else 
			{
				FTDI.printf("\r\n sendATcommand=ERROR");
				FTDI.printf("\r\n Changement vitesse en 1200bauds");
				//Changement vitesse transmission RS232 suite nouvelle configuration.
				//Configuration liaison serie
				//reglage usine : 1200 bauds
				HC_12.baud(1200);
				//Tempo pour mise en service de la nouvelle vitesse de la RS232.
				wait_ms(200);
				
			}
			
			
			//Configuration en FU4 - 1200bauds - canal 1 (433.4MHz) - 100mW ou 20dBm
			//FU4 -> AT+FU4
			//100mW -> AT+P8
			//433.4MHz -> AT+C001
			//1200bauds -> AT+B1200
			
			//Configuration mode FU4 -> "AT+FU4"
			sprintf(tab_HC_12_CMD_AT, HC12_FU4);
			sprintf(tab_HC_12_reponse_AT, HC12_FU4_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+FU4");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 100mW -> "AT+P8"
			sprintf(tab_HC_12_CMD_AT, HC12_100mW);
			sprintf(tab_HC_12_reponse_AT, HC12_100mW_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+P8");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 433.1MHz -> "AT+C001"
			sprintf(tab_HC_12_CMD_AT, HC12_CANAL1);
			sprintf(tab_HC_12_reponse_AT, HC12_CANAL1_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+C001");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 1200bauds -> "AT+B1200"
			sprintf(tab_HC_12_CMD_AT, HC12_B1200);
			sprintf(tab_HC_12_reponse_AT, HC12_B1200_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+B1200");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Entrer en mode SLEEP -> "AT+SLEEP"
			sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
			sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+SLEEP");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			Leave_CMD_mode();
			
			//Transmission ici
			
			
			int TX_Buffer[]={'0','1','2','3','4','5','6','7','8','9'};
			
			for(int i=0;i<10;i++)
			{
				//Attente Buffer libre
				while(!HC_12.writeable());
				//Emission d'un caratere
				HC_12.putc((char)TX_Buffer[i]);
				//Affichage par le maitre de l'emission
				FTDI.printf("\r\nEmission de %c",(char)TX_Buffer[i]);
				
			}
			
			
			
			Enter_CMD_mode();
			
			//Entrer en mode AT -> "AT"
			sprintf(tab_HC_12_CMD_AT, HC12_AT);
			sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand en 1200bauds = OK");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			//Leave_CMD_mode();
			
			//Enter_CMD_mode();
			
			//Entrer en mode SLEEP -> "AT+SLEEP"
			sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
			sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+SLEEP");
			else FTDI.printf("\r\n sendATcommand=ERROR");
			
			Leave_CMD_mode();

*/		


	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
		/*
		Res_CAN=AIN*3.3;
		Res_us=AIN.read_u16();
		
		//PC.printf("\r\n\nCAN sur 16bits");
		//PC.printf("\r\nRes_CAN=%5.5fV",Res_CAN);
		//PC.printf("\r\nRes_us(10)=%d | Res_us(16)=%0x",Res_us,Res_us);
		//PC.printf("\r\nTa=%5.5f C",(Res_CAN-0.5)/0.01);
		
		for (cpt_buffer=1 ; ; cpt_buffer++)
		{
			if(FTDI.writeable()) {
															FTDI.putc(cpt_buffer);
													 }
			else {
								break;
					 }
		}
		
		PC.printf("\r\n\nbreak pour cpt_buffer=%d",cpt_buffer);
		
		//Utilisation avec printf et USBSerial
	PC.printf("\r\n core %d",SystemCoreClock);
	PC.printf("\r\n Bus  %d",bus_frequency());
	PC.printf("\r\n Osc  %d",extosc_frequency());
		*/
		/*
		
		//Mount...
		sd.mount();
		
		PC.printf("\r\n\nDebut Ecriture sur carte SD apres mount.");
		
		FILE *fp=fopen("/sd/sdtest.txt","w");
		if(fp==NULL) {
										PC.printf("\r\n\nCould not open file for write.");
								 }
		else {
						fprintf(fp, "Hello fun SD Card World!");
		
						fclose(fp);
						PC.printf("\r\n\nFin test carte SD avec ecriture terminee.");
				 }
		
		//unmount...
		sd.unmount();
		
		*/
		

	
    
}

//---------------------------------------------------------------------------

//Codage des fonctions
void IT_LED(void)
{
	//LED1_ON_Off=LED1_ON_Off^1;
	FLAG=false;
}


char i2c_lecture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE, bool *TimeOut_Ended)
{
        bool FLAG=false;
        char temp;
				
				Timer t;
				t.reset();
				t.start();
	
				float debut = t.read();
	
				*TimeOut_Ended=false;

			
        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);
				
				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255};

								do
                {

                        if (Capt_I2C0.write(ADRESSE_EEPROM,NULL,0))
                         {
                          FLAG=true;
													if((t.read()-debut)>I2C_TIMEOUT) {
																															t.stop();
																															//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																															*TimeOut_Ended=true;
																															break;
																													 }
                         }
                        else
                                {
                                        FLAG=false;
																				t.stop();
																	
																				Capt_I2C0.write(ADRESSE_EEPROM, tab, 2, true);
																	
                                        //Capt_I2C0.write(ADRESSE_INTERNE>>8);
                                        //i2c1__ack_S2M();
																				//Capt_I2C0.write(ADRESSE_INTERNE&255);
                                        //i2c1__ack_S2M();
																				
																				Capt_I2C0.read(ADRESSE_EEPROM, &temp, 1);
																	
                                        //i2c1__rstart();
                                        //Mise à 1 du bit R/W de l'octet de "controle": pour une lecture
                                        //i2c1__write(ADRESSE_EEPROM|1);
                                        //i2c1__ack_S2M();

                                        //temp=i2c1__read(I2C_NOACQ);
                                        //noack nécessaire ici, car stop ensuite.
                                        //i2c1__noack_M2S();

                                        //i2c1__stop();
                                        //StopI2C1();
                                        return(temp);
                                }
                }
								while(FLAG);

}


void i2c_lecture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE,  int NOMBRE_A_LIRE, char *dest, bool *TimeOut_Ended)
{
        bool FLAG=false;
        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);

				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255};
				
				*TimeOut_Ended=false;
				
				Timer t;
				t.reset();
				t.start();
	
				float debut = t.read();
        
        do
        {
        
                if(Capt_I2C0.write(ADRESSE_EEPROM,NULL,0))
                        {
                         FLAG=true;
												 if((t.read()-debut)>I2C_TIMEOUT) {
																															t.stop();
																															//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																															*TimeOut_Ended=true;
																															break;
																													}
                        }
                  
                else
                {
                        FLAG=false;
												t.stop();
									
												Capt_I2C0.write(ADRESSE_EEPROM, tab, 2, true);
                        
                        //i2c1__write(ADRESSE_INTERNE>>8);
                        //i2c1__ack_S2M();
                        //i2c1__write(ADRESSE_INTERNE&255);
                        //i2c1__ack_S2M();
        
                        ///*
                        ///////////////////////////////////////////////////////////
                        //1ére Solution :
                        ///////////////////////////////////////////////////////////
        
												Capt_I2C0.read(ADRESSE_EEPROM, dest , NOMBRE_A_LIRE);
									
												dest[NOMBRE_A_LIRE]='\0';
									
												/*
                        i2c1__rstart();
                        i2c1__write(ADRESSE_EEPROM|1);
                        //i2c1__ack_S2M();

                        for(i=0;i<NOMBRE_A_LIRE;i++)
                        {
                          if(i==NOMBRE_A_LIRE-1)
                                                {
                                                  dest[i]=i2c1__read(I2C_NOACQ);
                                                  //i2c1__noack_M2S();
                                                }
                          else
                          {
                            dest[i]=i2c1__read(I2C_ACQ);
                            //i2c1__ack_M2S();
                          }
                         }
               
                         i2c1__stop();
                         dest[i]='\0';
												 */

                }
       
        }
        while(FLAG);

}


void i2c_ecriture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char DATA, bool *TimeOut_Ended)
{
        bool FLAG=false;
        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);
	
				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255, DATA};
				
				*TimeOut_Ended=false;
				
				Timer t;
				t.reset();
				t.start();
	
				float debut = t.read();

								do
                {
                        if (Capt_I2C0.write(ADRESSE_EEPROM,NULL,0))
                         {
                           FLAG=true;
													 if((t.read()-debut)>I2C_TIMEOUT) {
																															t.stop();
																															//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																															*TimeOut_Ended=true;
																															break;
																														}
                         }
                        else
                                {
                                        FLAG=false;
																				t.stop();
																	
																				Capt_I2C0.write(ADRESSE_EEPROM, tab, 3);
																	
                                        //i2c1__write(ADRESSE_INTERNE>>8);
                                        //i2c1__ack_S2M();
                                        //i2c1__write(ADRESSE_INTERNE&255);
                                        //i2c1__ack_S2M();
                                        //i2c1__write(DATA);
                                        //i2c1__ack_S2M();
                                        
                                        //i2c1__stop();
                                }
                }
								while(FLAG);
}


void i2c_ecriture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char *t, bool *TimeOut_Ended)
{
        bool PAGE=false;
        bool FLAG=false;
				int i,j=0;
				char *p;
				p=t;
        //int offset=0,i;
	
				*TimeOut_Ended=false;
	
				Timer temps;
				temps.reset();
				temps.start();
	
				float debut;

        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);
		
				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255};
				
				//recherche de la taille du tableau
				//Si le tableau est une chaine de caractéres
				//terminaison par '\0'
				i=strlen(t);
				
								do
                {
									debut=temps.read();
									
												do
                        {
                                if (Capt_I2C0.write(ADRESSE_EEPROM,NULL,0))
                                 {
                                  FLAG=true;
																	if((temps.read()-debut)>I2C_TIMEOUT) {
																																				temps.stop();
																																				//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																																				*TimeOut_Ended=true;
																																				PAGE=false;
																																				break;
																																			 }
                                 }
                                else
                                {
                                        FLAG=false;
																				temps.reset();
																	
																				Capt_I2C0.write(ADRESSE_EEPROM, tab, 2);
																	
                                        //Capt_I2C0.write(ADRESSE_INTERNE>>8);
                                        //i2c1__ack_S2M();
                                        //i2c1__write(ADRESSE_INTERNE&255);
                                        //i2c1__ack_S2M();
																	
																				
																				//Calculons la division et le modulo
																				if((i/TAILLE_BUFFER)==0) { 
																																	// reste à ecrire
																																	i= i % TAILLE_BUFFER;
																					
																																	Capt_I2C0.write(ADRESSE_EEPROM, p, i);
																					
																																	//ADRESSE_INTERNE=(ADRESSE_INTERNE+i)& MASK_ADRESSE_INTERNE;
																																	//tab[0]=ADRESSE_INTERNE>>8;
																																	//tab[1]=ADRESSE_INTERNE&255;
																					
																																	PAGE=false;
																																	}
																				else {
																							//Ecriture de TAILLE_BUFFER octets
																							
																							Capt_I2C0.write(ADRESSE_EEPROM, p, TAILLE_BUFFER);
																					
																							ADRESSE_INTERNE=(ADRESSE_INTERNE+TAILLE_BUFFER)& MASK_ADRESSE_INTERNE;
																							tab[0]=ADRESSE_INTERNE>>8;
																							tab[1]=ADRESSE_INTERNE&255;
																					
																							//Pointer sur le nouvel élèment à copier
																							j=j+TAILLE_BUFFER;
																							p=&t[j];
																					
																							//reste à ecrire
																							i=i-TAILLE_BUFFER;
																					
																							FLAG=true;
																							PAGE=true;
																					
																							temps.stop();
																						 }
																	
																	
																				/*
                                        while(t[i+offset]!='\0' && i<TAILLE_BUFFER)
                                                {
                                                        i2c1__write(t[offset+i++]);
                                                        //i2c1__ack_S2M();
                                                }

                                        i2c1__stop();
                                        
                                        ADRESSE_INTERNE=(ADRESSE_INTERNE+i)& MASK_ADRESSE_INTERNE;
                                        if (i==TAILLE_BUFFER && t[i+offset]!='\0')
                                         {
                                          offset=offset+TAILLE_BUFFER;
                                          PAGE=TRUE;
                                          i=0;
                                         }
                                        else PAGE=FALSE;
																				*/
                                }
                        }
												while(FLAG);
                }
								while(PAGE);
}





void i2c_lecture_OFFSET_SENS_HH10D(int ADRESSE_HH10D,int *SENS, int *OFFSET, bool *TimeOut_Ended)
{
        bool FLAG=false;
				char tab[2];
	
				*TimeOut_Ended=false;
	
				Timer t;
				t.reset();
				t.start();
	
				float debut = t.read();

								do
                {

                        if (Capt_I2C0.write(ADRESSE_HH10D,NULL,0))
                           {
                            FLAG=true;
														if((t.read()-debut)>I2C_TIMEOUT) {
																															t.stop();
																															//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																															*TimeOut_Ended=true;
																															break;
																														 }
                           }
                        else
                            {
                                        FLAG=false;
																				t.stop();
															
																				tab[0]=ADRESSE_SENS_HH10D;
															
																				Capt_I2C0.write(ADRESSE_HH10D, tab, 1, true);
																				Capt_I2C0.read(ADRESSE_HH10D,tab,2);
															
																				*SENS=(tab[0]<<8)|tab[1];
															
																				//SENS_MSB=tab[0];
															
																				/*
                                        //Emission adresse de début de lecture : 10. 4 octets à lire
                                        i2c1__write(ADRESSE_SENS_HH10D);
                                        //i2c1__ack_S2M();
                                        i2c1__rstart();

                                        //Mise à 1 du bit R/W de l'octet de "contrôle": pour une lecture
                                        i2c1__write(ADRESSE_HH10D|1);
                                        //i2c1__ack_S2M();

                                        //Lecture MSB de SENS @10
                                        SENS_MSB=i2c1__read(I2C_NOACQ);
                                        //i2c1__noack_M2S();
                                        
                                        i2c1__stop();
																				*/

                             }
                }
								while(FLAG);

							
								t.reset();
								t.start();
								debut = t.read();
								
								
								do
                {

                        if (Capt_I2C0.write(ADRESSE_HH10D,NULL,0))
                           {
                              FLAG=true;
															if((t.read()-debut)>I2C_TIMEOUT) {
																																t.stop();
																																//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																																*TimeOut_Ended=true;
																																break;
																															 }
                           }
                        else
                                {
                                        FLAG=false;
																				t.stop();
																	
																				tab[0]=ADRESSE_OFFSET_HH10D;
															
																				Capt_I2C0.write(ADRESSE_HH10D, tab, 2, true);
																				Capt_I2C0.read(ADRESSE_HH10D,tab,1);
															
																				*OFFSET=(tab[0]<<8)|tab[1];
																	
																	
																				/*
                                        //Emission adresse de début de lecture : 10. 4 octets à lire
                                        i2c1__write(ADRESSE_OFFSET_HH10D);
                                        //i2c1__ack_S2M();
                                        i2c1__rstart();

                                        //Mise à 1 du bit R/W de l'octet de "controle": pour une lecture
                                        i2c1__write(ADRESSE_HH10D|1);
                                        //i2c1__ack_S2M();

                                        //Lecture MSB de OFFSET @12
                                        OFFSET_MSB=i2c1__read(I2C_NOACQ);
                                        //i2c1__noack_M2S();
                                        
                                        i2c1__stop();
																				*/

                                }
                }
								while(FLAG);
}


void IT_HH10D(void)
{
	CPT_IT_HH10D++;
}


void IT_HH10D_CPT(void)
{
	IT_Fout_HH10D.rise(NULL);
	FLAG_HH10D=false;
}




void i2c_init_tc_74(int ADRESSE_TC74, bool *TimeOut_Ended)
{
        bool FLAG=false;
				char temp[]={TC74_CONFIG,0};
				
				*TimeOut_Ended=false;
	
				Timer t;
				t.reset();
				t.start();
				
				float debut = t.read();
			
        do
         {

          if (Capt_I2C0.write(ADRESSE_TC74,NULL,0))
             {
              FLAG=true;
							 
							if((t.read()-debut)>I2C_TIMEOUT) {
																								t.stop();
																								//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																								*TimeOut_Ended=true;
																								break;
																							 }
             }
          else
             {
              FLAG=false;
							t.stop();
							 
              Capt_I2C0.write(ADRESSE_TC74,temp,2);
              //Mode normal (b7=0); mode standby (b7=1)
             }
         }
        while(FLAG);
}


void i2c_standby_tc_74(int ADRESSE_TC74, bool *TimeOut_Ended)
{
        bool FLAG=false;
				char temp[]={TC74_CONFIG,0b10000000};
				
				*TimeOut_Ended=false;
	
				Timer t;
				t.reset();
				t.start();
				
				float debut = t.read();
 
        do
         {

          if (Capt_I2C0.write(ADRESSE_TC74,NULL,0))
             {
              FLAG=true;
							if((t.read()-debut)>I2C_TIMEOUT) {
																								t.stop();
																								//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																								*TimeOut_Ended=true;
																								break;
																							 }
             }
          else
              {
               FLAG=false;
							 t.stop();
								
               Capt_I2C0.write(ADRESSE_TC74,temp,2);
               //Mode normal (b7=0); mode standby (b7=1)
              }
         }
        while(FLAG);
}


bool i2c_test_ready_tc_74(int ADRESSE_TC74, bool *TimeOut_Ended)
{

        bool FLAG=false;
        char temp=TC74_CONFIG;
	
				*TimeOut_Ended=false;
	
				Timer t;
				t.reset();
				t.start();
				
				float debut = t.read();

        do
         {
          if (Capt_I2C0.write(ADRESSE_TC74,NULL,0))
             {
              FLAG=true;
							if((t.read()-debut)>I2C_TIMEOUT) {
																								t.stop();
																								//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																								*TimeOut_Ended=true;
																								break;
																							 }
             }
          else
             {
              FLAG=false;
							t.stop();
							 
              Capt_I2C0.write(ADRESSE_TC74,&temp,1,true);
              //Mode normal (b7=0); mode standby (b7=1)
							 Capt_I2C0.read(ADRESSE_TC74,&temp,1);
              //Test du bit b6 (1:Ready)
							 return(temp&0b01000000); 
             }
         }
        while(FLAG);
}


signed char i2c_lecture_temp_tc74(int ADRESSE_TC74, bool *TimeOut_Ended)
{
        bool FLAG=false;
        char temp=TC74_TEMP;
	
				*TimeOut_Ended=false;
	
				Timer t;
				t.reset();
				t.start();
				
				float debut = t.read();

				do
         {

                        if (Capt_I2C0.write(ADRESSE_TC74,NULL,0))
                         {
                          FLAG=true;
													if((t.read()-debut)>I2C_TIMEOUT) {
																														t.stop();
																														//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																														*TimeOut_Ended=true;
																														break;
																													 }
                         }
                        else
                           {
                            FLAG=false;
														t.stop();
														 
                            Capt_I2C0.write(ADRESSE_TC74,&temp,1,true);
                            //Mode normal (b7=0); mode standby (b7=1)
														Capt_I2C0.read(ADRESSE_TC74,&temp,1);
                            return((signed char)temp);
														//return(0b10111111);
                           }
         }
        while(FLAG);
}

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////

bool sendATcommand(char *ATcommand, char *expected_answer, float timeout)
{
    int i=0;
		float debut;  
		bool answer=false;
	
    static char buffer[HC12_TAILLE_BUFFER];
		static char reponse[HC12_TAILLE_BUFFER];
    
		Timer t;
		//Initialisation par le '\0' sur la totalité
    memset(reponse, '\0', HC12_TAILLE_BUFFER);
		//memset(buffer, '\0', HC12_TAILLE_BUFFER);
    
		//Attente si ecriture impossible via RS232 pour configurer HC12
		while(!HC_12.writeable());
    //Preparation de la commande AT et terminée par CR non necessaire ici
		sprintf(buffer,"%s%s",ATcommand,HC12_END_CMD);
	
    HC_12.printf("%s",buffer);
		
		//Preparation du Timeout si la reponse du HC12 ne vient pas...
		t.reset();
		t.start();
	
    debut = t.read();
 
    // Gestion de la reponse du HC12
    do
		{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(HC_12.readable())
					{    
            reponse[i++] = (char)HC_12.getc();
            // test si la reponse est bien OK
            if (strstr(reponse, expected_answer) != NULL)
							{
                answer = true;
							}
					}	
    // Test si la reponse OK est bien presente ou le Timeout est terminé 
    } 
		while((answer == false) && ((t.read() - debut) < timeout));  

		//Arret du Timer
		t.stop();
		
		//DEBUG
		//PC.printf("\r\nTX=%s | RX=%s",buffer,reponse);
		//PC.printf("\r\nTimeout=%fs",t.read() - debut);
		
		//true ou false -> true si OK bien present et false dans l'autre cas
    return (answer);
}


void Enter_CMD_mode(void)
{
	//NL0 pendant >40ms
	CS_HC12=0;
	wait_ms(41);
}

void Leave_CMD_mode(void)
{
	//NL1 et attente MAJ pendant >80ms avant retout mode transparent
	CS_HC12=1;
	wait_ms(81);
}



/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du module XBee par commande AT
/////////////////////////////////////////////////////////////////////////////

bool Entree_Mode_ATcommand_XBee(char *ATcommand, char *expected_answer, float timeout)
{
    int i=0;
		float debut;  
		bool answer=false;
	
		//Taille : +++ et OK\r -> soit 3 caracteres plus '\0' -> 4 caracteres
    static char buffer[4];
		static char reponse[4];
    
		Timer t;
		//Initialisation par le '\0' sur la totalité
    memset(reponse, '\0',4);
	
		//Attente au moins 1s
		wait(1);
		//Attente si ecriture impossible via RS232 pour configurer le module XBee
		while(!XBee_SX.writeable());
    //Preparation de la commande AT "+++" sans CR
		sprintf(buffer,"%s",ATcommand);
	
    XBee_SX.printf("%s",buffer);
		
		//Preparation du Timeout si la reponse du module XBee ne vient pas...
		t.reset();
		t.start();
	
    debut = t.read();
 
    // Gestion de la reponse du module XBee
    do
		{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(XBee_SX.readable())
					{    
            reponse[i++] = (char)XBee_SX.getc();
            // test si la reponse est bien "OK\r"
            if (strstr(reponse, expected_answer) != NULL)
							{
                answer = true;
							}
					}	
    // Test si la reponse OK est bien presente ou le Timeout est terminé 
    } 
		while((answer == false) && ((t.read() - debut) < timeout));  

		//Arret du Timer
		t.stop();
		
		//DEBUG
		//FTDI.printf("\r\nEntree_Mode_ATcommand_XBee : %s %s",ATcommand,expected_answer);
		//FTDI.printf("\r\nBuf=%s | Rep=%s",buffer,reponse);
		//FTDI.printf("\r\nTimeout=%fs",t.read() - debut);
		
		//true ou false -> true si OK bien present et false dans l'autre cas
    return (answer);
}

bool sendATcommand_XBee(char *ATcommand, char *Param, char *expected_answer, float timeout)
{
    int i=0;
		float debut;  
		bool answer=false;
	
    static char buffer[XBee_TAILLE_BUFFER];
		static char reponse[XBee_TAILLE_BUFFER];
    
		Timer t;
		//Initialisation par le '\0' sur la totalité
    memset(reponse, '\0', XBee_TAILLE_BUFFER);
	
		//Attente si ecriture impossible via RS232 pour configurer HC12
		while(!XBee_SX.writeable());
    //Preparation de la commande AT avec CR
		sprintf(buffer,"%s%s%s",ATcommand,Param,XBee_CR);
	
    XBee_SX.printf("%s",buffer);
		
		//Preparation du Timeout si la reponse du module XBee ne vient pas...
		t.reset();
		t.start();
	
    debut = t.read();
 
    // Gestion de la reponse du module XBee
    do
		{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(XBee_SX.readable())
					{    
            reponse[i++] = (char)XBee_SX.getc();
            // test si la reponse est bien "OK\r"
            if (strstr(reponse, expected_answer) != NULL)
							{
                answer = true;
							}
					}	
    // Test si la reponse OK est bien presente ou le Timeout est terminé 
    } 
		while((answer == false) && ((t.read() - debut) < timeout));  

		//Arret du Timer
		t.stop();
		
		//DEBUG
		FTDI.printf("\r\nsendATcommand_XBee :%s %s %s",ATcommand,Param,expected_answer);
		FTDI.printf("\r\nBuf=%s | Rep=%s",buffer,reponse);
		FTDI.printf("\r\nTimeout=%fs",t.read() - debut);
		
		//true ou false -> true si OK bien present et false dans l'autre cas
    return (answer);
}

bool Lecture_ResultATcommand_XBee(char *ATcommand, char *Param, char *answer_XBee, float timeout)
{
    int i=0;
		float debut;  
		bool answer=false;
	
    static char buffer[XBee_TAILLE_BUFFER];
		static char reponse[XBee_TAILLE_BUFFER];
		
    
		Timer t;
		//Initialisation par le '\0' sur la totalité
    memset(reponse, '\0', XBee_TAILLE_BUFFER);
		
		//Attente si ecriture impossible via RS232 pour configurer HC12
		while(!XBee_SX.writeable());
    //Preparation de la commande AT avec CR
		sprintf(buffer,"%s%s%s",ATcommand,Param,XBee_CR);
	
    XBee_SX.printf("%s",buffer);
		
		//Preparation du Timeout si la reponse du module XBeene vient pas...
		t.reset();
		t.start();
	
    debut = t.read();
 
    // Gestion de la reponse du module XBee
    do
		{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(XBee_SX.readable())
					{    
						reponse[i++] = (char)XBee_SX.getc();
						if (strstr(reponse, "\r") != NULL)
							{
                answer = true;
							}
					}
        
						
    // Test si la reponse OK est bien presente ou le Timeout est terminé 
    } 
		while((answer == false) && ((t.read() - debut) < timeout));  

		//Arret du Timer
		t.stop();
		
		//Copie du tableau reponse dans answer_XBee
		i=0;
		while((answer_XBee[i]=reponse[i])!='\0') i++;
		
		//DEBUG
		FTDI.printf("\r\nLecture_ResultATcommand_XBee : %s %s %s",ATcommand,Param,answer_XBee);
		FTDI.printf("\r\nBuf=%s | Rep=%s",buffer,answer_XBee);
		FTDI.printf("\r\nTimeout=%fs",t.read() - debut);
		
		//true ou false -> true si OK bien present et false dans l'autre cas
    return (answer);
}

bool Lecture_ATND_Nom_command_XBee(char *ATcommand, char *Param, char *answer_XBee, float timeout)
{
		int i=0;
		float debut;  
		bool answer=false;
	
    static char buffer[XBee_TAILLE_BUFFER];
		static char reponse[XBee_TAILLE_BUFFER];
		
    
		Timer t;
		//Initialisation par le '\0' sur la totalité
    memset(reponse, '\0', XBee_TAILLE_BUFFER);
		
		//Attente si ecriture impossible via RS232 pour configurer HC12
		while(!XBee_SX.writeable());
    //Preparation de la commande AT 
		sprintf(buffer,"%s%s%s",ATcommand,Param,XBee_CR);
	
    XBee_SX.printf("%s",buffer);
		
		//Preparation du Timeout si la reponse du module XBeene vient pas...
		t.reset();
		t.start();
				
    debut = t.read();
 
    // Gestion de la reponse du module XBee
    do
		{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(XBee_SX.readable())
					{    
						reponse[i++] = (char)XBee_SX.getc();
						
						if (strstr(reponse, Param) != NULL)
							{
								answer = true;
							}
					}
        
						
    // Test si la reponse OK est bien presente ou le Timeout est terminé 
    } 
		while((answer == false) && ((t.read() - debut) < timeout));  

		//Arret du Timer
		t.stop();
		
		//Copie du tableau reponse dans answer_XBee
		i=0;
		while((answer_XBee[i]=reponse[i])!='\0') i++;
		
		//DEBUG
		//FTDI.printf("\r\nLecture_ATND_Nom_command_XBee : %s %s %s",ATcommand,Param,answer_XBee);
		//FTDI.printf("\r\nBuf=%s | Rep=%s",buffer,answer_XBee);
		//FTDI.printf("\r\nTimeout=%fs",t.read() - debut);
		
		//true ou false -> true si OK bien present et false dans l'autre cas
    return (answer);
}
//Attention : atoi suppose que la chaine de caractere est en base 10.
//La soluton est d'utiliser strtol qui permet d'intiquer la base dans lequel est le resultat.
//http://www.cplusplus.com/reference/cstdlib/strtol/
void Extraire_ATND(char *data, unsigned int *MY, unsigned int *SH, unsigned int *SL, unsigned int *DB, char *NI)
{
	
	char *token;
				
	token = strtok(data, "\r");
	*MY =(unsigned int) strtol(token,NULL,16);
	//FTDI.printf("\r\nMY=%s",token);
		
	token = strtok(NULL, "\r");
	*SH = (unsigned int) strtol(token,NULL,16);
	//FTDI.printf("\r\nSH=%s",token);
		
	token = strtok(NULL, "\r");
	*SL = (unsigned int) strtol(token,NULL,16);
	//FTDI.printf("\r\nSL=%s",token);
		
	token = strtok(NULL, "\r");
	*DB = (unsigned int) strtol(token,NULL,16);
	//FTDI.printf("\r\nDB=%s",token);
		
	token = strtok(NULL, "\r");
	strcpy(NI,token);
	//FTDI.printf("\r\nNI=%s",token);
																													
}



/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du capteur 1 Wire DS18B20 
/////////////////////////////////////////////////////////////////////////////

//Fonction validée
bool OneWireReset(void) 
{ // reset.  Avec detection reponse esclave.
	//Lecture DQ : 0-> presence et 1->absence.
		bool DQ;
		//Bus au repos et attente qu'il y soit
		Capt_1_Wire.input();
		while(!Capt_1_Wire);
	
    Capt_1_Wire.output();
    Capt_1_Wire = 0;     // Pulse de reset de 500us
    wait_us(480);
	
    Capt_1_Wire.input();
		wait_us(70);				//attente debut reponse esclave 60us
	
		DQ=Capt_1_Wire;
    wait_us(410);				//attente fin reponse esclave 240us + complement pour 480us
		return(DQ);
}


//Fonction validée
void OneWireOutByte(unsigned char d) 
{ 
	// Emission du Bit de poids faible en premier
	unsigned char bitMask;
	
	for(bitMask=0x01; bitMask ; bitMask <<=1)
	{
		OneWireOut_1_Bit( (bitMask & d) ? 1:0 );
	}
}

/*
void OneWireOutByte(unsigned char d) { // output byte d (least sig bit first).
    for(int n=8; n!=0; n--) {
        if ((d & 0x01) == 1) { // test least sig bit
            temperature_pin.output();
            temperature_pin = 0;
            wait_us(5);
            temperature_pin.input();
            wait_us(60);
        } else {
            temperature_pin.output();
            temperature_pin = 0;
            wait_us(60);
            temperature_pin.input();
        }
 
        d=d>>1; // now the next bit is in the least sig bit position.
    }
 
}
*/

//Fonction validée
void OneWireOut_1_Bit(unsigned char d) 
{ 
   
	 
        if ((d & 0x01)) 
				{ // test least sig bit
            Capt_1_Wire.output();
            Capt_1_Wire = 0;
            wait_us(5);
            Capt_1_Wire.input();
            wait_us(60);
        } else 
				{
            Capt_1_Wire.output();
            Capt_1_Wire = 0;
            wait_us(60);
            Capt_1_Wire.input();
						wait_us(5);
        }

}

//Fonction validée
void OneWireOut_N_Byte(const unsigned char *buf, unsigned int N)
{
	for(unsigned int i=0 ; i<N ; i++)
		OneWireOutByte(buf[i]);
}


//Fonction validée
unsigned char OneWireInByte(void) 
{ 
	//Lecture d'un octet, bit de poids faible en premier.
	unsigned char bitMask,r=0;
	
	for(bitMask=0x01 ; bitMask ; bitMask <<=1)
	{ 
		if(OneWireIn_1_Bit()) r |=bitMask;
	}
	
	return(r);
}

/*
unsigned char OneWireInByte() { // read byte, least sig byte first
    unsigned char d = 0, b;
    for (int n=0; n<8; n++) {
        temperature_pin.output();
        temperature_pin = 0;
        wait_us(5);
        temperature_pin.input();
        wait_us(5);
        b = temperature_pin;
        wait_us(50);
        d = (d >> 1) | (b << 7); // shift d to right and insert b in most sig bit position
    }
    return d;
}
*/

//Fonction validée
unsigned char OneWireIn_1_Bit(void) 
{ 
    unsigned char b;

        Capt_1_Wire.output();
        Capt_1_Wire = 0;
        wait_us(3);
        Capt_1_Wire.input();
        wait_us(10);
        b = Capt_1_Wire;
				//Tempo de 50us pour 1er test
        wait_us(53);
	
    return b;
}


//Fonction validée
void OneWireIn_N_Bytes(unsigned char *buf, unsigned short N)
{
	for(int i=0 ; i<N ; i++)
		buf[i]=OneWireInByte();
}


void OneWireSendCmd(uint8_t *ROMID, uint8_t cmd)
{
		OneWireReset();
	
    if (ROMID == NULL) 
			{
        OneWireOutByte(0xCC);//OW_SKIP_ROM_CMD
			}
		else
			{
        OneWireOutByte(0x55); //OW_MATCH_ROM_CMD
				
        for (int i = 0; i < 8; i++) 
            OneWireOutByte(ROMID[i]);
			}
    OneWireOutByte(cmd);
	
}

//Fonction validée
void OneWire_SelectRom(const unsigned char rom[8])
{
	OneWireOutByte(0x55); //Match ROM
	
	for(int i=0 ; i<8 ; i++) OneWireOutByte(rom[i]);
}

//Fonction validée
void OneWire_SkipRom(void)
{
	OneWireOutByte(0xCC); //SKIP ROM
}

//Fonction validée
void OneWire_ResetSearch(uint8_t *OW_LastDiscrepancy, uint8_t *OW_LastDevice, uint8_t *OW_LastFamilyDiscrepancy, unsigned char *ROM_NO)
{
	//Reset des FLAG pour la recherche
	*OW_LastDiscrepancy=0;
	*OW_LastDevice=false;
	*OW_LastFamilyDiscrepancy=0;
	
	for(int i=7 ; ; i--)
	{
		ROM_NO[i]=0;
		if(i==0) break;
	}
}

//Fonction validée
void OneWire_TargetSearch(unsigned char *OW_LastDiscrepancy, bool *OW_LastDevice, unsigned char *OW_LastFamilyDiscrepancy, unsigned char Family_Code, unsigned char *ROM_NO)
{
	ROM_NO[0]=Family_Code;
	
	for(int i=1 ; i < 8 ; i++)
		ROM_NO[i]=0;
	
	*OW_LastDiscrepancy=64;
	*OW_LastFamilyDiscrepancy=0;
	*OW_LastDevice=false;
	
}


unsigned char OneWireCRC8(const unsigned char *addr, unsigned char len)
{
	unsigned char crc=0;
	
	while(len--)
	{
		unsigned char inbyte=*addr++;
		
		for (int j = 8; j ; j--)
		{
			unsigned char mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
		
	}

	return crc;
}



void OneWireConvertAll(bool wait)
{
	OneWireSendCmd(NULL, 0x44);//OW_CONVERT_T_CMD
	
  if (wait)
		{
				//Capt_1_Wire.input();
        wait_ms(750);//CONVERT_T_DELAY
    }
	
}

/*
int OneWireReadTemperature(uint8_t *ROMID, int *result)
{
		uint8_t crc = 0, buf[8];
    
    OneWireSendCmd(ROMID, 0xBE );//OW_RD_SCR_CMD
	
    for (int i = 0; i < 8; i++) 
		{
        buf[i] = OneWireInByte();
        OneWireCRC(buf[i], &crc);
    }
    if (crc != OneWireInByte()) 
			{
        return (0x8000);//ERR_BADCRC
			}
 
    switch (ROMID[0]) 
		{
        case 0x10: // ds18s20
											*result = (signed char)((buf[1] & 0x80) | (buf[0] >> 1)) * 100 + 75 - buf[6] * 100 / 16;
											break;
        case 0x28: // ds18b20
											*result = (signed char)((buf[1] << 4) | (buf[0] >> 4)) * 100 + (buf[0] & 0x0F) * 100 / 16;
											break;
        default:
											return (0x8001);//ERR_BADFAMILY
    }
    return (0);
}
*/


//Fonction validée
bool OneWiresearch(uint8_t *OW_LastDiscrepancy, uint8_t *OW_LastDevice, uint8_t *OW_LastFamilyDiscrepancy, uint8_t *newAddr, bool search_mode /* = true */,unsigned char *ROM_NO)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
	 uint8_t id_bit, cmp_id_bit;
	 uint8_t search_result;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;

   // if the last call was not the last one
   if (!*OW_LastDevice)
   {
      // 1-Wire reset
      if (!OneWireReset())
      {
         // reset the search
         *OW_LastDiscrepancy = 0;
         *OW_LastDiscrepancy = false;
         *OW_LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true) 
				{
					OneWireOutByte(0xF0);   // NORMAL SEARCH
				} 
			else {
						 OneWireOutByte(0xEC);   // CONDITIONAL SEARCH
					 }

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = OneWireIn_1_Bit();
         cmp_id_bit = OneWireIn_1_Bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < *OW_LastDiscrepancy)
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == *OW_LastDiscrepancy);

               // if 0 was picked then record its position in LastZero
               if (search_direction == 0)
               {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     *OW_LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            OneWireOut_1_Bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         *OW_LastDiscrepancy = last_zero;

         // check for last device
         if (*OW_LastDiscrepancy == 0)
            *OW_LastDevice = true;

         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0])
   {
      *OW_LastDiscrepancy = 0;
      *OW_LastDevice = false;
      *OW_LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
}


//-----------------------------------------------------------------------------------
//Fonctions pour la recherche des escalves
//-----------------------------------------------------------------------------------

void FindDevices(void)
{

	if(!OneWireReset()) //Begins when a presence is detected
	{
		if(First()) //Begins when at least one part is found
		{
			numROMs=0;
			do
			{
			
				for(int m=0 ; m<8 ; m++)
				{
					FoundROM[numROMs][m]=ROM[m]; //Identifies ROM number on found device
				} 
				numROMs++;
			}
			while (Next()&&(numROMs<MAX_SENSORS)); //Continues until no additional devices are found
		}

	}
}

unsigned char First(void)
{
	lastDiscrep = 0; // reset the rom search last discrepancy global
	doneFlag = false;
	return Next(); // call Next and return its return value
}


bool Next(void)
{
	unsigned char m = 1; // ROM Bit index
	unsigned char n = 0; // ROM Byte index
	unsigned char k = 1; // bit mask
	unsigned char x = 0;
	unsigned char discrepMarker = 0; // discrepancy marker
	unsigned char g; // Output bit
	bool nxt; // return value
	bool flag;

	nxt = false; // set the next flag to false
	dowcrc = 0; // reset the dowcrc
	flag = OneWireReset(); // reset the 1-Wire
	if(flag||doneFlag) // no parts -> return false
	{
		lastDiscrep = 0; // reset the search
		return false;
	}
	OneWireOutByte(0xF0); // send SearchROM command
	do	// for all eight bytes
	{
		x = 0;
		if(OneWireIn_1_Bit()==1) x = 2;
			wait_us(6);
		if(OneWireIn_1_Bit()==1) x |= 1; // and its complement
			if(x ==3) // there are no devices on the 1-Wire
				break;
			else	
			{
				if(x>0) // all devices coupled have 0 or 1
					g = x>>1; // bit write value for search
				else
				{
// if this discrepancy is before the last
// discrepancy on a previous Next then pick
// the same as last time
					if(m<lastDiscrep)
						g = ((ROM[n]&k)>0);
					else // if equal to last pick 1
						g = (m==lastDiscrep); // if not then pick 0
// if 0 was picked then record
// position with mask k
					if (g==0) 
						discrepMarker = m;
				}
				if(g==1) // isolate bit in ROM[n] with mask k
					ROM[n] |= k;
				else
					ROM[n] &= ~k;
				OneWireOut_1_Bit(g); // ROM search write
				m++; // increment bit counter m
				k = k<<1; // and shift the bit mask k
				if(k==0) // if the mask is 0 then go to new ROM
				{ // byte n and reset mask
					ow_crc(ROM[n]); // accumulate the CRC
					n++; k++;
				}
		}
	}while(n<8); //loop until through all ROM bytes 0-7
	
	if(m<65||dowcrc) // if search was unsuccessful then
		lastDiscrep=0; // reset the last discrepancy to 0
	else
	{
// search was successful, so set lastDiscrep,
// lastOne, nxt
		lastDiscrep = discrepMarker;
		doneFlag = (lastDiscrep==0);
		nxt = true; // indicates search is not complete yet, more
// parts remain
	}
	return nxt;
}

unsigned char ow_crc( unsigned char x)
{
	dowcrc = dscrc_table[dowcrc^x];
	return dowcrc;
}

float read_one_DS18B20(unsigned char index)
{
	static unsigned char data[9];
	static char buffer[9];
	short Result;
	OneWireReset();
	OneWireOutByte(0x55);          //-- Match Rom
	                         //-- Send the Address ROM Code.

	for(int count=0;count<8;count++)
	{
		OneWireOutByte(FoundROM[index][count]);
	}

	OneWireOutByte(0x44); 
	
	while(!OneWireInByte());

	OneWireReset();
	OneWireOutByte(0x55);          //-- Match Rom
	
	for(int count=0;count<8;count++)
	{
		OneWireOutByte(FoundROM[index][count]);
	}

	OneWireOutByte(0xBE);
	
	for(int i=0 ; i<9 ; i++)
	{
	 data[i]=OneWireInByte();
	}
				
	//verification du CRC
	//Lecture de 9 octets et CRC calculé sur 8 octets.
	if( OneWireCRC8(data, 8) != data[8]) FTDI.printf("\r\nErreur de CRC Scratchpad en 1 WIRE.");
	else 
	{
			FTDI.printf("\r\nCRC OK Scratchpad en 1 WIRE.");
			FTDI.printf("\r\nCRC lu=%#x.",data[8]);
			FTDI.printf("\r\nCRC calcule=0x%x.",OneWireCRC8(data, 8));
			FTDI.printf("\r\nConfig register=0x%x",data[4]);
			//FTDI.printf("\r\nAlarme TH=%d°C",data[2]);
			if(data[2] & 0x80) 
			{
				data[2]=~data[2]+1;
				sprintf(buffer,"-%d",data[2]);
			}
			else sprintf(buffer,"%d",data[2]);
			FTDI.printf("\r\nAlarme TH=%s°C",buffer);
			//FTDI.printf("\r\nAlarme TL=%d°C",data[3]);
			
			if(data[3] & 0x80) 
			{
				data[3]=~data[3]+1;
				sprintf(buffer,"-%d",data[3]);
			}
			else sprintf(buffer,"%d",data[3]);
			FTDI.printf("\r\nAlarme TL=%s°C",buffer);
			
	
	switch(data[4]>>5)
						{
							case 0b11 : FTDI.printf("\r\nMesure sur 12bits demandee.");
													Result = (data[1] << 8) + data[0];
													FTDI.printf("\r\nTemperature sur 12bits DS18B20 : %0.3f°C\r\n",(float)(Result*0.0625));
													return((float)(Result*0.0625));
													break;
							
							case 0b00 : FTDI.printf("\r\nMesure sur 9bits demandee.");
													Result = (data[1] << 8) + data[0];
													Result=Result >>3;
													FTDI.printf("\r\nTemperature sur 9bits DS18B20 : %0.3f°C\r\n",(float)(Result*0.5));
													return((float)(Result*0.5));
													break;
							
							case 0b01 : FTDI.printf("\r\nMesure sur 10bits demandee.");
													Result = (data[1] << 8) + data[0];
													Result=Result >>2;
													FTDI.printf("\r\nTemperature sur 10bits DS18B20 : %0.3f°C\r\n",(float)(Result*0.25));
													return((float)(Result*0.25));
													break;
							
							case 0b10 : FTDI.printf("\r\nMesure sur 11bits demandee.");
													Result = (data[1] << 8) + data[0];
													Result=Result >>1;
													FTDI.printf("\r\nTemperature DS18B20 : %0.3f°C\r\n",(float)(Result*0.125));
													return((float)(Result*0.125));
													break;
						}
						
		
	}

}

//------------------------------------------------------------------------------------
//Gestion de OSV3
//------------------------------------------------------------------------------------
//Fonction pour obtenir la configuration initiale des frequences de la Teensy 3.2
//Suite au mode PowerDown, la Teensy ne fonctionne qu'à 72MHz
	
	void Teensy32_Back2Speed_After_PowerDown(void)
{
	/* SIM->CLKDIV1: OUTDIV1=0,OUTDIV2=1,OUTDIV4=3 Set Prescalers 96MHz cpu, 48MHz bus, 24MHz flash*/
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) |  SIM_CLKDIV1_OUTDIV4(3);
  /* SIM->CLKDIV2: USBDIV=2, Divide 96MHz system clock for USB 48MHz */
  SIM->CLKDIV2 = SIM_CLKDIV2_USBDIV(1);  
  /* OSC0->CR: ERCLKEN=0,EREFSTEN=0,SC2P=1,SC4P=0,SC8P=1,SC16P=0 10pF loading capacitors for 16MHz system oscillator*/
  OSC0->CR = OSC_CR_SC8P_MASK | OSC_CR_SC2P_MASK;
  /* Switch to FBE Mode */
  /* MCG->C7: OSCSEL=0 */
  MCG->C7 = (uint8_t)0x00u;
  /* MCG->C2: LOCKRE0=0,RANGE0=2,HGO=0,EREFS=1,LP=0,IRCS=0 */
  MCG->C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS0_MASK;
  //MCG->C2 = (uint8_t)0x24u;
  /* MCG->C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3) | MCG_C1_IRCLKEN_MASK;
  /* MCG->C4: DMX32=0,DRST_DRS=0,FCTRIM=0,SCFTRIM=0 */  
  MCG->C4 &= (uint8_t)~(uint8_t)0xE0u; 
  /* MCG->C5: PLLCLKEN=0,PLLSTEN=0,PRDIV0=7 */
  MCG->C5 = MCG_C5_PRDIV0(7);
  /* MCG->C6: LOLIE=0,PLLS=0,CME=0,VDIV0=0 */
  MCG->C6 = (uint8_t)0x00u;
  while((MCG->S & MCG_S_OSCINIT0_MASK) == 0u) { } /* Check that the oscillator is running */
  while((MCG->S & 0x0Cu) != 0x08u) { } /* Wait until external reference clock is selected as MCG output */
  /* Switch to PBE Mode */
  /* MCG_C5: PLLCLKEN=0,PLLSTEN=0,PRDIV0=5 */
  MCG->C5 = MCG_C5_PRDIV0(3); // config PLL input for 16 MHz Crystal / 4 = 4 MHz
  /* MCG->C6: LOLIE=0,PLLS=1,CME=0,VDIV0=3 */
  MCG->C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0);// config PLL for 96 MHz output
  while((MCG->S & MCG_S_PLLST_MASK) == 0u) { } /* Wait until the source of the PLLS clock has switched to the PLL */
  while((MCG->S & MCG_S_LOCK0_MASK) == 0u) { } /* Wait until locked */
  /* Switch to PEE Mode */
  /* MCG->C1: CLKS=0,FRDIV=2,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_FRDIV(2) | MCG_C1_IRCLKEN_MASK;
  while((MCG->S & 0x0Cu) != 0x0Cu) { } /* Wait until output of the PLL is selected */
  while((MCG->S & MCG_S_LOCK0_MASK) == 0u) { } /* Wait until locked */
}

//Fonction pour OSV3

unsigned char OSV3_CALC_CRC(bool *InitFait)
{
	unsigned char CRC=0;
	
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	//Inclure les octets de l'indice 4 à 10. Ne pas inclure le CHECKSUM à l'indice 11
	
	OSV3_INIT_CRC(InitFait);
	
	CRC=OSV3_TAB_CRC_INIT[OSV3_TAB_DATA[3] & 0x0F];
	//ne sert à rien
	//CRC=CRC & 0xFF;
	
	for (int i=4 ; i<11 ; i++)
	{ 
		CRC=OSV3_TAB_CRC_INIT[CRC ^ OSV3_TAB_DATA[i]];
		//ne sert à rien
		//CRC=CRC & 0xFF;
	}
	//Permutation des NIBBLES
	return( ((CRC & 0x0F)<<4) | ((CRC & 0xF0)>>4) );
	
}

void OSV3_INIT_CRC(bool *InitFait)
{
	//A faire avant de calculer le CRC -> Fait lors du 1er calcul
	unsigned char CRC;
	
	if(!*InitFait) {
										for(int i=0 ; i<256 ; i++)
										{
											CRC=i;
											
											for(int j=0 ; j<8 ; j++)
											{
												CRC=(CRC<<1)^((CRC & 0x80) ? OSV3_CRC8_M433_DI : 0);
											}
											
											//ne sert à rien ici le & bit à bit
											//OSV3_TAB_CRC_INIT[i]=CRC & 0xFF; 
											OSV3_TAB_CRC_INIT[i]=CRC;
										}
										
										*InitFait=true;
								 }
	
}

unsigned char OSV3_CALC_CHECKSUM(void)
{
	unsigned char CheckSum;
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	
	CheckSum = ( OSV3_TAB_DATA[3] & 0x0F );

	//Inclure les nibbles de 4 à 10
	for(int i=4 ; i <= OSV3_TAILLE_DATA_CAPTEUR-3 ; i++)
	{
		CheckSum = CheckSum + (OSV3_TAB_DATA[i] & 0x0F) + ((OSV3_TAB_DATA[i]>>4) & 0x0F);
	}
	
	//Permutation des NIBBLES
	return( ((CheckSum & 0x0F)<<4) | ((CheckSum & 0xF0)>>4) );
	
}

void OSV3_CONSTRUIRE_TRAME(float Temp_f, int HumiHR)
{
	//Les nibbles sont envoyés LSB first
	
	//Preambule du protocole OSV3
	//24 bits à 1 -> 6 nibbles
	OSV3_TAB_DATA[0]=0xFF;
	OSV3_TAB_DATA[1]=0xFF;
	OSV3_TAB_DATA[2]=0xFF;
	//nibble de synchro -> 0101 -> LSB en 1er soit 0xA0
	OSV3_TAB_DATA[3]=0xA0;
	
	//Trame de données du capteur THGR810 -> payload
	//les nibbles 0..3 sont l'ID du capteur qui est unique pour chaque capteur ou commun pour
	//un groupe de capteur.
	//Ici ID du capteur est F824 dans l'ordre de reception
	OSV3_TAB_DATA[3]=OSV3_TAB_DATA[3] | 0x0F;
	OSV3_TAB_DATA[4]=0x82;
	OSV3_TAB_DATA[5]=0x40;
	
	//le nibble 4 pour le CANAL de 1 à 15  
	//Insertion du CANAL
	OSV3_TAB_DATA[5]=OSV3_TAB_DATA[5] | OSV3_CANAL_TEMP_HR;

	//Les nibbles 5..6 pour le code tournant dont la valeur est aleatoire
	//à chaque reset du capteur : exemple changement de piles.
	//OSV3_TAB_DATA[6]=OSV3_ROLLING_CODE();
	OSV3_TAB_DATA[6]=VAL_ROLLING_CODE;
	//Capteur avec bit d'etat de la batterie -> toujours à 0 pour batterie chargée
	//valeur à 1 lorsque la batterie est à changer
	//A changer par une variable pour evolution.
	OSV3_TAB_DATA[7]=0x80;
	
	//Les nibbles 8..[N-5] sont les données du capteur
	//Les nibbles 10..8 sont la temperature avec 1 LSB qui represente 0.1 °C
	//exemple : un float de 23.5°C est à transformer en entier de 235
	int temp=(int)(Temp_f*10);
	//Extraction de 5 de 23.5°C
	OSV3_TAB_DATA[7]=OSV3_TAB_DATA[7] | ( (abs(temp) % 10) & 0x0F );
	//Extraction de 3 de 23.5°C
	OSV3_TAB_DATA[8]=((abs(temp)/10) % 10) << 4;
	//Extraction de 2 de 23.5°C
	OSV3_TAB_DATA[8]=OSV3_TAB_DATA[8] | ((abs(temp)/100) & 0x0F);
	//Le nibble 11 represente le signe de la temperature -> une valeur differente de 0 est 
	//une temperature negative
	OSV3_TAB_DATA[9]=(Temp_f <0) ? 0x80 : 0;
	//Extraction de HD en %
	OSV3_TAB_DATA[9]=OSV3_TAB_DATA[9] | ((HumiHR % 10) & 0x0F);
	OSV3_TAB_DATA[10]=((HumiHR /10) % 10) <<4 ;
	//Placement du CHECKSUM
	//Le resultat de la somme sur 8 bits des nibbles 0..[N-5]
	//Le CHECKSUM est placé dans [N-3]..[N-4]
	OSV3_TAB_DATA[11]=OSV3_CALC_CHECKSUM();
	//Placement du CRC
	OSV3_TAB_DATA[12]=OSV3_CALC_CRC(&Fait_Init_TAB_CRC);
}

unsigned char OSV3_ROLLING_CODE(void)
{
	
	//Lecture d'une entrée analogique sur 16 bits sur la teensy 3.2
	//Ici l'entrée analogique est PTC0 soit A1.
	unsigned short VCAN=AIN.read_u16();
	//Initialisation du generateur aléatoire
	srand(VCAN);
	
	//Nombre aleatoire entre 1 et 254.
	return( (rand() % 253) +1 );
	
}

void OSV3_MANCHESTER_ENCODE(unsigned char Octet_Encode, bool Fin)
{
	unsigned char MASK=0x10;
	//Timer pour la gestion du temps.
	//Lecture de la durée
	static int TimeBase=TimeManchester.read_us();
	//Bouleen pour tester le dernier bit
	static bool LastBit=false;
	
	//OSV3 emet à la frequence de 1024Hz ou 1024bit/s
	//Prevoir un ajustement en fonction du temp de traitement par le uC
	//En mesure : 1020Hz et donc 490us
	const unsigned int DureeDesire=490;
	//Mode PowerDown -> plus lent et on mesure 764,5Hz au lieu de 1020Hz
	//Au reveil, la Teensy est à 72MHz au lieu de 96MHz -> 96/72=1.33
	//const unsigned int DureeDesire=367;
	//Valeur ajustemet du au temps de traitement
	//Reduction de DureeDesiree
	const unsigned int ReduireDe=32;
	//Mode PowerDown -> plus lent et on mesure 764,5Hz au lieu de 1020Hz
	//const unsigned int ReduireDe=24;
	
	//Durée de decalage entre chaque bit pour 8 octets
	//static unsigned int tab_Duree[]={10,20,30,40,50,60,70,80};
	
	//Les bits sont transmis de 4 à 7 puis de 0 à 3
	for(int i=0 ; i<8 ; i++)
	{
		
		TimeBase=TimeBase+DureeDesire;
		
		unsigned int CalculRetard=TimeBase-TimeManchester.read_us();
		
		//PC.printf("\r\n\n CalculRetard=%d | TimeBase=%d",CalculRetard,TimeBase);
		
		
		if(CalculRetard > 2*DureeDesire)  {
																				//Retard trop grand indique un break entre la transmission : reset de TimeBase
																				TimeBase=TimeManchester.read_us();
																				//PC.printf("\r\n\n CalculRetard > 2*DureeDesire ->TimeBase=%d",TimeBase);
																			}
		else {
						if(CalculRetard > 0) wait_us(CalculRetard);
						//PC.printf("\r\n\n CalculRetard > 0 ->TimeBase=%d",CalculRetard);
				 }
		
		
		//wait_us(tab_Duree[i]*10);
		
		if((Octet_Encode & MASK)==0) {
																		//Un 0 est représenté par une transition de 0 à 1 dans le signal RF
																		//Mise à 0 de la broche
																		Data_433=0;
																		wait_us(DureeDesire-ReduireDe);
																		Data_433=1;
			
																		//Test si dernier bit
																		//Dans ce cas pas de retard long apres la transition de 0 à 1 
																		//pour indiquer que plus de donnée à suivre
																		if(Fin) wait_us(DureeDesire);
																		LastBit=false;
			
																 }
		else {
							Data_433=1;
							wait_us(DureeDesire-ReduireDe);
							Data_433=0;
							LastBit=true;
				 }
			
	  if(MASK==0x80) MASK=0x01;
		else MASK=MASK<<1;
				
		TimeBase=TimeBase+DureeDesire;
	}
	
	
}

void OSV3_MANCHESTER_SEND_DATA(void)
{
	//Est-ce necessaire de faire un reset du Timer ?
	TimeManchester.reset();
	TimeManchester.start();
	
	//Ajouter mise sous tension du module TX en 433MHz
	//Prevoir une tempo de 60ms
	
	//Mise à 0 de la broche
	//Data_433=0;
	
	for(int i=0 ; i<OSV3_TAILLE_DATA_CAPTEUR ; i++)
	{
		OSV3_MANCHESTER_ENCODE(OSV3_TAB_DATA[i], i+1==OSV3_TAILLE_DATA_CAPTEUR);

	}
	
	//Ajouter mise hors tension du module TX en 433MHz
	//Mise à 0 de la broche
	//Data_433=0;
	
	//Arret du Timer
	//A tester si necessaire suite à PowerDown de la Teensy 3.2
	TimeManchester.stop();
	
}

/////////////////////////////////////////////////////////////////////////////
// Gestion du VEML6075 pour UVA - UVB et UVC en I2C0
/////////////////////////////////////////////////////////////////////////////
int VEML6075_GetID(int ADDR_VEML6075, bool *TimeOut_Ended)
{
	bool FLAG=false;
	char temp[]={VEML6075_REG_DEVID,0};
				
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	t.start();
				
	float debut = t.read();
			
  do
   {

          if (Capt_I2C0.write(ADDR_VEML6075,NULL,0))
             {
              FLAG=true;
							 
							if((t.read()-debut)>I2C_TIMEOUT) {
																								t.stop();
																								//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																								*TimeOut_Ended=true;
																								break;
																							 }
             }
          else
             {
              FLAG=false;
							t.stop();
							 
              Capt_I2C0.write(ADDR_VEML6075,temp,1,true);
							Capt_I2C0.read(ADDR_VEML6075,temp,2);
              //Lecture du LSB en 1er puis MSB ensuite
							//(MSB << 8) | LSB
							return((temp[1]<<8) | temp[0]);
             }
    }
   while(FLAG);
		
}

int VEML6075_GetRauUVABD(int ADDR_VEML6075, char SelectABD, bool *TimeOut_Ended)
{
	bool FLAG=false;
	char temp[]={SelectABD,0};
				
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	t.start();
				
	float debut = t.read();
			
  do
   {

          if (Capt_I2C0.write(ADDR_VEML6075,NULL,0))
             {
              FLAG=true;
							 
							if((t.read()-debut)>I2C_TIMEOUT) {
																								t.stop();
																								//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																								*TimeOut_Ended=true;
																								break;
																							 }
             }
          else
             {
              FLAG=false;
							t.stop();
							 
              Capt_I2C0.write(ADDR_VEML6075,temp,1,true);
							Capt_I2C0.read(ADDR_VEML6075,temp,2);
              //Lecture du LSB en 1er puis MSB ensuite
							//(MSB << 8) | LSB
							return((temp[1]<<8) | temp[0]);
             }
    }
   while(FLAG);
		
}

void VEML6075_WriteConfig(int ADDR_VEML6075, char Config, bool *TimeOut_Ended)
{
	bool FLAG=false;
	char temp[]={VEML6075_REG_UV_CONF, Config, 0};
				
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	t.start();
				
	float debut = t.read();
			
  do
   {

          if (Capt_I2C0.write(ADDR_VEML6075,NULL,0))
             {
              FLAG=true;
							 
							if((t.read()-debut)>I2C_TIMEOUT) {
																								t.stop();
																								//variable globale bool pour le TIMEOUT en I2C-> mettre à true
																								*TimeOut_Ended=true;
																								break;
																							 }
             }
          else
             {
              FLAG=false;
							t.stop();
							 
              Capt_I2C0.write(ADDR_VEML6075,temp,3);
				   			
             }
    }
   while(FLAG);
		
}

float VEML6075_GetUVA(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uva, unsigned int raw_dark)
{
	float comp_vis = raw_vis - raw_dark;
	float comp_ir = raw_ir - raw_dark;
	float comp_uva = raw_uva - raw_dark;
	
	comp_uva = comp_uva - (VEML6075_UVI_UVA_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVA_IR_COEFF * comp_ir);
	
	return(comp_uva);
	
}



float VEML6075_GetUVB(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uvb, unsigned int raw_dark)
{
	float comp_vis = raw_vis - raw_dark;
	float comp_ir = raw_ir - raw_dark;
	float comp_uvb = raw_uvb - raw_dark;
	
	//comp_uvb -= (VEML6075_UVI_UVB_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVB_IR_COEFF * comp_ir);
	comp_uvb = comp_uvb - (VEML6075_UVI_UVB_VIS_COEFF * comp_vis) - (VEML6075_UVI_UVB_IR_COEFF * comp_ir);
	
	return(comp_uvb);
	
}

float VEML6075_GetUVIndex(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uva,unsigned int raw_uvb, unsigned int raw_dark)
{
	float uva_pond = VEML6075_GetUVA(raw_vis, raw_ir, raw_uva, raw_dark) * VEML6075_UVI_UVA_RESPONSE;
	float uvb_pond = VEML6075_GetUVB(raw_vis, raw_ir, raw_uvb, raw_dark) * VEML6075_UVI_UVB_RESPONSE;
	
	return((uva_pond + uvb_pond) / 2.0);
	
}



/*

//Codage des fonctions d'IT pour LCD NEXTION
//Aucun caractere de perdu
void LCD_NEXTION_IT_RX(void)
{

	do
	{
		data_RX_LCD_Nextion[CPT_RX_LCD_Nextion++]=LCD_HMI.getc();
	}
	while(LCD_HMI.readable());
	
	data_RX_LCD_Nextion[CPT_RX_LCD_Nextion]='\0';

	
	IT_RX_LCD_Nextion_OK=true;
	
}

*/

//Copie du tableau data_RX dans le tableau Copie_data_RX
//Le compilateur ne supporte pas de travailler directement avec un type volatile dans la fonction Extraction_Data
void Copie_de_Tableau(volatile char *s, char *d)
{
	while((*d++=*s++)!='\0');
}	



/////////////////////////////////////////////////////////////////////////////
// Gestion du LSM6DS33TR
/////////////////////////////////////////////////////////////////////////////
unsigned char LSM_spi_read(unsigned char target)
{
	unsigned char out1,out2;
	
	LSM_CS=0;
	
	out1=LSM.write(target | LSM_READ);
	out2=LSM.write(0x00);
	
	LSM_CS=1;
	
	return(out2);
}

short LSM_spi_read_16bits(unsigned char target)
{
	unsigned char out1, out2;
	short out4;
	
	LSM_CS=0;
	
	LSM.write(target | LSM_READ);
	out1=LSM.write(0x00);
	out2=LSM.write(0x00);
	
	LSM_CS=1;
	
	out4 =  (short)(out1 | ((unsigned short)out2 << 8));
	
	//FTDI.printf("\r\nout4=%#x|%#x|%#x",out4,out2,out1);
	
	return(out4);
	
}

void LSM_spi_write(unsigned char target, unsigned char data2write)
{
	LSM_CS=0;
	
	LSM.write(target & LSM_WRITE);
	LSM.write(data2write);
	
	LSM_CS=1;
	
}

float CalcAcc(int acc)
{
	//accelrange : 2 ou 4 ou 8 ou 16
	float output = (float)acc*0.061*(16>>1)/1000;
	return output;
}

float CalcGyro(int gyro)
{
	//gyrorange : 125 ou 245 ou 500 ou 1000 ou 2000
	float output = (float)gyro*4.375*(1000/125)/1000;
	return output;
}



//Lecture des fonctionalité page 23 de la Doc (§5)

void LSM_spi_init(void)
{
	//page 17 de l'AN - attendre 20ms apres la mise sous tension. Acc et Gyro sont ensuite en mode Power_Down automatiquement.
	unsigned char adress, target, writeout, result;
	
	FTDI.printf("\r\nWho am I : %#x",WHO_I_AM);
	target=0x0F;
	result=LSM_spi_read(target);
	if(result!=WHO_I_AM) FTDI.printf("\r\nErreur LSM6DS33 non indentifie avec %#x",result);
	else FTDI.printf("\r\nLSM6DS33 indentifie par reponse %#x",result);
	
	
	FTDI.printf("\r\nCTRL3_C->IF_INC=1");
	//Incrementation des adresses lors d'acces multiple en I2C et SPI - 1 valeur par defaut de IF_INC
	//page 36 de l'AN -> polarite de l'IT ici passage de 0 à 1 par bit H_ACTIVE
	//page 49 de la doc -> BDU=1 recommandé (bit 6) 
	target=0x12;
	result=LSM_spi_read(target);
	writeout=result | 0b01000100;
	LSM_spi_write(target,writeout);
	
	FTDI.printf("\r\nFIFO_CTRL3->No decimation pour Gyro et Acc:");
	//no decimation pour acc et gyro - gyro pas dans la FIFO
	target=0x08;
	result=LSM_spi_read(target);
	writeout=result | 0b00000001;
	LSM_spi_write(target,writeout);
	
	FTDI.printf("\r\nCTRL1_XL->Accel");
	//0110 0100, odr=416Hz , +/-16g, Filter=400Hz
	//0100 0110, odr=104Hz , +/-16g, Filter=100Hz 
	//0100 01 00, odr=104Hz , +/-16g, Filter=400Hz<-
	target=0x10;
	//writeout=0b01100100;
	//writeout=0b01000110;
	writeout=0b01000100;
	LSM_spi_write(target,writeout);
	
	FTDI.printf("\r\nCTRL6_C->Accel en XL_HM_MODE");
	//Mise en service High-performance operating mode par XL_HM_MODE
	//mettre 0 dans XL_HM_MODE soit bit 4
	target=0x15;
	result=LSM_spi_read(target);
	writeout=result & 0b11100000;
	LSM_spi_write(target,writeout);
	
	
	FTDI.printf("\r\nCTRL2_G->Gyro");
	//GYro en power down
	//0110 1000, odr=416Hz , 1000dps
	//0100 1000, odr=104Hz , 1000dps
	//0000 00 0 0
	target=0x11;
	writeout=0b00000000;
	//writeout=0b01001000;
	LSM_spi_write(target,writeout);
	
	FTDI.printf("\r\nFIFO_CTRL1-> ");
	//Changement de la taille de la FIFO par FTH[7:0]
	//Test : ne pas activer pour WAKE_UP et mode bypass to continuous : page 86 de l'AN
	//Valeur par defaut -> 00000000
	//Test : activation de la limitation en WAKE_UP et FIFO en bypass et continuous
	target=0x06;
	writeout=(TAILLE_FIFO) & 0xFF;
	//writeout=0b00000000;
	LSM_spi_write(target,writeout);
	
	FTDI.printf("\r\nFIFO_CTRL2-> ");
	//Changement de la taille de la FIFO par FTH[11:8] - les 4 bits de poids failble
	//Ne pas toucher au 4 bits de poids fort sauf laisser à 0 les bits 4 et 5
	//Ne pas changer la taille de la FIFO pour le mode WAKE_UP et FIFO en bypass puis continuous.
	//Test : activation de la limitation en WAKE_UP et FIFO en bypass et continuous
	target=0x07;
	result=LSM_spi_read(target);
	writeout=(result & 0b11000000) | ((TAILLE_FIFO)>>8);
	//writeout=(result & 0b11000000);
	LSM_spi_write(target,writeout);
	
	FTDI.printf("\r\nCTRL4_C-> BW-I2C-FIFO");
	//bandwidth determined by setting BW_XL[1:0]
	//Taille FIFO limitée par STOP_ON_FTH à 1
	//I2C desactive - limitation de la taille de FIFO - Gyro Sleep Mode activé
	//Ne pas activer XL_BW_SCAL_ODR pour le WAKE_UP et FIFO en bypass puis continuous.
	//1 1 0 0 0 1 0 0
	//Test : activation de la limitation en WAKE_UP et FIFO en bypass et continuous
	//1 1 0 0 0 1 0 1
	target=0x13;
	result=LSM_spi_read(target);
	writeout=result | 0b11000101;
	//writeout=result | 0b11000100;
	LSM_spi_write(target,writeout);
	
	//Activation des 3 axes de ACC
	FTDI.printf("\r\nCTRL9_XL-> Zen_XL Yen_XL Xen_XL");
	//Activation des 3 axes de ACC
	//Mise à 1 des bits 3,4 et 5
	//Test la place est reservée pour X, Y et Z meme si on n'active pas Y et Z -> toujours laisser les 3 activés
	//00ZYX000
	target=0x18;
	result=LSM_spi_read(target);
	writeout=(result | 0b00111000) & 0b00111000;
	LSM_spi_write(target,writeout);
	
	
	//Configuration de la FIFO en : FIFO mode -> stockage dans la FIFO jusqu'à ce qu'elle soit pleine -> Continuous mode et taille FIFO specifiee
	//odr FIFO plus eleve que odr ACC par securité (au moins x3)
	//Test avec ACC en X, Y et Z
	FTDI.printf("\r\nFIFO_CTRL5-> ");
	//odr(0110)=416Hz et FIFO mode en -> Continuous mode (110)
	//wake_up avec continuous mode puis FIFO mode (011)
	//Bypass puis continuous mode (100) -> pas d'information sur la limitation de la FIFO -> pas d'IT apres test -> suppression de la limitation de la FIFO
	//0 0110 110
	//0 0110 011
	//0 0110 100
	//Test : activation de la limitation en WAKE_UP et FIFO en bypass et continuous
	//0 0110 100
	target=0x0A;
	//writeout=0b00110100;
	writeout=0b00110100;
	LSM_spi_write(target,writeout);
	
	
	//Configuration INT1
	FTDI.printf("\r\nINT1_CTRL->");
	//Activation de l'IT vers INT1
	//INT1_FTH seul -> bit 3
	//INT1 pour FIFO FULL -> bit 5
	//Pour WAKE_UP activation de FIFO Full flage -> page 86 et 87 de l'AN
	//FIFO Full Flag dirige vers INT1
	//Test : activation de la limitation en WAKE_UP et FIFO en bypass et continuous
	//Activation de INT1_FTH -> bit 3
	target=0x0D;
	writeout=0b00001000;
	//writeout=0b00100000;
	//writeout=0b00100000;
	LSM_spi_write(target,writeout);
	
	//Configuration du WAKE_UP
	//page 40 de l'AN
	//page 36 de l'AN pour memoriser l'evenement declenchent -> mettre le bit LIR à 1 (bit 0)
	//Le bit LIR est lis à 0 par lecture du registre d'etat des IT
	FTDI.printf("\r\nTAP_CFG->");
	//Seulement TAP sur Z
	//Pour WAKE UP -> page 40,41 de l'AN -> 0x00 dans TAP_CFG -> ne fonctionne que si LIR =1 en mode bypass puis continuous.
	target=0x58;
	//writeout=0b00000010;
	writeout=0b00000001;
	LSM_spi_write(target,writeout);
	
	//Configuration du WAKE_UP_DUR
	FTDI.printf("\r\nWAKE_UP_DUR->");
	//aucune duree
	//aucune durée pour le mode WAKE_UP -> page 41 de l'AN
	//0 00 0 0000
	//page 66 de la doc -> 1 LSB à pour durée 1 ODR_time
	target=0x5C;
	writeout=0b00000000;
	LSM_spi_write(target,writeout);
	
	//Configuration du WAKE_UP_THS
	FTDI.printf("\r\nWAKE_UP_THS->");
	//2*FS_XL/2^6->0.5g pour +-16g
	//En mode WAKE_UP -> configuration du seuil
	//0 0 000010
	//Valeur de seuil comme indiquée dans l'AN page 41
	target=0x5B;
	writeout=0b00000010;
	LSM_spi_write(target,writeout);
	
	//Lecture WAKE_UP_SRC
	//adresse registre 0x1B
	//page 55 de la doc.
	//le bit WU_IA indique la detection d'un evenement de type WAKE up
	//X_WU et Y_WU et Z_WU indique l'evenement.
	
	//IT generée sur INT2 par le WAKE_UP
	//Configuration de MD2_CFG
	FTDI.printf("\r\nMD2_CFG->");
	//routage sur INT2 -> mise à 1 du bit 5
	//Test : WAKE_UP avec limitation de la taille de la FIFO
	target=0x5F;
	writeout=0b00100000;
	LSM_spi_write(target,writeout);
	
}

short LSM_Read_FIFO(void)
{
	unsigned char Read_Octet;
	unsigned short tempFIFO=0;
	
	Read_Octet=LSM_spi_read(LSM_FIFO_Data_L);
	tempFIFO=Read_Octet;
	Read_Octet=LSM_spi_read(LSM_FIFO_Data_H);
	tempFIFO=tempFIFO | (Read_Octet<<8);
	
	return(tempFIFO);

}

unsigned short LSM_FIFO_Status(void)
{
	unsigned char Read_Octet;
	unsigned short tempFIFO=0;
	
	Read_Octet=LSM_spi_read(LSM_FIFO_STATUS1);
	tempFIFO=Read_Octet;
	Read_Octet=LSM_spi_read(LSM_FIFO_STATUS2);
	tempFIFO=tempFIFO | (Read_Octet<<8);
	
	return(tempFIFO);

}

unsigned short LSM_FIFO_Pattern(void)
{
	unsigned char Read_Octet;
	unsigned short tempFIFO=0;
	
	Read_Octet=LSM_spi_read(LSM_FIFO_STATUS3);
	tempFIFO=Read_Octet;
	Read_Octet=LSM_spi_read(LSM_FIFO_STATUS4);
	tempFIFO=tempFIFO | (Read_Octet<<8);
	
	return(tempFIFO & 0x03FF);

}




//page 81 de lAN explication sur les differents mode de la FIFO
//page 85 de l'AN le mode bypass to continous
//page 89 de l'AN explication sur le rangement dans la FIFO et identification par Pattern.

void LSM_Vidage_FIFO(void)
{
	static short AccMesX[TAILLE_FIFO_MAX]={0};
	static short AccMesY[TAILLE_FIFO_MAX]={0};
	static short AccMesZ[TAILLE_FIFO_MAX]={0};
	
	int i=0;
	unsigned short Pattern;
	
	while((LSM_FIFO_Status() & 0x1000)==0)
	{
		//Lecture du pattern et affichage
		Pattern=LSM_FIFO_Pattern();
		//FTDI.printf("P=%d|",Pattern);
		
		//Test si 1 seul axe en X permet d'avoir plus d'echantillons en FIFO
		//AccMesX[i++]=LSM_Read_FIFO();
		
		
		switch(Pattern)
		{
			case 0 :	AccMesX[i++]=LSM_Read_FIFO();
								break;
			case 1 :	AccMesY[i++]=LSM_Read_FIFO();
								break;
			case 2 :	AccMesZ[i++]=LSM_Read_FIFO();
								break;
		}
	}
	
	FTDI.printf("\r\ni=%d\r\n",i);
	
	for (i=0; i<TAILLE_FIFO ; i++)
	{
		
		//FTDI.printf("%d|\r\n",AccMesX[i]);
		//Affichage par Teraterm
		FTDI.printf("%d|%d|%d\r\n",AccMesX[i],AccMesY[i],AccMesZ[i]);
		
		//Affichage Arduino
		/*
		FTDI.printf("%d,",AccMesX[i]);
		FTDI.printf("%d,",AccMesY[i]);
		FTDI.printf("%d\n",AccMesZ[i]);
		*/
		
	}
}

void LSM_bypass_FIFO(void)
{
	unsigned char target, writeout, result;
	//Configuration dela FIFO en mode bypass -> la FIFO est vidée et plus utilisée
	//Pour l'utiliser de nouveau, changer le mode de la <FIFO
	//FTDI.printf("\r\nFIFO_CTRL5->");
	//
	//
	target=0x0A;
	result=LSM_spi_read(target);
	writeout=(result & 0b11111000);
	LSM_spi_write(target,writeout);
	
}

void LSM_ISR_INT2(void)
{
	//COM_SD_POWER=COM_SD_POWER^1;
	Flag_Wake=false;
}


void LSM_ISR_INT1(void)
{
	COM_SD_POWER=COM_SD_POWER^1;
	Flag_Fifo=false;
}

/////////////////////////////////////////////////////////////////////////////
// Gestion du HDC1080
/////////////////////////////////////////////////////////////////////////////
bool HDC1080_Test_ID(int addr, bool *TimeOut_Ended)
{
	bool FLAG=false;
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	
	
	unsigned short ID;
	//Reservation 2 octets pour lecture de l'ID -> 0x1050
	char Tab[]={Reg_ID_Device,0};
	
	t.start();
	float debut=t.read();
	
	do
	{
		if(HDC1080.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
				
																			 }
		}
			else
			{
				FLAG=false;
				t.stop();
				//Lancement I2C - ecriture de Reg_ID_Device puis STOP
				HDC1080.write(addr,Tab,1);
				//Lecture du resultat - 2 octets
				HDC1080.read(addr,Tab,2);
				//Association des 2 octets - MSB LSB
				ID=(Tab[0]<<8) | Tab[1];
	
				//Comparaison avec le resultat attendu
				if(ID==VAL_ID_Device) return true;
				else return false;
			}
	}
	while(FLAG);

}


unsigned short HDC1080_Read_Temp(int addr, unsigned char Precision, bool *TimeOut_Ended)
{
	bool FLAG=false;
	*TimeOut_Ended=false;
	//Variable pour analyse de l'execution de write et read en I2C
	int res;
	
	Timer t;
	t.reset();
	
	unsigned short Temp;
	//Reservation 3 octets pour lecture de Temp et configuration de Reg_Config
	char Tab[]={Reg_Config,0,0};
	
	//Configuration pour Temp seule et precision sur 14bits ou 11bits
	//Cf. page 15 de la doc.
	Tab[1]=((Precision==14) ? 0b00000000 : 0b00000100);
	//Tab[2] toujours à 0 - Cf. page 15 §8.6.3 de la doc.
	
	t.start();
	float debut=t.read();
	
	do
	{
		if(HDC1080.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				//Ecriture configuration puis STOP
				res=HDC1080.write(addr,Tab,3);
				if(res) return(0);
	
				//lancement de l'acquisition de Temp puis attente Tempo associée à 14bits ou 11bits
				Tab[0]=Reg_Temp;
				res=HDC1080.write(addr,Tab,1);
				if(res) return(0);
				
				//Attente tempo en fonction de 14bits ou 11bits
				//x3 car page 5 de la doc. note (7)
				if(Precision==14) wait_us((int)(Temp_14bits*1000*3));
				else wait_us((int)(Temp_11bits*1000*3));
	
	
				//Test Tempo de 20ms - car valeur donnée dans video
				//wait_ms(20);
				//Avec cette valeur cela fonctionne - page 5 de la doc, la valeur de tempo est theorique et non verifiee experimentalement.
				
				//Lecture du resultat brut - 2 octets puis STOP
				res=HDC1080.read(addr,Tab,2);
				if(res) return(0);
	
				//Mise en forme du resultat
				Temp=(Tab[0]<<8) | Tab[1];
	
				return(Temp);
			}
		
	}
	while(FLAG);
}

unsigned short HDC1080_Read_HR(int addr, unsigned char Precision, bool *TimeOut_Ended)
{
	
	bool FLAG=false;
	*TimeOut_Ended=false;
	//Variable pour analyse de l'execution de write et read en I2C
	int res;
	
	Timer t;
	t.reset();
	
	
	unsigned short HR;
	//Reservation 3 octets pour lecture de HR et configuration de Reg_Config
	char Tab[]={Reg_Config,0,0};
	
	//Configuration pour HR seule et precision
	//Cf. page 15 de la doc.
	//14bits ou 11bits ou 8bits
	switch(Precision)
	{
		case 14 : Tab[1]=0b00000000;
							break;
		case 11 : Tab[1]=0b00000001;
							break;
		case 8 	: Tab[1]=0b00000010;
							break;
	}
	
	//Tab[2] toujours à 0 - Cf. page 15 §8.6.3 de la doc.
	
	t.start();
	float debut=t.read();
	
	do
	{
		if(HDC1080.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				//Ecriture configuration puis STOP
				res=HDC1080.write(addr,Tab,3);
				if(res) return(0);
	
				//lancement de l'acquisition de HR puis attente Tempo associée à 14bits ou 11bits ou 8bits
				Tab[0]=Reg_HR;
				res=HDC1080.write(addr,Tab,1);
				if(res) return(0);
			
				//Attente tempo en fonction de 14bits ou 11bits ou 8bits
				//x3 car page 5 de la doc. note (7)
				switch(Precision)
						{
							case 14 : wait_us((int)(RH_14bits*1000*3));
												break;
							case 11 : wait_us((int)(RH_11bits*1000*3));
												break;
							case 8 	: wait_us((int)(RH_8bits*1000*3));
												break;
						}
								
				//Lecture du resultat brut - 2 octets puis STOP
				res=HDC1080.read(addr,Tab,2);
				if(res) return(0);
	
				//Mise en forme du resultat
				HR=(Tab[0]<<8) | Tab[1];
	
				return(HR);

			}
	}
  while(FLAG);	

}

bool HDC1080_Read_Temp_HR(int addr, unsigned char PrecisionTemp, unsigned char PrecisionHR, unsigned short *Temp, unsigned short *HR, bool *TimeOut_Ended)
{
	
	bool FLAG=false;
	*TimeOut_Ended=false;
	//Variable pour analyse de l'execution de write et read en I2C
	int res;
	
	Timer t;
	t.reset();
	
	unsigned short myHR,myTemp;
	//Reservation 4 octets pour lecture de Temp et HR et configuration de Reg_Config
	char Tab[]={Reg_Config,0,0,0};
	
	//Configuration pour HR et Temp en fonction de la precision
	//Cf. page 15 de la doc.
	//14bits ou 11bits ou 8bits pour HR
	switch(PrecisionHR)
	{
		case 14 : Tab[1]=Tab[1] | 0b00010000;
							break;
		case 11 : Tab[1]=Tab[1] | 0b00010001;
							break;
		case 8 	: Tab[1]=Tab[1] | 0b00010010;
							break;
	}
	
	//14bits ou 11bits pour Temp
	switch(PrecisionTemp)
	{
		case 14 : Tab[1]=Tab[1] | 0b00010000;
							break;
		case 11 : Tab[1]=Tab[1] | 0b00010100;
							break;
		
	}
	
	//Tab[2] toujours à 0 - Cf. page 15 §8.6.3 de la doc.
	
	t.start();
	float debut=t.read();
	
	do
	{
		if(HDC1080.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				//Ecriture configuration puis STOP
				res=HDC1080.write(addr,Tab,3);
				if(res) return(false);
	
				//lancement de l'acquisition de Temp puis attente Tempo associée à 14bits ou 11bits
				Tab[0]=Reg_Temp;
				res=HDC1080.write(addr,Tab,1);
				if(res) return(false);
				
				
				
				//Attente tempo en fonction de 14bits ou 11bits ou 8bits
				//x3 car page 5 de la doc. note (7)
				switch(PrecisionHR)
					{
						case 14 : wait_us((int)(RH_14bits*1000*3));
											break;
						case 11 : wait_us((int)(RH_11bits*1000*3));
											break;
						default : wait_us((int)(Temp_11bits*1000*3));
											break;
					}
					
					
				
								
				//Lecture du resultat brut - 4 octets puis STOP
				res=HDC1080.read(addr,Tab,4);
				if(res) return(false);
	
				//Mise en forme du resultat
				myTemp=(Tab[0]<<8) | Tab[1];
				myHR=(Tab[2]<<8) | Tab[3];
	
				//Sortie du resultat
				*Temp=myTemp;
				*HR=myHR;
								
			
			}
		
	}
	while(FLAG);
	
	return (true);

}

float Calc_Temp(unsigned short Temp)
{
	return(((float)Temp / pow(2.0,16.0))*165.0-40.0);
}

float Calc_HR(unsigned short HR)
{
	return(((float)HR / pow(2.0,16.0))*100.0);
}

/////////////////////////////////////////////////////////////////////////////
// Codage des fonctions pour la gestion du BMP280
/////////////////////////////////////////////////////////////////////////////

//Autre solution de codage du BMP280
//https://github.com/BoschSensortec/BMP280_driver
//https://developer-sjc-indigo-border.mbed.org/teams/MtM/code/BMP280/docs/tip/BMP280_8cpp_source.html

bool BMP280_Test_Presence(int addr, bool *TimeOut_Ended)
{
	bool FLAG=false;
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	
	t.start();
	float debut=t.read();
	
	char data=BMP280_REG_ID;
	
	do
	{
		if(Capt_I2C1.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				Capt_I2C1.write(addr, &data, 1, true);
				Capt_I2C1.read(addr,&data,1);
	
				FTDI.printf("\r\nLecture de Chip_ID=%#x ",data);
	
				if(data != BMP280_CHIPID) return false;
				else return true;
			}
	}
	while(FLAG);
}

bool BMP280_Fin_Mesure(int addr, bool *TimeOut_Ended)
{
	bool FLAG=false;
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	
	t.start();
	float debut=t.read();
	
	char data=BMP280_REG_STATUS;
	
	do
	{
		if(Capt_I2C1.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				Capt_I2C1.write(addr, &data, 1, true);
				Capt_I2C1.read(addr,&data,1);
	
				//FTDI.printf("\r\nLecture du registre etat=%#x ",data);
	
				if(data & (1<<3)) return false;
				else return true;
			}
	}
	while(FLAG);
}

void BMP280_write(int addr, char reg, char ctrl, bool *TimeOut_Ended)
{
	bool FLAG=false;
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	
	t.start();
	float debut=t.read();
	
	char data[2] = {reg, ctrl};
	
	do
	{
		if(Capt_I2C1.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				Capt_I2C1.write(addr, data, 2);
			}
	}		
	while(FLAG);
}
 
void BMP280_read(int addr, char reg, char *data, int length, bool *TimeOut_Ended) 
{
	bool FLAG=false;
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	
	t.start();
	float debut=t.read();
	
	do
	{
		if(Capt_I2C1.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				Capt_I2C1.write(addr, &reg, 1, true);
				Capt_I2C1.read(addr, data, length);
			}
	}		
	while(FLAG);
}



int BMP280_init(int addr, T_BMP280 *BMP280_Data ,char ctrl_meas , char config, bool *TimeOut_Ended)
{
	
	bool FLAG=false;
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	
	t.start();
	float debut=t.read();
	
	do
	{
		if(Capt_I2C1.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				char reg_ctrl, reg_config;
				reg_ctrl = BMP280_REG_CTRL_MEAS;
				reg_config = BMP280_REG_CONFIG;
	
				char begin = BMP280_REG_CALIBRATION;
				char bf[24];
    
				BMP280_read(addr, begin, bf, 24, TimeOut_Ended);
	
				(*BMP280_Data).dig_T1 = (bf[1]<<8) | bf[0];
				(*BMP280_Data).dig_T2 = (bf[3]<<8) | bf[2];
				(*BMP280_Data).dig_T3 = (bf[5]<<8) | bf[4];
				(*BMP280_Data).dig_P1 = (bf[7]<<8) | bf[6];
				(*BMP280_Data).dig_P2 = (bf[9]<<8) | bf[8];
				(*BMP280_Data).dig_P3 = (bf[11]<<8) | bf[10];
				(*BMP280_Data).dig_P4 = (bf[13]<<8) | bf[12];
				(*BMP280_Data).dig_P5 = (bf[15]<<8) | bf[14];
				(*BMP280_Data).dig_P6 = (bf[17]<<8) | bf[16];
				(*BMP280_Data).dig_P7 = (bf[19]<<8) | bf[18];
				(*BMP280_Data).dig_P8 = (bf[21]<<8) | bf[20];
				(*BMP280_Data).dig_P9 = (bf[23]<<8) | bf[22];
    
				BMP280_write(addr, reg_ctrl,   ctrl_meas, TimeOut_Ended);
				BMP280_write(addr, reg_config, config, TimeOut_Ended);
    
				//    FTDI.printf("t1:%d, t2:%d, t3:%d\r\n", dig_T1, dig_T2, dig_T3);
				//    FTDI.printf("p1:%d, p2:%d, p3:%d, p4:%d, p5:%d, p6:%d, p7:%d, p8:%d, p9:%d\r\n", dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
    
				return 0;
				
			}
	}		
	while(FLAG);
    
    
}
 
int BMP280_readData(int addr,T_BMP280 BMP280_Data , float *tempC, float *pressPa, bool *TimeOut_Ended)
{
    
    int var1,var2,T;
    int t_fine;
    char read_reg = BMP280_REG_PRESS;
    char rx[6];
    int adc_T,adc_P;
    
    BMP280_read(addr, read_reg, rx, 6, TimeOut_Ended);
    
    adc_T = ((rx[3]<<12)|(rx[4]<<4)|rx[5]>>4);
    adc_P = ((rx[0]<<12)|(rx[1]<<4)|rx[2]>>4);
//    FTDI.printf("adc_T %d, adc_p %d\r\n", adc_T, adc_P);
    
    var1=((((adc_T>>3)-((int)(BMP280_Data).dig_T1<<1)))*((int)(BMP280_Data).dig_T2))>>11;
    var2=(((((adc_T>>4)-((int)(BMP280_Data).dig_T1))*((adc_T>>4)-((int)(BMP280_Data).dig_T1)))>>12)*((int)(BMP280_Data).dig_T3))>>14;
    t_fine=var1+var2;
    T=(t_fine*5+128)>>8;
    *tempC = (float)(T/100.0);
//    FTDI.printf("BMP280 T %d\r\n", T);
 
		int64_t var1_p, var2_p, press;
 
    var1_p = ((int64_t)t_fine ) - 128000;
    var2_p = var1_p * var1_p * (int64_t)((BMP280_Data).dig_P6);
    var2_p = var2_p + ((var1_p * (int64_t)((BMP280_Data).dig_P5))<<17);
    var2_p = var2_p + (((int64_t)(BMP280_Data).dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)((BMP280_Data).dig_P3)) >>8) + ((var1_p * ((int64_t)((BMP280_Data).dig_P2))) <<12);
    var1_p = ((((int64_t)1)<<47) + var1_p) * ((int64_t)((BMP280_Data).dig_P1))>>33;
		
    if (var1_p == 0) {
//        						FTDI.printf("var1 is zero!\r\n");
											return 0;
										 }
    press = 1048576 - adc_P;
		press = (((press << 31) - var2_p) * 3125) / var1_p;
							
    var1_p = (((int64_t)(BMP280_Data).dig_P9) * (press >> 13) * (press >> 13)) >> 25;
    var2_p = (press * (int64_t)((BMP280_Data).dig_P8)) >> 19;
    press = ((press + var1_p + var2_p) >> 8) + (((int64_t)((BMP280_Data).dig_P7)) <<4);
    *pressPa = (float)(press / 256.0);
//    FTDI.printf("press is %d\r\n", press);
    
    return 0;
}


float BMP280_readAltitude(float seaLevelhPa, float Press)
{
  float altitude;
	
	Press=Press/100.0;

  altitude = 44330 * (1.0 - pow(((double)Press / seaLevelhPa), 0.1903));

  return altitude;
}

bool BMP280_Soft_Reset(int addr, bool *TimeOut_Ended)
{
	char data[]={BMP280_REG_RESET, 0xB6};
	
	bool FLAG=false;
	*TimeOut_Ended=false;
	
	Timer t;
	t.reset();
	
	t.start();
	float debut=t.read();
	
	do
	{
		if(Capt_I2C1.write(addr, NULL,0))
		{
			FLAG=true;
			if((t.read()-debut)>I2C_TIMEOUT) {
																				t.stop();
																				*TimeOut_Ended=true;
																				break;
																			 }
		}	
			else
			{
				FLAG=false;
				t.stop();
				
				Capt_I2C1.write(addr, data, 2);
	
				//FTDI.printf("\r\nSoft Reset termine");
				//Start up time de 2ms
				wait_ms(2);
				return true;
				
			}
	}
	while(FLAG);

}
