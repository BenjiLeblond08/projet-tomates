// Teensy 3.2 USB Serial

//A lire
//https://developer.mbed.org/cookbook/Power-Management
//https://developer.mbed.org/blog/entry/Using-the-new-mbed-power-management-API/
//https://developer.mbed.org/teams/SiliconLabs/wiki/Using-the-improved-mbed-sleep-API
//https://developer.mbed.org/forum/bugs-suggestions/topic/434/?page=1#comment-14029
//https://developer.mbed.org/users/no2chem/notebook/mbed-power-controlconsumption/
//http://electronicdesign.com/power/power-saving-tips-when-rapid-prototyping-arm-cortex-m-mcus

//A lire sur capteur 4IN1
//http://www.kandrsmith.org/RJS/Misc/Hygrometers/calib_many.html
//http://www.electrodragon.com/w/AM2322
//https://learn.sparkfun.com/tutorials/tsl2561-luminosity-sensor-hookup-guide
//https://learn.adafruit.com/bmp085
//https://developer.mbed.org/users/kgills/code/BMP180/
//https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/input/misc/bmp180.c
//https://github.com/BoschSensortec/BMP180_driver/blob/master/bmp180.h
//https://developer.mbed.org/users/kgills/code/BMP180/file/b2219e6e444b/BMP180.h
//https://developer.mbed.org/users/spiridion/code/BMP180/file/072073c79cfd/BMP180.h
//https://developer.mbed.org/users/kgills/code/BMP180/file/b2219e6e444b/BMP180.h
//https://developer.mbed.org/users/kgills/code/BMP180_example/file/20c4b5932337/main.cpp
//http://www.dsscircuits.com/forum/index.php?topic=9.0
//Autres unités :
//https://developer.mbed.org/users/onehorse/code/BMP180/file/06dc60296e6e/main.cpp
//Valeur pression niveau de la mer :
//http://www.wofrance.fr/weather/maps/forecastmaps?CONT=euro&MAPS=d&LAND=FR
//http://www.meteociel.fr/observations-meteo/pression.php
//http://www.meteo-bretagne.fr/observations/pression-atmospherique

//A lire pour I2C
//https://developer.mbed.org/forum/bugs-suggestions/topic/905/?page=2
//https://developer.mbed.org/forum/mbed/topic/586/
//https://developer.mbed.org/forum/bugs-suggestions/topic/4254/
//A lire -> important sur I2C
//https://developer.mbed.org/forum/bugs-suggestions/topic/5220/

//A lire pour SPI Slave
//https://developer.mbed.org/users/mbed_official/code/mbed/docs/tip/SPISlave_8h_source.html
//https://developer.mbed.org/questions/1331/How-to-Delay-SPI-Slave-to-Master-Respons/
//A lire important -> preuve SPI master pas de SSEL
//https://developer.mbed.org/users/mbed_official/code/mbed-src/file/d73ca02bc818/targets/hal/TARGET_Freescale/TARGET_KPSDK_MCUS/spi_api.c


//A lire SPI hardware
//https://github.com/FastLED/FastLED/wiki/SPI-Hardware-or-Bit-banging
//https://developer.mbed.org/users/embeddedartists/notebook/lpc4088-quickstart-board---how-to-expand---spi/
//https://developer.mbed.org/questions/54407/Does-mbeds-SPI-class-make-use-of-hardwar/

//A lire Oregon S protocole -> Lien Arduino
//http://www.connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/
//http://www.connectingstuff.net/blog/decodage-des-protocoles-oregon-scientific-sur-arduino-2/
//https://github.com/Cactusbone/ookDecoder

//A lire -> mbed
//https://developer.mbed.org/users/igfarm/notebook/decoder-for-oregon-scientific-weather-sensor/

//A lire -> mbed -> Sleep
//https://developer.mbed.org/users/mbed_official/code/mbed-src/file/d73ca02bc818/targets/hal/TARGET_Freescale/TARGET_KPSDK_MCUS/sleep.c
//https://forum.pjrc.com/threads/23660-Low-Power-quot-Green-quot-Battery-Operation-Solutions-For-The-Teensy-3/page7?s=b8b15649fba27e603e01e6d200864f30
//https://www.pjrc.com/teensy/low_power.html

//A lire -> mbed | Timer et mesure de durée
//https://developer.mbed.org/forum/mbed/topic/3876/
//https://developer.mbed.org/users/frank26080115/code/LPC1700CMSIS_Lib/docs/84d7747641aa/lpc17xx__timer_8c.html
//https://developer.mbed.org/users/frank26080115/code/LPC1700CMSIS_Lib/
//https://developer.mbed.org/users/kenjiArai/notebook/simple-frequency-counter/
//https://developer.mbed.org/questions/54393/This-pulse-counter-use-interrupts/
//https://developer.mbed.org/forum/mbed/topic/2326/

//A lire CAN entrée differentielle
//https://developer.mbed.org/components/ADC-DIFF-K64F/
//https://developer.mbed.org/questions/69732/ADC_DIFF-library-does-not-work-with-the-/
//https://developer.mbed.org/components/ADC-DIFF-K64F/
//https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1
//https://github.com/PaulStoffregen/Audio/commit/d5bfe6c3ea554395e90bab178eabedb795984e6f
//https://developer.mbed.org/users/fblanc/code/AnalogIn_Diff_ok/file/f39be15f056c/AnalogIn_Diff.h
//https://developer.mbed.org/users/fblanc/code/AnalogIn_Diff_ok/file/f39be15f056c/AnalogIn_Diff.cpp

//A lire CAN par IT
//https://developer.mbed.org/forum/mbed/topic/2052/

//A lire Lowpower API
//https://developer.mbed.org/blog/entry/Using-the-new-mbed-power-management-API/
//https://developer.mbed.org/teams/SiliconLabs/wiki/Using-the-improved-mbed-sleep-API
//https://developer.mbed.org/teams/SiliconLabs/code/Serial-LowPower-Demo/file/7136baf24c9c/main.cpp

//A lire -> tres bien pour economie energie
//https://developer.mbed.org/users/Sissors/code/WakeUp/
//https://developer.mbed.org/questions/54101/NucleoF411RE-is-LowPowerTicker-available/

//A lire 1 Wire
//https://developer.mbed.org/users/fblanc/code/OneWire_II/file/5d39f2521173/onewire.cpp

//A lire : Capteur AM2322 est commandable par 1-Wire
//http://rocksolidhead.blogspot.com/2016/11/am2322_28.html
//Adresse trouvée par scan : 0x5c -> ne repond pas systematiquement.
//https://github.com/micropython/micropython/issues/2290
//https://github.com/Ten04031977/AM2320-master/blob/master/am2320.cpp
//https://github.com/Ten04031977/AM2320-master/blob/master/AM2320.h

//A lire - SPI MASTER avec SSEL hardware
//https://developer.mbed.org/forum/helloworld/topic/4474/

//A lire - cours c - static -vsprintf
//http://stackoverflow.com/questions/572547/what-does-static-mean-in-a-c-program
//http://www.cplusplus.com/reference/cstdio/vsprintf/
//https://developer.mbed.org/forum/mbed/topic/3874/?page=1#comment-24690






// Unplug then plug back in the Teensy after programing to reactivate the Teensy USB serial port.
// if your terminal program can't see the Teensy

#include "mbed.h"
#include "math.h"
#include "USBSerial.h"



// include to check/display clock rates
#include "clk_freqs.h"
//Utilisation avec printf et USBSerial
//PC.printf("\r\n core %d",SystemCoreClock);
//PC.printf("\n Bus  %d",bus_frequency());
//PC.printf("\n Osc  %d",extosc_frequency());

//Definition de la durée entre chaque lecture
#define DUREE_ENTRE_IT	1

//Taille du tableau de mesure
#define MAX_TAILLE 80

//Definition pour I2C
#define I2C_ACK			1
#define I2C_NACK		0
#define I2C_TIMEOUT	2

//Adresse du capteur I2C
//Ici 8bits car attente de 8bits pour l'adresse en I2C
#define ADDR_LUX 0b01000110
//Resolution de 0.5lx et attente de 120ms au minimum
#define Resolution_05LUX 0b00100001

//Adresse capteur BOSCH BMP180
#define BMP180_ADDR 0x77<<1
#define BMP180_CALIBRATION_DATA_START 	0xAA
#define BMP180_CALIBRATION_DATA_LENGTH 	11 
#define BMP180_CHIP_ID 									0x55
#define BMP180_CHIP_ID_REG              0xD0
#define BMP180_VERSION_REG              0xD1
#define BMP180_CTRL_REG                 0xF4
#define BMP180_TEMP_MEASUREMENT         0x2E
#define BMP180_PRESSURE_MEASUREMENT     0x34
#define BMP180_CONVERSION_REGISTER_MSB  0xF6
#define BMP180_CONVERSION_REGISTER_LSB  0xF7
#define BMP180_CONVERSION_REGISTER_XLSB 0xF8
#define BMP180_TEMP_CONVERSION_TIME     5
#define BMP180_VENDORID                 0x0001
#define BMP180_SOFT_RESET_REG						0xE0
#define BMP180_SOFT_RESET								0xB6

#define BMP085_ULTRALOWPOWER         		0
#define BMP085_STANDARD              		1
#define BMP085_HIGHRES               		2
#define BMP085_ULTRAHIGHRES          		3

#define PRESSION_NIVEAU_MER							1033.9//1013.25//1033.9//Paris le 20/2/2017 1022.2hPa

//-----------------------------------------------------------------------
//Memoire EEPROM 24LC256
//-----------------------------------------------------------------------

//Adresse EEPROM : 1010 A2(=0) A1(=0) A0(=0) R/W(0)
#define ADRESSE_SELECTION_EEPROM        0b10100000 //0x50
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
//Capteur humidité HH10D - la sortie est une frequence. 
//-----------------------------------------------------------------------

//Gestion du HH10D - Selection nombre de bits - Adresse
#define ADRESSE_SELECTION_HH10D 0b10100010 //0x51
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



//Permuter un octet - 1ere solution
//b7b6b5b4b3b2b1b0 -> b0b1b2b3b4b5b6b7
#define MSB2LSB(b) (((b)&1?128:0)|((b)&2?64:0)|((b)&4?32:0)|((b)&8?16:0)|((b)&16?8:0)|((b)&32?4:0)|((b)&64?2:0)|((b)&128?1:0))

//Permuter un octet sur un ARM 32bits - 2ème solution
//uint8_t b -> b est l'octet à inverser
//b = ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;


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
//1200bauds -> AT+B1200
#define HC12_B1200 "AT+B1200"
#define HC12_B1200_Rep "OK+B1200"
//Verification Configuration
#define HC12_Config "AT+RX"
//Mode SLEEP
#define HC12_SLEEP "AT+SLEEP"
#define HC12_SLEEP_Rep "OK+SLEEP"



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
 
//Virtual serial port over USB
USBSerial PC;

//Liaison serie via RX0(PTB16) TX0(PTB17)
//               TX    RX
Serial Serie_433(PTB17,PTB16);

//Configuration I2C
//					 SDA   SCL
I2C Capt_I2C(PTB1,PTB0);

//Commande de la LED1 en sortie
//Ici incompatible aves SPI hardware
DigitalOut LED1_ON_Off(PTC5,LED1_OFF);

//Entrée analogique pour le CAN
//AnalogIn AIN(PTC0);
//Broche avec changement etat pour mesure duree de conversion
//Initialisation NL1
//Mesure commence sur front descendant sur PTD0
DigitalOut TCAN(PTD0,1);

//AnalogOut sur la broche DAC
//AnalogOut VCNA(DAC);

/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////

DigitalOut CS_HC12(PTC7,1);

//Liaison SPI avec selection materiel -> ADXL345
//SPI ACC(MOSI,MISO,SCLK,SSEL);
//SPI ACC(PTD2,PTD3,PTD1,PTD4); //Ne fonctionne pas
//SPI ACC(PTC6,PTC7,PTC5,PTC4); //Test avec les broches recommandées dans le PinNames.h -> ne fonctionne pas
//Broches recommandées :
//Solution 1 :
//SPI hardware : 	MOSI (DOUT) = PTC6 (D11)
//SPI hardware : 	SCK         = PTC5 (D13)
//Solution 2 :
//SPI hardware : 	MOSI (DOUT) = PTD2 (D7)
//SPI hardware : 	SCK         = PTD1 (D14)
//Analyse du datasheet : Cf. page 64
//								CS		SCK		OUT		IN				MOSI	MISO	SCLK	SSEL
//Association 1:  PTC4	PTC5	PTC6	PTC7 ->		PTC6	PTC7	PTC5	PTC4	-> ne fonctionne pas
//Association 2:  PTD0	PTD1	PTD2	PTD3 ->		PTD2	PTD3	PTD1	PTD0	-> ne fonctionne pas
//SPI ACC(MOSI,MISO,SCLK,SSEL);
//SPI ACC(PTD2,PTD3,PTD1,PTD0);
//SPI ACC(PTD2,PTD3,PTD1);
//SS logiciel pour la liason SPI
//DigitalOut CS(PTD0,1);

//Utilisation d'un Ticker pour changer periodiquement l'etat de la LED et lire la luminosité en I2C
//Ticker Ma_Duree;

//Utilisation d'une broche en entrée pour compter les fronts d'horloge en sortie du 555.
//Fout du HH10D
//Sur broche PTC2(D23) -> allumage de la LED (PTC5)
//Ici sur un front montant (importe peu le front)
//InterruptIn IT_Fout_HH10D(PTC2);
//TimeOut pour une durée de 1s
//Timeout Mesure_T_HH10D_1s;
//Compteur IT du HH10D
volatile int CPT_IT_HH10D=0,CPT_IT_HH10D_Res;

//Timer - mesure d'une durée
//Timer MesCAN;

//Mode LowPower
//LowPowerTicker ModeEco;

//Declaration PWM sur broche PTA12
//PwmOut PWM_PTA12(PTA12);





//Variables globales
volatile bool FLAG=TRUE;
volatile float Res_CAN;
volatile unsigned short Res_us;

//Declaration du type struct pour le capteur BMP180
//Afin d'obtenire les valeurs de la calibration
typedef struct
{
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
	//Champ pour calculer la pression à partir de la température brute
	long B5;
} Type_BMP180;

//Declaration du type struct pour obtenir les meseures brutes de : Altitude, Pression, Temperature
typedef struct
{
	float Altitude, Pression, Temperature;
	int PressionBrute, VAlTemperatureBrute;
	char OverSampling;
} Type_Mesure_BMP180;
						
						
						
//Prototype des fonctions
void IT_LED(void);
void Lecture_Calibration(Type_BMP180 *Capt);
void Affichage_Calibration(Type_BMP180 Capt);
int Lecture_Temperature_Brute_BMP180(void);
float Lecture_Temperature_BMP180(int ValTempBrute , Type_BMP180 *Capt);
int Lecture_Pression_Brute_BMP180(char oversampling);
float Lecture_Pression_BMP180(int ValPressBrute , char oversampling , Type_BMP180 Capt);
float Calcul_ALtitude_BMP180(float Pression, float Pression_Niveau_Mer);

char i2c_lecture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE);
void i2c_lecture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE,  int NOMBRE_A_LIRE, char *dest);
void i2c_ecriture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char DATA);
void i2c_ecriture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char *t);

//Permuter tous les bits d'un octet
char Permuter_8bits(char octet);

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du HH10D en I2C
/////////////////////////////////////////////////////////////////////////////

void i2c_lecture_OFFSET_SENS_HH10D(int ADRESSE_HH10D,int *SENS, int *OFFSET);

//IT sur front montant de HH10D
void IT_HH10D(void);
void IT_HH10D_CPT(void);

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////
bool sendATcommand(char* ATcommand, char* expected_answer, float timeout);
void Enter_CMD_mode(void);//NL0 pendant >40ms
void Leave_CMD_mode(void);//NL1 et attente MAJ pendant >80ms avant retout mode transparent


//---------------------------------------------------------------------------

int main(void) 
{
	//Tableau pour la transmission des caractéres
	char TabMes[MAX_TAILLE];
	
	//Capteur BOSCH BMP180
	char ID_BMP180=BMP180_CHIP_ID_REG;
	char VAL_ID_BMP180;
	char TempTab[2];
	
	//Déclaration de la variable Type_BPM180
	Type_BMP180 					Capteur_BMP180;
	Type_Mesure_BMP180 		Data_Capteur_BMP180;
	
	//Vitesse du bus I2C -> 400kbit/s
	Capt_I2C.frequency(100000);
	
	//Affectation de la fonction à appeler toutes les DUREE_ENTRE_IT
	//Ma_Duree.attach(&IT_LED,DUREE_ENTRE_IT);
	
	//Configuration liaison serie
	//reglage usine : 9600 bauds
	Serie_433.baud(9600);
	Serie_433.format(8,SerialBase::None,1);
	
	//Tableau pour affichage vers Serie_433
	char Serie_433_tab[80];
	
	//Variables pour UART MASTER
	//Emission toutes les 1s d'un caractére
	//Affichage par le maitre de l'emission
	int TX_Buffer[]={'0','1','2','3','4','5','6','7','8','9'};
	int RX_Buffer[10];
	int i=0;
	
	//Variable pour EEPROM 24LC256
	int Var_Adresse_Interne=VAL_ADRESSE_INTERNE;
	
	//Variable pour attendre pression d'une touche
	int c;
	
	//Variable pour HH10D
	int HH10D_Sens, HH10D_Offset;
	
	//I2C
	bool FLAG_I2C=false;
	
	//CAN
	//Sauvegarde resultat de conversion
	float Res_CAN_f;
	//Mesure de la durée en us
	int Debut,Fin;
	
	//HC-12 en 433.4MHz
	char tab_HC_12_CMD_AT[HC12_TAILLE_BUFFER];
	char tab_HC_12_reponse_AT[HC12_TAILLE_BUFFER];
		
	
	
	
	//Test sortie du CNA
	//Sortie de 3.3V
	//VCNA=1;
	//Sortie de 1V
	//VCNA=1/3.3;
	
	//Variables pour ADXL345
	//char buffer[6];
	//int16_t data[3];
	//float x,y,z;
		
	//ADXL345 inactif -> CS=1 fait par SPI hardware 
  //CS=1;
    
  //Configuration de la vitesse et format de transmission pour SPI
	//ADXL345 -> fmax=5MHz
	//SPI en 4 bits et mode 2 et 20kbit/s -> vitesse mesurée de 18,76kHz
	//Vitesse de 1Mhz -> bonne valeur.
   //ACC.format(16,2);
   //ACC.frequency(1000000);
    
   
	 
	  wait(10);
		
		//Serie_433.printf("\r\nFin attente de 10s.");
		//Serie_433.printf("\r\n core %dHz",SystemCoreClock);
		//Serie_433.printf("\r\n Bus  %dHz",bus_frequency());
		//Serie_433.printf("\r\n Osc  %dHz",extosc_frequency());
		
		//Periode pour la PWM
		//T=100us pour toutes les PWM
		//PWM_PTA12.period_us(100);
		
		//Compteur pour AM2322
		Debut=0;
    
    while(TRUE)
    {
			PC.printf("\r\n\n Presser une touche pour continuer - AT HC12 - 9600bauds.\r\n");	
			c=PC.getc();
			
			Enter_CMD_mode();
			
			//Entrer en mode AT -> "AT"
			sprintf(tab_HC_12_CMD_AT, HC12_AT);
			sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand=OK");
			else 
			{
				PC.printf("\r\n sendATcommand=ERROR");
				PC.printf("\r\n Changement vitesse en 1200bauds");
				//Changement vitesse transmission RS232 suite nouvelle configuration.
				//Configuration liaison serie
				//reglage usine : 1200 bauds
				Serie_433.baud(1200);
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
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand=AT+FU4");
			else PC.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 100mW -> "AT+P8"
			sprintf(tab_HC_12_CMD_AT, HC12_100mW);
			sprintf(tab_HC_12_reponse_AT, HC12_100mW_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand=AT+P8");
			else PC.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 433.1MHz -> "AT+C001"
			sprintf(tab_HC_12_CMD_AT, HC12_CANAL1);
			sprintf(tab_HC_12_reponse_AT, HC12_CANAL1_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand=AT+C001");
			else PC.printf("\r\n sendATcommand=ERROR");
			
			//Configuration mode 1200bauds -> "AT+B1200"
			sprintf(tab_HC_12_CMD_AT, HC12_B1200);
			sprintf(tab_HC_12_reponse_AT, HC12_B1200_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand=AT+B1200");
			else PC.printf("\r\n sendATcommand=ERROR");
			
			//Entrer en mode SLEEP -> "AT+SLEEP"
			sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
			sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand=AT+SLEEP");
			else PC.printf("\r\n sendATcommand=ERROR");
			
			Leave_CMD_mode();
			
			
			
			Enter_CMD_mode();
			
			//Entrer en mode AT -> "AT"
			sprintf(tab_HC_12_CMD_AT, HC12_AT);
			sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand en 1200bauds = OK");
			else PC.printf("\r\n sendATcommand=ERROR");
			
			//Leave_CMD_mode();
			
			//Enter_CMD_mode();
			
			//Entrer en mode SLEEP -> "AT+SLEEP"
			sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
			sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
			if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) PC.printf("\r\n sendATcommand=AT+SLEEP");
			else PC.printf("\r\n sendATcommand=ERROR");
			
			Leave_CMD_mode();
			
			
			
			
			
			
			
			
			
			
			//Gestion du mode Sleep()
			//On coupe les phéripheriques inutiles.
			//declaration d'un LowPowerTimeout
			
			/*
			//Test de la permutation des bits d'un octet
			char octet=0b11110100;
			Serie_433.printf("\r\n\n Avant SWAP : 0x%x | %d\r\n",octet,octet);	
			octet = MSB2LSB(octet);
			Serie_433.printf("\r\n\n Apres SWAP : 0x%x | %d\r\n",octet,octet);	
			
			//Test 2eme solution
			octet = 0b11110100;
			Serie_433.printf("\r\n\n Avant SWAP : 0x%x | %d\r\n",octet,octet);	
			octet = ((octet * 0x0802LU & 0x22110LU) | (octet * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
			Serie_433.printf("\r\n\n Apres SWAP : 0x%x | %d\r\n",octet,octet);
			
			//Test 3ème solution
			octet=0b11110100;
			char inverse=0;
			
			Serie_433.printf("\r\n\n Avant SWAP : 0x%x | %d\r\n",octet,octet);	
			
			for(int i=0; i<8 ; i++)
			{
				inverse = inverse | ((octet>>i) & 0b1)<<(7-i);
			}
			
			Serie_433.printf("\r\n\n Apres SWAP : 0x%x | %d\r\n",inverse,inverse);
			
			
			//Test 3ème solution
			octet = 0b11110100;
			Serie_433.printf("\r\n\n Avant SWAP : 0x%x | %d\r\n",octet,octet);
			octet=Permuter_8bits(octet);
			Serie_433.printf("\r\n\n Apres SWAP : 0x%x | %d\r\n",octet,octet);
			*/
			
			
			
			
			/*
			//Test I2C avec une adresse qui ne correspond à un esclave, puis aucun escalve.
			//L'objectif est de vérifier si write(adresse,NULL,0) emet un stop en cas d'echec.
			//La fonction retourne 0 en cas de succés, une valeur differente de 0 en cas d'echec.
			
			Capt_I2C.start();
			
			do
			{
				TCAN=0;
				
				if(Capt_I2C.write(ADRESSE_SELECTION_HH10D)==0)
				{
					//L'esclave ne répond pas ou est occupé
					//write termine par un stop
					//Ici, stop puis start est un restart
					Capt_I2C.stop();
					Capt_I2C.start();
					
					
					LED1_ON_Off=LED1_ON_Off^1;
					
					//Serie_433.printf("\r\nErreur Slave repond : NAK");
					
					FLAG_I2C=true;
				}
				else
				{
					//L'esclave a bien repondu présent et est pres à travailler avec le maitre
					
					//Serie_433.printf("\r\nSlave repond : AK");
					
					
					FLAG_I2C=false;
				}
				
				
			}
			while(FLAG_I2C);
			
			TCAN=1;
			*/
			
			
			
			
			
			
			/*
			
			do
			{
				TCAN=0;
				
				if(Capt_I2C.write(ADRESSE_SELECTION_HH10D,NULL,0))
				{
					//L'esclave ne répond pas ou est occupé
					//write termine par un stop
					//Ici, stop puis start est un restart
					
					
					LED1_ON_Off=LED1_ON_Off^1;
					
					//Serie_433.printf("\r\nErreur Slave repond : NAK");
					
					FLAG_I2C=true;
				}
				else
				{
					//L'esclave a bien repondu présent et est pres à travailler avec le maitre
					
					//Serie_433.printf("\r\nSlave repond : AK");
					
					
					FLAG_I2C=false;
				}
				
				
			}
			while(FLAG_I2C);
			
			TCAN=1;
			*/
			
			/*
			//Scan du bus I2C
			Serie_433.printf("\n\r\nScan du bus I2C.\n\r");
			
			for (int i=0; i<128 ; i++)
			{
				Capt_I2C.start();
				
				if(Capt_I2C.write(i<<1)==1) Serie_433.printf("0x%x ACK \r\n",i);
				
				wait_ms(10);
				
				Capt_I2C.stop();
				
			}
			*/
			
			/*
								FLAG=false;
			
								do
                {
                        if (Capt_I2C.write(ADRESSE_SELECTION_AM2322,NULL,0))
                         {
                           FLAG=true;
													 LED1_ON_Off=LED1_ON_Off^1;
                         }
                        else
                                {
                                        FLAG=FALSE;
																				Serie_433.printf("\n\r\nReponse ACK du AM2322 : %d.\n\r",Debut++);
																	
																				//Capt_I2C.write(ADRESSE_EEPROM, tab, 3);
																	
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
			
			
			
			
			
				//LED1_ON_Off=LED1_ON_Off^1;
			
			*/
			
			
			
    }   
		
		
		
		
}

//---------------------------------------------------------------------------

//Codage des fonctions

void IT_HH10D(void)
{
	//LED1_ON_Off=LED1_ON_Off^1;
	CPT_IT_HH10D++;
}

/*
void IT_HH10D_CPT(void)
{
	//LED1_ON_Off=LED1_ON_Off^1;
	//CPT_IT_HH10D_Res=CPT_IT_HH10D;
	IT_Fout_HH10D.rise(NULL);
	FLAG=false;
	
}
*/


void IT_LED(void)
{
	//Incompatible SPI hardware
	LED1_ON_Off=LED1_ON_Off^1;

	FLAG=FALSE;
}

void Lecture_Calibration(Type_BMP180 *Capt)
{
	//Variables temporaires pour ecrire et lire les valeurs de calibration
	char MSB,LSB,ValTemp;
	int Result;
	bool FLAG_MSB=true;
	
	
	//Lecture des 11 valeurs de calibration
	for (char i=BMP180_CALIBRATION_DATA_START ; i<(BMP180_CALIBRATION_DATA_START+BMP180_CALIBRATION_DATA_LENGTH*2) ; i++)
	{
		
		Capt_I2C.write(BMP180_ADDR,&i,1,true);//selection case mémoire 0xAA en 1er -> MSB en 1er -> MSB
		Capt_I2C.read(BMP180_ADDR,&ValTemp,1);//lecture de la valeur à l'adresse 0xAA
		
		if(FLAG_MSB) {
									MSB=ValTemp;
									FLAG_MSB=false;
									//PC.printf("\r\n0x%0x=0x%0x",i,MSB);
			
								 }
		else {
					
					LSB=ValTemp;
					FLAG_MSB=true;
					//PC.printf("\r\n0x%0x=0x%0x",i,LSB);
			
					//Sauvegarde de cette valeur lue :
					Result=(MSB<<8) | LSB;
					//PC.printf("\r\nResult=0x%0x | %d",Result,Result);
					switch(i)
									{
										case 0xAB :	(*Capt).AC1=(short)Result;
																break;
										case 0xAD :	(*Capt).AC2=(short)Result;
																break;
										case 0xAF :	(*Capt).AC3=(short)Result;
																break;
										case 0xB1 :	(*Capt).AC4=(unsigned short)Result;
																break;
										case 0xB3:	(*Capt).AC5=(unsigned short)Result;
																break;
										case 0xB5 :	(*Capt).AC6=(unsigned short)Result;
																break;
										case 0xB7:	(*Capt).B1=(short)Result;
																break;
										case 0xB9:	(*Capt).B2=(short)Result;
																break;
										case 0xBB:	(*Capt).MB=(short)Result;
																break;
										case 0xBD:	(*Capt).MC=(short)Result;
																break;
										case 0xBF:	(*Capt).MD=(short)Result;
																break;
			
									}
			
				 }
		
		
		
	}
}

void Affichage_Calibration(Type_BMP180 Capt)
{
	PC.printf("\r\nAC1=0x%0x|AC2=0x%0x|AC3=0x%0x|AC4=0x%0x|AC5=0x%0x|AC6=0x%0x|B1=0x%0x|B2=0x%0x|MB=0x%0x|MC=0x%0x|MD=0x%0x",Capt.AC1,Capt.AC2,Capt.AC3,Capt.AC4,Capt.AC5,Capt.AC6,Capt.B1,Capt.B2,Capt.MB,Capt.MC,Capt.MD);
	PC.printf("\r\nAC1=%d|AC2=%d|AC3=%d|AC4=%d|AC5=%d|AC6=%d|B1=%d|B2=%d|MB=%d|MC=%d|MD=%d",Capt.AC1,Capt.AC2,Capt.AC3,Capt.AC4,Capt.AC5,Capt.AC6,Capt.B1,Capt.B2,Capt.MB,Capt.MC,Capt.MD);
}

int Lecture_Temperature_Brute_BMP180(void)
{
	//Variables temporaires
	//								0xF4							0x2E
	char TabTemp[2]={BMP180_CTRL_REG , BMP180_TEMP_MEASUREMENT};
	int Result;
	
	//selection du registre de controle (0xF4) et ecriture de la valeur 0x2E
	Capt_I2C.write(BMP180_ADDR,TabTemp,2);
	//Attente de 4,5ms mimimum - ici 5ms
	wait_ms(5);
	
	
	//Lecture des 2 octets de la temperature brute
	//lecture de 2 valeurs : MSB en 0xF6 et LSB en 0xF7
	//MSB en 1er
	TabTemp[0]=BMP180_CONVERSION_REGISTER_MSB;
	
	Capt_I2C.write(BMP180_ADDR,TabTemp,1,true);
	Capt_I2C.read(BMP180_ADDR,TabTemp,2);
		
	Result=(TabTemp[0]<<8) | TabTemp[1];
	
	return(Result);
}
//--------------------------------------------------------------------------

float Lecture_Temperature_BMP180(int ValTempBrute , Type_BMP180 *Capt)
{
	long X1=(((long)ValTempBrute-(long)(*Capt).AC6)*(long)(*Capt).AC5)>>15;
	long X2=((long)(*Capt).MC<<11)/(X1+(long)(*Capt).MD);
	long B5=X1+X2;
	(*Capt).B5=B5;
	long T=(B5+8)>>4;
	//Resultat precedent donné en 0,1°C
	return(T/10.0);
}
//--------------------------------------------------------------------------

int Lecture_Pression_Brute_BMP180(char oversampling)
{
	//Variables temporaires
								
	char TabTemp[3];
	//								0xF4
	TabTemp[0]=BMP180_CTRL_REG;
	//0x34+oversampling
	TabTemp[1]=0x34 + (oversampling<<6);
	
	int Result;
		
	//selection du registre de controle (0xF4) et ecriture de la valeur 0x34+(oversampling<<6)
	Capt_I2C.write(BMP180_ADDR,TabTemp,2);
	//Attente en fonction de oversampling
	
	switch(oversampling)
	{
		case 0: wait_ms(5); 
						break;
		case 1: wait_ms(8); 
						break;
		case 2: wait_ms(14); 
						break;
		case 3: wait_ms(26); 
						break;
	}
	
	
	//Lecture des 2 octets de la pression brute
	//lecture de 2 valeurs : MSB en 0xF6 et LSB en 0xF7 si 16bits et option 19bits lire en plus 0xF8
	//MSB en 1er
	TabTemp[0]=BMP180_CONVERSION_REGISTER_MSB;
	
	Capt_I2C.write(BMP180_ADDR,TabTemp,1,true);
	Capt_I2C.read(BMP180_ADDR,TabTemp,3);
		
	Result=(TabTemp[0]<<8) | TabTemp[1];
	Result=Result << 3;
	Result=Result | TabTemp[2];
	
	return(Result);
}
//--------------------------------------------------------------------------

float Lecture_Pression_BMP180(int ValPressBrute , char oversampling,  Type_BMP180 Capt)
{
	long B6=Capt.B5-4000;
	long X1=(B6*B6)>>12;
	
	X1=X1*(long)Capt.B2;
	X1=X1>>11;
	
	long X2=(long)Capt.AC2*B6;
	X2=X2>>11;
	
	long X3=X1+X2;
	
	long B3=((((long)Capt.AC1*4+X3)<<oversampling)+2)>>2;

	X1=((long)Capt.AC3*B6)>>13;
	
	X2=((long)Capt.B1*((B6*B6)>>12))>>16;
	X3=((X1+X2)+2)>>2;

	unsigned long B4=(Capt.AC4*(unsigned long)(X3+32768))>>15;

	unsigned long B7=((unsigned long)(ValPressBrute-B3)*(50000>>oversampling));

	long P=(B7<0x80000000) ? (B7<<1)/B4 : (B7/B4)<<1;

	X1=P>>8;
	X1=X1*X1;
	X1=(X1*3038)>>16;
	X2=(P*-7357)>>16;
	P=P+((X1+X2+3791)>>4);
	//Pression en Pa
	return(P);

}
//--------------------------------------------------------------------------

float Calcul_ALtitude_BMP180(float Pression, float Pression_Niveau_Mer)
{
	//Calcul de l'altitude, la pression locale au niveau de la mer est à rentrer
	
	float Altitude = 1 - pow(((double)Pression/((double)Pression_Niveau_Mer*100)),(double) 1/5.255);
	Altitude=Altitude*44330;
	
	return(Altitude);
	
}
//--------------------------------------------------------------------------



char i2c_lecture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE)
{
        bool FLAG=false;
        char temp;
        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);
				
				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255};

        do
                {

                        if (Capt_I2C.write(ADRESSE_EEPROM,NULL,0))
                         {
                          FLAG=true;
                         }
                        else
                                {
                                        FLAG=false;
																	
																				Capt_I2C.write(ADRESSE_EEPROM, tab, 2, true);
																	
                                        //Capt_I2C.write(ADRESSE_INTERNE>>8);
                                        //i2c1__ack_S2M();
																				//Capt_I2C.write(ADRESSE_INTERNE&255);
                                        //i2c1__ack_S2M();
																				
																				Capt_I2C.read(ADRESSE_EEPROM, &temp, 1);
																	
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


void i2c_lecture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE,  int NOMBRE_A_LIRE, char *dest)
{
        bool FLAG=false;
        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);

				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255};
        
        do
        {
        
                if(Capt_I2C.write(ADRESSE_EEPROM,NULL,0))
                        {
                         FLAG=true;
                        }
                  
                else
                {
                        FLAG=false;
									
												Capt_I2C.write(ADRESSE_EEPROM, tab, 2, true);
                        
                        //i2c1__write(ADRESSE_INTERNE>>8);
                        //i2c1__ack_S2M();
                        //i2c1__write(ADRESSE_INTERNE&255);
                        //i2c1__ack_S2M();
        
                        ///*
                        ///////////////////////////////////////////////////////////
                        //1ére Solution :
                        ///////////////////////////////////////////////////////////
        
												Capt_I2C.read(ADRESSE_EEPROM, dest , NOMBRE_A_LIRE);
									
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


void i2c_ecriture_eeprom_octet(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char DATA)
{
        bool FLAG=false;
        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);
	
				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255, DATA};

        do
                {
                        if (Capt_I2C.write(ADRESSE_EEPROM,NULL,0))
                         {
                           FLAG=true;
                         }
                        else
                                {
                                        FLAG=FALSE;
																	
																				Capt_I2C.write(ADRESSE_EEPROM, tab, 3);
																	
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


void i2c_ecriture_eeprom_page(int ADRESSE_EEPROM, int ADRESSE_INTERNE, char *t)
{
        bool PAGE=false;
        bool FLAG=false;
				int i,j=0;
				char *p;
				p=t;
        //int offset=0,i;

        ADRESSE_INTERNE=(ADRESSE_INTERNE & MASK_ADRESSE_INTERNE);
		
				char tab[]={ADRESSE_INTERNE>>8 , ADRESSE_INTERNE&255};
				
				//recherche de la taille du tableau
				//Si le tableau est une chaine de caractéres
				//terminaison par '\0'
				i=strlen(t);
				
							do
                {
									
											do
                        {
                                if (Capt_I2C.write(ADRESSE_EEPROM,NULL,0))
                                 {
																	Capt_I2C.stop(); 
                                  Capt_I2C.start();
                                  FLAG=true;
                                 }
                                else
                                {
                                        FLAG=false;
																	
																				Capt_I2C.write(ADRESSE_EEPROM, tab, 2);
																	
                                        //Capt_I2C.write(ADRESSE_INTERNE>>8);
                                        //i2c1__ack_S2M();
                                        //i2c1__write(ADRESSE_INTERNE&255);
                                        //i2c1__ack_S2M();
																	
																				
																				//Calculons la division et le modulo
																				if((i/TAILLE_BUFFER)==0) { 
																																	// reste à ecrire
																																	i= i % TAILLE_BUFFER;
																					
																																	Capt_I2C.write(ADRESSE_EEPROM, p, i);
																					
																																	//ADRESSE_INTERNE=(ADRESSE_INTERNE+i)& MASK_ADRESSE_INTERNE;
																																	//tab[0]=ADRESSE_INTERNE>>8;
																																	//tab[1]=ADRESSE_INTERNE&255;
																					
																																	PAGE=false;
																																	}
																				else {
																							//Ecriture de TAILLE_BUFFER octets
																							
																							Capt_I2C.write(ADRESSE_EEPROM, p, TAILLE_BUFFER);
																					
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





void i2c_lecture_OFFSET_SENS_HH10D(int ADRESSE_HH10D,int *SENS, int *OFFSET)
{
        bool FLAG=false;

        int SENS_MSB,OFFSET_MSB;
        char SENS_LSB,OFFSET_LSB;
	
				char tab[1];

							do
                {

                        if (Capt_I2C.write(ADRESSE_HH10D,NULL,0))
                           {
                            FLAG=true;
                           }
                        else
                            {
                                        FLAG=false;
															
																				tab[0]=ADRESSE_SENS_HH10D;
															
																				Capt_I2C.write(ADRESSE_HH10D, tab, 1, true);
																				Capt_I2C.read(ADRESSE_HH10D,tab,1);
															
																				SENS_MSB=tab[0];
															
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

								
        do
                {

                        if (Capt_I2C.write(ADRESSE_HH10D,NULL,0))
                           {
                             FLAG=true;
                           }
                        else
                                {
                                        FLAG=false;
																	
																				tab[0]=ADRESSE_SENS_HH10D+1;
															
																				Capt_I2C.write(ADRESSE_HH10D, tab, 1, true);
																				Capt_I2C.read(ADRESSE_HH10D,tab,1);
															
																				SENS_LSB=tab[0];
																	
																				/*
                                        //Emission adresse de début de lecture : 10. 4 octets à lire
                                        i2c1__write(ADRESSE_SENS_HH10D+1);
                                        //i2c1__ack_S2M();
                                        i2c1__rstart();

                                        //Mise à 1 du bit R/W de l'octet de "contrôle": pour une lecture
                                        i2c1__write(ADRESSE_HH10D|1);
                                        //i2c1__ack_S2M();

                                        //Lecture LSB de SENS @11
                                        SENS_LSB=i2c1__read(I2C_NOACQ);
                                        //i2c1__noack_M2S();
                                        
                                        i2c1__stop();
																				*/

                                }
                }
        while(FLAG);

        do
                {

                        if (Capt_I2C.write(ADRESSE_HH10D,NULL,0))
                           {
                              FLAG=true;
                           }
                        else
                                {
                                        FLAG=false;
																	
																				tab[0]=ADRESSE_OFFSET_HH10D;
															
																				Capt_I2C.write(ADRESSE_HH10D, tab, 1, true);
																				Capt_I2C.read(ADRESSE_HH10D,tab,1);
															
																				OFFSET_MSB=tab[0];
																	
																	
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

        do
                {

                        if (Capt_I2C.write(ADRESSE_HH10D,NULL,0))
                           {
                            FLAG=true;
                           }
                        else
                                {
                                        FLAG=false;
																	
																				tab[0]=ADRESSE_OFFSET_HH10D+1;
															
																				Capt_I2C.write(ADRESSE_HH10D, tab, 1, true);
																				Capt_I2C.read(ADRESSE_HH10D,tab,1);
															
																				OFFSET_LSB=tab[0];
																	
																	
																				/*
                                        //Emission adresse de début de lecture : 10. 4 octets à lire
                                        i2c1__write(ADRESSE_OFFSET_HH10D+1);
                                        //i2c1__ack_S2M();
                                        i2c1__rstart();

                                        //Mise à 1 du bit R/W de l'octet de "controle": pour une lecture
                                        i2c1__write(ADRESSE_HH10D|1);
                                        //i2c1__ack_S2M();

                                        //Lecture LSB de OFFSET @12
                                        OFFSET_LSB=i2c1__read(I2C_NOACQ);
                                        //i2c1__noack_M2S();
                                        
                                        i2c1__stop();
																				*/

                                }
                }
        while(FLAG);

                                        *SENS=(SENS_LSB|(SENS_MSB <<8));
                                        *OFFSET=(OFFSET_LSB|(OFFSET_MSB <<8));
}





char Permuter_8bits(char octet)
{
	char inverse=0;
	
	for(int i=0; i<8 ; i++)
			{
				inverse = inverse | ((octet>>i) & 0b1)<<(7-i);
			}
			
	return(inverse);
	
}

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////

bool sendATcommand(char* ATcommand, char* expected_answer, float timeout)
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
		while(!Serie_433.writeable());
    //Preparation de la commande AT et terminée par CR non necessaire ici
		sprintf(buffer,"%s%s",ATcommand,HC12_END_CMD);
	
    Serie_433.printf("%s",buffer);
		
		//Preparation du Timeout si la reponse du HC12 ne vient pas...
		t.reset();
		t.start();
	
    debut = t.read();
 
    // Gestion de la reponse du HC12
    do
		{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(Serie_433.readable())
					{    
            reponse[i++] = (char)Serie_433.getc();
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