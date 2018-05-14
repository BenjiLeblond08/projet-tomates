// Teensy 3.2 
// main_prof_reception.cpp

// Unplug then plug back in the Teensy after programing to reactivate the Teensy USB serial port.
// if your terminal program can't see the Teensy

#include "mbed.h"
// include to check/display clock rates
#include "clk_freqs.h"
//FTDI.printf("\r\n core %d",SystemCoreClock);
//FTDI.printf("\n Bus  %d",bus_frequency());
//FTDI.printf("\n Osc  %d",extosc_frequency());

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

#define ANNEE 2018
#define JOUR_SEMAINE_1_31	17
#define MOIS_ANNEE AVRIL

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

/////////////////////////////////////////////////////////////////////////////
// Fonction pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////
bool sendATcommand(char* ATcommand, char* expected_answer, float timeout);
void Enter_CMD_mode(void);//NL0 pendant >40ms
void Leave_CMD_mode(void);//NL1 et attente MAJ pendant >80ms avant retour mode transparent

/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////
DigitalOut CS_HC12(PTB19,1);
//HC12-433
//						TX   RX
Serial HC_12(PTB17,PTB16);


/////////////////////////////////////////////////////////////////////////////
// Gestion du 433MHz en Manchester
/////////////////////////////////////////////////////////////////////////////
#define OSV3_CRC8_M433_GP	0x107	//x^8+x^2+x+1
#define OSV3_CRC8_M433_DI	0x07
//Taille du tableau pour les données du capteur 
//Le capteur est temperature en °C et HR en % -> protocole Oregon Scientific V3 : THGR810
#define OSV3_TAILLE_DATA_CAPTEUR_THR	13
//Le capteur d'UV  -> protocole Oregon Scientific V3 : UVN800
#define OSV3_TAILLE_DATA_CAPTEUR_UV	12
//Possibilité de definir un canal de 1 à 15 : Oregon de 1 à 3
#define OSV3_CANAL_TEMP_HR	2
#define OSV3_CANAL_HAUTEUR_EAU 3
#define OSV3_CANAL_UV 9
#define DUREE_ENTRE_CHAQUE_MESURE_OSV3	9

/////////////////////////////////////////////////////////////////////////////
// Gestion du 433MHz en Manchester - encodage OSV2
/////////////////////////////////////////////////////////////////////////////
#define OSV2_TIME 		512
#define OSV2_TWOTIME	OSV2_TIME*2
#define SEND_HIGH()	Data_433=1;
#define SEND_LOW()	Data_433=0;
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

#define OSV2_CANAL_TEMP_HR_P	6
#define OSV2_CANAL_TEMP				3
#define OSV2_CANAL_TEMP_HR		7
 
//Virtual serial port over USB
//USBSerial PC;
//FTDI @ 921600bauds
//          TX,RX
Serial FTDI(PTD3,PTD2);


//Commande de la LED sur PTC5
DigitalOut LED_ON_Off(PTC5,0);

//Declaration du Ticker toutes les 30s
////Ticker Toutes_30s;


/////////////////////////////////////////////////////////////////////////////
// Objet la Gestion du 433MHz en Manchester
/////////////////////////////////////////////////////////////////////////////
Timer TimeManchester;
//Entrée analogique pour le CAN
AnalogIn AIN(PTC8);
/////////////////////////////////////////////////////////////////////////////
// Sortie pour l'emission de la data en 433.92MHz
/////////////////////////////////////////////////////////////////////////////
DigitalOut Data_433(PTC2,0);
/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion du 433MHz
/////////////////////////////////////////////////////////////////////////////
//NL0 pour test module 433 -> coupure alimentation
DigitalOut POWER(PTD7,0);






//Variables modifiées par la fonction d'IT
//Pour l'IT toutes les 30s
volatile bool FLAG_IT_MESURE=true;


//FLAG indique si un traitement est a effectuer par le programme principal
volatile bool FLAG=true;

//Prototype des fonctions pour OSV3
void OSV3_MANCHESTER_SEND_DATA(void);
unsigned char OSV3_CALC_CRC(bool *InitFait);
unsigned char OSV3_CALC_CRC_UV(bool *InitFait);
unsigned char OSV3_CALC_CHECKSUM_THR(void);
unsigned char OSV3_CALC_CHECKSUM_UV(void);
void OSV3_CONSTRUIRE_TRAME_THR(float Temp_f, int HumiHR);
unsigned char OSV3_ROLLING_CODE(void);
void OSV3_MANCHESTER_ENCODE(unsigned char Octet_Encode, bool Fin);
void OSV3_MANCHESTER_SEND_DATA_THR(void);
void OSV3_MANCHESTER_SEND_DATA_UV(void);
void OSV3_INIT_CRC(bool *InitFait);



//Prototype des fonctions pour OSV2
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
void SetTemperature(unsigned char *data,float Temp);
void SetHumidity(unsigned char *data,unsigned char Hum);
void SetPressure(unsigned char *data, float pres);
unsigned char Sum(unsigned char count, unsigned char *data);
void calculateAndSetChecksum_T(unsigned char *data);
void calculateAndSetChecksum_THR(unsigned char *data);
void calculateAndSetChecksum_THRP(unsigned char *data);

//---------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////
// Gestion du 433MHz en Manchester -> variables globales
/////////////////////////////////////////////////////////////////////////////
//https://sourceforge.net/p/wmrx00/discussion/855159/thread/3358e4e8/
//
//I got a BTHGN129 Temp, Hum and Pressure sensor, signal is reconize by WXSheild :
//12/01/2015 08:13:14 (UTC) - *OS2:5D5322F008108605D1295,3FF,00
//https://github.com/deennoo/rtl_osv21
//http://www.mattlary.com/2012/06/23/weather-station-project/
//https://gist.github.com/RouquinBlanc/5cb6ff88cd02e68d48ea
//https://www.weather-watch.com/smf/index.php?topic=56997.0
//OSV3 - UVN800
//https://github.com/letscontrolit/NodoClassic/blob/master/RFLinkNRF/Plugins/Plugin_048.c
//FF FF FF AD 87 41 64 80 0D 60 64 11
//
// 	packet[0] = 0xFF;
//	packet[1] = 0xFF;
// 	packet[2] = 0xFF;
//  packet[3] = 0xAD;
//  packet[4] = 0x87;
//  packet[5] = 0x41;		 //canal (1 à 15) : 4 bits de poids faible pour la valeur  
//  packet[6] = 0x64;    //rolling code
//  packet[7] = 0x80;		 //4 bits de poids faible pour la valeur des UV (0 à 15)
//  packet[8] = 0x0D;
//  packet[9] = 0x60;
//  packet[10] = 0x64;   // checksum
//  packet[11] = 0x11;   // not included in the checksum

//Tableau pour stocker tout le protole OSV3
unsigned char OSV3_TAB_DATA_HR[OSV3_TAILLE_DATA_CAPTEUR_THR];
unsigned char OSV3_TAB_DATA_UV[OSV3_TAILLE_DATA_CAPTEUR_UV]={0xFF,0xFF,0xFF,0xAD,0x87,0x41,0x57,0x80,0x0D,0x60,0x64,0x11};
unsigned char OSV3_TAB_CRC_INIT[256];
//Calcul de ROLLING_CODE : 114 - 59 - 30 - 183 - 242 - 29 - 41 - 162 - 63 - 233 - 152 - 243 - 170 - 219 - 163 - 212 - 60 - 246 - 138 - 196 - 131 - 139 - 243 - 71
unsigned char VAL_ROLLING_CODE;
bool Fait_Init_TAB_CRC=false;
//Pour tester l'emission en Manchester et 433MHz
float VAL_TempC;
unsigned int VAL_HR,VAL_UV;

//Pour codage OSV2
//Capteur T : THN132N
unsigned char OSV2_MessageBuffer_T[8];
//Capteur T et HR : THGR2228N
unsigned char OSV2_MessageBuffer_THR[9];
//Capteur T et HR et P : BTHR918N
unsigned char OSV2_MessageBuffer_THRP[11];
//ID pour THN132N
unsigned char ID_T[]={0xEA,0x4C};
//ID pour THGR2228N
unsigned char ID_THR[]={0x1A,0x2D};
//ID pour BTHR918N
unsigned char ID_THRP[]={0x5A,0x6D};


/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////
//Test transfert en binaire par HC-12
//Par un struct dans un union
//Bilan des capteurs utilises :
//T et HR 			-> THGR810 (OSV3)
//T et HR 			-> THGR2228N (OSV2)
//T et HR et P	-> BTHR918N (OSV2)
//T 					  -> THN132N (OSV2)
//UV					  -> UVN800 (OSV3)

//Cours C
//https://stackoverflow.com/questions/20426716/how-do-i-use-typedef-and-typedef-enum-in-c

//Fonction IT associee au HC12 en reception (RX)
void HC12_IT_RX(void);

//Variable globale pour HC12_IT_Flag
volatile bool HC12_IT_Flag=false;
volatile bool First_IT_HC12=false;
volatile int CPT_IT_HC12=0;

typedef enum list_OSV23 {THGR810=1, THGR2228N=2, UVN800=3, BTHR918N=4, THN132N=5} T_Enum;
							 
typedef struct {
								 time_t HeureAcqui;	//4 octets
								 float Temperature;	//4 octets
								 unsigned int HR;		//4 octets
								 unsigned int UV;					//4 octets
								 float UVA;					//4 octets
								 float UVB;					//4 octets
								 int Pression;		//4 octets
								 int RollingCode; 	//4 octet
								 T_Enum type_capteur; //4 octets
								 
							 } T_DB; //sizeof donne 36 octets

typedef union {
								T_DB Data_Capteur;
								char Tab_TU[sizeof(T_DB)]; //meme taille que T_DB et au même emplacement mémoire.
	
							} TU_DB;

//Test pour transmettre en binaire de transmission en HC12 puis encodage en OSV2 et OSV3:
//Struct associee aux champs - taille donnee par sizeof : 36
//Union associee aux champs - taille donnee par sizeof : 36
volatile TU_DB Tab_OSV23Binaire;


int main(void) 
{

	//Mise à l'heure
	//Variables pour la gestion de l'horloge RTC
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
	//Mise à l'heure de l'horloge RTC
	set_time(mktime(&mytime));
	
	//Configuration vitesse FTDI
	FTDI.baud(9600);
	FTDI.format(8, SerialBase::None, 1);
	
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
	
	//Mise sous tension du module 433.92MHz
	POWER=1;
	wait(1);

	
	FTDI.printf("\r\n\n Presser une touche pour continuer - HC12 TX par IT.");	
	int c=FTDI.getc();
	
	//FTDI.printf("\r\n\n Taille du type T_DB=%d | TU_DB=%d | float=%d | time_t=%d | int=%d | unsigned char=%d | enum list_OSV23=%d\r\n", sizeof(T_DB), sizeof(TU_DB), sizeof(float), sizeof(time_t), sizeof(int), sizeof(unsigned char), sizeof(T_Enum));
	
	//FTDI.printf("\r\n\n Taille de Tab_OSV23Binaire=%d | Tab_OSV23Binaire.Tab_TU[]=%d\r\n", sizeof(Tab_OSV23Binaire), sizeof(Tab_OSV23Binaire.Tab_TU));	
	
	
	if(!HC12_CONFIG_DONE)
	{
		HC12_CONFIG_DONE=true;

		Enter_CMD_mode();

		//Entrer en mode AT -> "AT"
		sprintf(tab_HC_12_CMD_AT, HC12_AT);
		sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
		if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT))
			FTDI.printf("\r\n sendATcommand=OK");
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

		//Configuration mode FU4 -> "AT+FU4"
		sprintf(tab_HC_12_CMD_AT, HC12_FU4);
		sprintf(tab_HC_12_reponse_AT, HC12_FU4_Rep);
		if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT))
			FTDI.printf("\r\n sendATcommand=AT+FU4");
		else 
			FTDI.printf("\r\n sendATcommand=ERROR");
		
		//Configuration mode 100mW -> "AT+P8"
		sprintf(tab_HC_12_CMD_AT, HC12_100mW);
		sprintf(tab_HC_12_reponse_AT, HC12_100mW_Rep);
		if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT))
			FTDI.printf("\r\n sendATcommand=AT+P8");
		else 
			FTDI.printf("\r\n sendATcommand=ERROR");
		
		//Configuration mode 441.4MHz -> "AT+C021"
		sprintf(tab_HC_12_CMD_AT, HC12_CANAL21);
		sprintf(tab_HC_12_reponse_AT, HC12_CANAL21_Rep);
		if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT))
			FTDI.printf("\r\n sendATcommand=AT+C021");
		else 
			FTDI.printf("\r\n sendATcommand=ERROR");
		
		//Configuration mode 1200bauds -> "AT+B1200"
		sprintf(tab_HC_12_CMD_AT, HC12_B1200);
		sprintf(tab_HC_12_reponse_AT, HC12_B1200_Rep);
		if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT))
			FTDI.printf("\r\n sendATcommand=AT+B1200");
		else 
			FTDI.printf("\r\n sendATcommand=ERROR");
		
		Leave_CMD_mode();
	}

	/*	
	//Test simple de transmission par HC12			
	char TabTest[]={'0','1','2','3','4','5','6','7','8','9','a','b','c','d'};							
		
	while(true)
	{
		FTDI.printf("\r\nTest transmission HC12");
		for(int i=0 ; i<sizeof(TabTest) ; i++)
		{
		//Attente Buffer libre
			while(!HC_12.writeable());
			//Emission d'un caratere
			HC_12.putc(TabTest[i]);
			//Affichage par le maitre de l'emission
			//FTDI.printf("\r\nEmission de %c",(char)Tab_OSV23Binaire.Tab_TU[i]);
		
		}
		FTDI.printf("\r\nFin Test transmission HC12");			
		//FTDI.printf("\r\nTempo de 5s");
		//wait(5);
		
	}
	*/

	//Attente de l'IT du HC12 puis encodage en OSV3 ou OSV2
	FTDI.printf("\r\n\n Attente de l'IT du HC12.");

	//Association de l'IT du HC12
	HC_12.attach(&HC12_IT_RX,Serial::RxIrq);

	//Boucle attente de l'IT de la trame HC12
	while(true)
	{
		if(HC12_IT_Flag)
		{
			CPT_IT_HC12=0;

			//Visualisation entree en IT
			LED_ON_Off=!LED_ON_Off;
			//Afichage des infos reçues
			FTDI.printf("\r\n");

			time_t OSV3_Time = *( (time_t*)(Tab_OSV23Binaire.Tab_TU) );
			FTDI.printf("\r\n OSV3_Time=%s",ctime(&OSV3_Time));

			float OSV3_Temperature = *( (float*)(Tab_OSV23Binaire.Tab_TU + 4) );
			FTDI.printf("\r\n OSV3_Temperature=%f",OSV3_Temperature);

			unsigned int OSV3_HR = *( (unsigned int*)(Tab_OSV23Binaire.Tab_TU + 8) );
			FTDI.printf("\r\n OSV3_HR=%d",OSV3_HR);

			int OSV3_Pression = *( (int*)(Tab_OSV23Binaire.Tab_TU + 24) );
			FTDI.printf("\r\n OSV3_Pression=%d",OSV3_Pression);

			int OSV3_numero = *( (int*)(Tab_OSV23Binaire.Tab_TU + 28) );
			FTDI.printf("\r\n OSV3_numero=%d",OSV3_numero);

			T_Enum OSV3_type_capteur = *( (T_Enum*)(Tab_OSV23Binaire.Tab_TU + 32) );
			FTDI.printf("\r\n OSV3_type_capteur=%d\r\n",OSV3_type_capteur);

			//Affichage sous forme hexa des donnees reçues
			FTDI.printf("\r\n");
			for (int i=0 ; i<sizeof(T_DB) ; i++)
			{
				FTDI.printf("|%#x",Tab_OSV23Binaire.Tab_TU[i]);
			}
			FTDI.printf("\r\n");
			
			//Affichage du type de capteur reçu
			FTDI.printf("\r\ntype_capteur=%d",Tab_OSV23Binaire.Data_Capteur.type_capteur);
			
			//Test du capteur pour encodage :
			switch (Tab_OSV23Binaire.Data_Capteur.type_capteur)
			{
				case THGR810 :
					//OSV23Binaire.HeureAcqui=seconds;
					//OSV23Binaire.Temperature=VAL_TempC;
					//OSV23Binaire.HR=VAL_HR;
					//OSV23Binaire.RollingCode=162;
					//OSV23Binaire.numero=2;
					//OSV23Binaire.type_capteur=THGR810;
					FTDI.printf("\r\n Construction de la Trame de THGR810");
		
					OSV3_TAB_DATA_HR[6]=(unsigned char)Tab_OSV23Binaire.Data_Capteur.RollingCode;
					OSV3_CONSTRUIRE_TRAME_THR(Tab_OSV23Binaire.Data_Capteur.Temperature, (int)Tab_OSV23Binaire.Data_Capteur.HR);
					//FTDI.printf("\r\n\n Transmission de la trame en OSV3 :T=%3.2f°C | HR=%d%%",VAL_TempC,VAL_HR);
		
					//Affichage complet
					//FTDI.printf("\r\n THGR810: %x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x",OSV3_TAB_DATA_HR[0],OSV3_TAB_DATA_HR[1],OSV3_TAB_DATA_HR[2],OSV3_TAB_DATA_HR[3],OSV3_TAB_DATA_HR[4],OSV3_TAB_DATA_HR[5],OSV3_TAB_DATA_HR[6],OSV3_TAB_DATA_HR[7],OSV3_TAB_DATA_HR[8],OSV3_TAB_DATA_HR[9],OSV3_TAB_DATA_HR[10],OSV3_TAB_DATA_HR[11],OSV3_TAB_DATA_HR[12]);
			
					OSV3_MANCHESTER_SEND_DATA_THR();
			
				break;
				case THGR2228N : 
					//OSV23Binaire.HeureAcqui=seconds;
					//OSV23Binaire.Temperature=VAL_TempC;
					//OSV23Binaire.HR=VAL_HR;
					//OSV23Binaire.RollingCode=63;
					//OSV23Binaire.numero=4;
					//OSV23Binaire.type_capteur=THGR2228N;
					FTDI.printf("\r\n Construction de la Trame de THGR2228N");

					//FTDI.printf("\r\n\n Transmission de la trame en OSV2 :T=%3.2f°C | HR=%d%%",VAL_TempC,VAL_HR);

					SEND_LOW();
					
					SetType(OSV2_MessageBuffer_THR, ID_THR);
					SetChannel(OSV2_MessageBuffer_THR,OSV2_CANAL_TEMP_HR << 4);//0x70
					//Mise en place du ROLLING CODE - SetId
					SetId(OSV2_MessageBuffer_THR, (unsigned char)Tab_OSV23Binaire.Data_Capteur.RollingCode); //0xCB ou 0xCC
					SetBatteryLevel(OSV2_MessageBuffer_THR,1);
					SetTemperature(OSV2_MessageBuffer_THR, Tab_OSV23Binaire.Data_Capteur.Temperature);
					SetHumidity(OSV2_MessageBuffer_THR, (unsigned char)Tab_OSV23Binaire.Data_Capteur.HR);
					calculateAndSetChecksum_THR(OSV2_MessageBuffer_THR);
					SendOregon(OSV2_MessageBuffer_THR, sizeof(OSV2_MessageBuffer_THR));

					SEND_LOW();

					wait_us(OSV2_TWOTIME*8);
					//SendOregon(OSV2_MessageBuffer, sizeof(OSV2_MessageBuffer));

					SEND_LOW();

					//Affichage complet
					//FTDI.printf("\r\n THGR2228N: FF|FF|%x|%x|%x|%x|%x|%x|%x|%x|%x|00",OSV2_MessageBuffer_THR[0],OSV2_MessageBuffer_THR[1],OSV2_MessageBuffer_THR[2],OSV2_MessageBuffer_THR[3],OSV2_MessageBuffer_THR[4],OSV2_MessageBuffer_THR[5],OSV2_MessageBuffer_THR[6],OSV2_MessageBuffer_THR[7],OSV2_MessageBuffer_THR[8]);
				break;
				case UVN800 : 	 
					//OSV23Binaire.HeureAcqui=seconds;
					//OSV23Binaire.UV=VAL_UV;
					//OSV23Binaire.RollingCode=71;
					//OSV23Binaire.numero=6;
					//OSV23Binaire.type_capteur=UVN800;
					FTDI.printf("\r\n Construction de la Trame de UVN800");

					//FTDI.printf("\r\n\n Transmission de la trame en OSV3 :UV=%2.0f",VAL_UV);
					//Mise en place du canal en OSV3_TAB_DATA_UV[5] : 1 à 15
					OSV3_TAB_DATA_UV[5]|=OSV3_CANAL_UV&0x0F;
					//Mise en place du ROLLING CODE en OSV3_TAB_DATA_UV[6]
					OSV3_TAB_DATA_UV[6]=(unsigned char)Tab_OSV23Binaire.Data_Capteur.RollingCode;
					OSV3_TAB_DATA_UV[7]|=((int)Tab_OSV23Binaire.Data_Capteur.UV)&0x0F;
					OSV3_TAB_DATA_UV[10]=OSV3_CALC_CHECKSUM_UV();
					OSV3_TAB_DATA_UV[11]=OSV3_CALC_CRC_UV(&Fait_Init_TAB_CRC);
					OSV3_MANCHESTER_SEND_DATA_UV();

					//Affichage complet
					//FTDI.printf("\r\n UVN800: %x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x",OSV3_TAB_DATA_UV[0],OSV3_TAB_DATA_UV[1],OSV3_TAB_DATA_UV[2],OSV3_TAB_DATA_UV[3],OSV3_TAB_DATA_UV[4],OSV3_TAB_DATA_UV[5],OSV3_TAB_DATA_UV[6],OSV3_TAB_DATA_UV[7],OSV3_TAB_DATA_UV[8],OSV3_TAB_DATA_UV[9],OSV3_TAB_DATA_UV[10],OSV3_TAB_DATA_UV[11]);
			
			
				break;
				case BTHR918N : 
					//OSV23Binaire.HeureAcqui=seconds;
					//OSV23Binaire.Temperature=VAL_TempC;
					//OSV23Binaire.HR=VAL_HR;
					//OSV23Binaire.Pression=Pression;
					//OSV23Binaire.RollingCode=131;
					//OSV23Binaire.numero=9;
					//OSV23Binaire.type_capteur=BTHR918N;
					FTDI.printf("\r\n Construction de la Trame de BTHR918N");

					//FTDI.printf("\r\n\n Transmission de la trame en OSV2 :T=%3.2f°C | HR=%d%% | P=%4.0f",VAL_TempC,VAL_HR,Pression);

					SEND_LOW();

					SetType(OSV2_MessageBuffer_THRP, ID_THRP);
					SetChannel(OSV2_MessageBuffer_THRP,OSV2_CANAL_TEMP_HR_P << 4);//0x70
					//Mise en place du ROLLING CODE - SetId
					SetId(OSV2_MessageBuffer_THRP, (unsigned char)Tab_OSV23Binaire.Data_Capteur.RollingCode); //0xCB ou 0xCC 
					SetBatteryLevel(OSV2_MessageBuffer_THRP,1);
					SetTemperature(OSV2_MessageBuffer_THRP, Tab_OSV23Binaire.Data_Capteur.Temperature);
					SetHumidity(OSV2_MessageBuffer_THRP, (unsigned char)Tab_OSV23Binaire.Data_Capteur.HR);
					SetPressure(OSV2_MessageBuffer_THRP, (float)Tab_OSV23Binaire.Data_Capteur.Pression);
					calculateAndSetChecksum_THRP(OSV2_MessageBuffer_THRP);
					SendOregon(OSV2_MessageBuffer_THRP, sizeof(OSV2_MessageBuffer_THRP));

					SEND_LOW();

					wait_us(OSV2_TWOTIME*8);
					//SendOregon(OSV2_MessageBuffer_THRP, sizeof(OSV2_MessageBuffer_THRP));

					SEND_LOW();

					//Affichage complet
					//FTDI.printf("\r\n BTHR918N: FF|FF|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|00",OSV2_MessageBuffer_THRP[0],OSV2_MessageBuffer_THRP[1],OSV2_MessageBuffer_THRP[2],OSV2_MessageBuffer_THRP[3],OSV2_MessageBuffer_THRP[4],OSV2_MessageBuffer_THRP[5],OSV2_MessageBuffer_THRP[6],OSV2_MessageBuffer_THRP[7],OSV2_MessageBuffer_THRP[8],OSV2_MessageBuffer_THRP[9],OSV2_MessageBuffer_THRP[10]);
					
				break;
				case THN132N :  
					//OSV23Binaire.HeureAcqui=seconds;
					//OSV23Binaire.Temperature=VAL_TempC;
					//OSV23Binaire.RollingCode=29;
					//OSV23Binaire.numero=2;
					//OSV23Binaire.type_capteur=THN132N;
					FTDI.printf("\r\n Construction de la Trame de THN132N");

					//FTDI.printf("\r\n\n Transmission de la trame en OSV2 :T=%3.2f°C",VAL_TempC);

					SEND_LOW();

					SetType(OSV2_MessageBuffer_T, ID_T);
					SetChannel(OSV2_MessageBuffer_T,OSV2_CANAL_TEMP << 4); //0x70
					//Mise en place du ROLLING CODE - SetId
					SetId(OSV2_MessageBuffer_T, (unsigned char)Tab_OSV23Binaire.Data_Capteur.RollingCode); //0xCB ou 0xCC 
					SetBatteryLevel(OSV2_MessageBuffer_T,1);
					SetTemperature(OSV2_MessageBuffer_T, Tab_OSV23Binaire.Data_Capteur.Temperature);
					calculateAndSetChecksum_T(OSV2_MessageBuffer_T);
					SendOregon(OSV2_MessageBuffer_T, sizeof(OSV2_MessageBuffer_T));

					SEND_LOW();

					wait_us(OSV2_TWOTIME*8);
					//SendOregon(OSV2_MessageBuffer, sizeof(OSV2_MessageBuffer));

					SEND_LOW();

					//Affichage complet
					//FTDI.printf("\r\n THN132N: FF|FF|%x|%x|%x|%x|%x|%x|%x|%x|00",OSV2_MessageBuffer_THR[0],OSV2_MessageBuffer_THR[1],OSV2_MessageBuffer_THR[2],OSV2_MessageBuffer_THR[3],OSV2_MessageBuffer_THR[4],OSV2_MessageBuffer_THR[5],OSV2_MessageBuffer_THR[6],OSV2_MessageBuffer_THR[7]);
								
				break;
			}
			HC12_IT_Flag = false;
		}
	}
}

//---------------------------------------------------------------------------

//Codage des fonctions

//Fonction pour OSV3
unsigned char OSV3_CALC_CRC(bool *InitFait)
{
	unsigned char CRC=0;
	
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	//Inclure les octets de l'indice 4 à 10. Ne pas inclure le CHECKSUM à l'indice 11
	
	OSV3_INIT_CRC(InitFait);
	
	CRC=OSV3_TAB_CRC_INIT[OSV3_TAB_DATA_HR[3] & 0x0F];
	//ne sert à rien
	//CRC=CRC & 0xFF;
	
	for (int i=4 ; i<11 ; i++)
	{ 
		CRC=OSV3_TAB_CRC_INIT[CRC ^ OSV3_TAB_DATA_HR[i]];
		//ne sert à rien
		//CRC=CRC & 0xFF;
	}
	//Permutation des NIBBLES
	return( ((CRC & 0x0F)<<4) | ((CRC & 0xF0)>>4) );
	
}



//Capteur T, HR et P
unsigned char OSV3_CALC_CRC_UV(bool *InitFait)
{
	unsigned char CRC=0;
	
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	//Inclure les octets de l'indice 4 à 9. Ne pas inclure le CHECKSUM à l'indice 10
	
	OSV3_INIT_CRC(InitFait);
	
	CRC=OSV3_TAB_CRC_INIT[OSV3_TAB_DATA_UV[3] & 0x0F];
	//ne sert à rien
	//CRC=CRC & 0xFF;
	
	for (int i=4 ; i<10 ; i++)
	{ 
		CRC=OSV3_TAB_CRC_INIT[CRC ^ OSV3_TAB_DATA_UV[i]];
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

//Capteur T et HR
unsigned char OSV3_CALC_CHECKSUM_THR(void)
{
	unsigned char CheckSum;
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	
	CheckSum = ( OSV3_TAB_DATA_HR[3] & 0x0F );

	//Inclure les octets de 4 à 10
	for(int i=4 ; i <= OSV3_TAILLE_DATA_CAPTEUR_THR-3 ; i++)
	{
		CheckSum = CheckSum + (OSV3_TAB_DATA_HR[i] & 0x0F) + ((OSV3_TAB_DATA_HR[i]>>4) & 0x0F);
	}
	
	//Permutation des NIBBLES
	return( ((CheckSum & 0x0F)<<4) | ((CheckSum & 0xF0)>>4) );
	
}

//Capteur UVN800
unsigned char OSV3_CALC_CHECKSUM_UV(void)
{
	unsigned char CheckSum;
	//Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	
	CheckSum = ( OSV3_TAB_DATA_UV[3] & 0x0F );

	//Inclure les octets de 4 à 10
	for(int i=4 ; i <= OSV3_TAILLE_DATA_CAPTEUR_UV-3 ; i++)
	{
		CheckSum = CheckSum + (OSV3_TAB_DATA_UV[i] & 0x0F) + ((OSV3_TAB_DATA_UV[i]>>4) & 0x0F);
	}
	
	//Permutation des NIBBLES
	return( ((CheckSum & 0x0F)<<4) | ((CheckSum & 0xF0)>>4) );
	
}



//Trame pour capteur T et HR
void OSV3_CONSTRUIRE_TRAME_THR(float Temp_f, int HumiHR)
{
	//Les nibbles sont envoyés LSB first
	
	//Preambule du protocole OSV3
	//24 bits à 1 -> 6 nibbles
	OSV3_TAB_DATA_HR[0]=0xFF;
	OSV3_TAB_DATA_HR[1]=0xFF;
	OSV3_TAB_DATA_HR[2]=0xFF;
	//nibble de synchro -> 0101 -> LSB en 1er soit 0xA0
	OSV3_TAB_DATA_HR[3]=0xA0;
	
	//Trame de données du capteur THGR810 -> payload
	//les nibbles 0..3 sont l'ID du capteur qui est unique pour chaque capteur ou commun pour
	//un groupe de capteur.
	//Ici ID du capteur est F824 dans l'ordre de reception
	OSV3_TAB_DATA_HR[3]=OSV3_TAB_DATA_HR[3] | 0x0F;
	OSV3_TAB_DATA_HR[4]=0x82;
	OSV3_TAB_DATA_HR[5]=0x40;
	
	//le nibble 4 pour le CANAL de 1 à 15  
	//Insertion du CANAL
	OSV3_TAB_DATA_HR[5]=OSV3_TAB_DATA_HR[5] | OSV3_CANAL_TEMP_HR;

	//Les nibbles 5..6 pour le code tournant dont la valeur est aleatoire
	//à chaque reset du capteur : exemple changement de piles.
	//OSV3_TAB_DATA[6]=OSV3_ROLLING_CODE();
	OSV3_TAB_DATA_HR[6]=VAL_ROLLING_CODE;
	//Capteur avec bit d'etat de la batterie -> toujours à 0 pour batterie chargée
	//valeur à 1 lorsque la batterie est à changer
	//A changer par une variable pour evolution.
	OSV3_TAB_DATA_HR[7]=0x80;
	
	//Les nibbles 8..[N-5] sont les données du capteur
	//Les nibbles 10..8 sont la temperature avec 1 LSB qui represente 0.1 °C
	//exemple : un float de 23.5°C est à transformer en entier de 235
	int temp=(int)(Temp_f*10);
	//Extraction de 5 de 23.5°C
	OSV3_TAB_DATA_HR[7]=OSV3_TAB_DATA_HR[7] | ( (abs(temp) % 10) & 0x0F );
	//Extraction de 3 de 23.5°C
	OSV3_TAB_DATA_HR[8]=((abs(temp)/10) % 10) << 4;
	//Extraction de 2 de 23.5°C
	OSV3_TAB_DATA_HR[8]=OSV3_TAB_DATA_HR[8] | ((abs(temp)/100) & 0x0F);
	//Le nibble 11 represente le signe de la temperature -> une valeur differente de 0 est 
	//une temperature negative
	OSV3_TAB_DATA_HR[9]=(Temp_f <0) ? 0x80 : 0;
	//Extraction de HD en %
	OSV3_TAB_DATA_HR[9]=OSV3_TAB_DATA_HR[9] | ((HumiHR % 10) & 0x0F);
	OSV3_TAB_DATA_HR[10]=((HumiHR /10) % 10) <<4 ;
	//Placement du CHECKSUM
	//Le resultat de la somme sur 8 bits des nibbles 0..[N-5]
	//Le CHECKSUM est placé dans [N-3]..[N-4]
	OSV3_TAB_DATA_HR[11]=OSV3_CALC_CHECKSUM_THR();
	//Placement du CRC
	OSV3_TAB_DATA_HR[12]=OSV3_CALC_CRC(&Fait_Init_TAB_CRC);
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
	
	//Les bits sont transmis de 4 à 7 puis de 0 à 3
	for(int i=0 ; i<8 ; i++)
	{
		TimeBase=TimeBase+DureeDesire;
		
		unsigned int CalculRetard=TimeBase-TimeManchester.read_us();
		
		//FTDI.printf("\r\n\n CalculRetard=%d | TimeBase=%d",CalculRetard,TimeBase);
		
		
		if(CalculRetard > 2*DureeDesire)  {
																				//Retard trop grand indique un break entre la transmission : reset de TimeBase
																				TimeBase=TimeManchester.read_us();
																				//FTDI.printf("\r\n\n CalculRetard > 2*DureeDesire ->TimeBase=%d",TimeBase);
																			}
		else {
						if(CalculRetard > 0) wait_us(CalculRetard);
						//FTDI.printf("\r\n\n CalculRetard > 0 ->TimeBase=%d",CalculRetard);
				 }
		
		
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

void OSV3_MANCHESTER_SEND_DATA_THR(void)
{
	//Est-ce necessaire de faire un reset du Timer ?
	TimeManchester.reset();
	TimeManchester.start();
	
	//Ajouter mise sous tension du module TX en 433MHz
	//Prevoir une tempo de 60ms
	
	//Mise à 0 de la broche
	//Data_433=0;
	
	for(int i=0 ; i<OSV3_TAILLE_DATA_CAPTEUR_THR ; i++)
	{
		OSV3_MANCHESTER_ENCODE(OSV3_TAB_DATA_HR[i], i+1==OSV3_TAILLE_DATA_CAPTEUR_THR);

	}
	
	//Ajouter mise hors tension du module TX en 433MHz
	//Mise à 0 de la broche
	//Data_433=0;
	
	//Arret du Timer
	//A tester si necessaire suite à PowerDown de la Teensy 3.2
	TimeManchester.stop();
	
}



void OSV3_MANCHESTER_SEND_DATA_UV(void)
{
	//Est-ce necessaire de faire un reset du Timer ?
	TimeManchester.reset();
	TimeManchester.start();
	
	//Ajouter mise sous tension du module TX en 433MHz
	//Prevoir une tempo de 60ms
	
	//Mise à 0 de la broche
	//Data_433=0;
	
	for(int i=0 ; i<OSV3_TAILLE_DATA_CAPTEUR_UV ; i++)
	{
		OSV3_MANCHESTER_ENCODE(OSV3_TAB_DATA_UV[i], i+1==OSV3_TAILLE_DATA_CAPTEUR_UV);
	}
	
	//Ajouter mise hors tension du module TX en 433MHz
	//Mise à 0 de la broche
	//Data_433=0;
	
	//Arret du Timer
	//A tester si necessaire suite à PowerDown de la Teensy 3.2
	TimeManchester.stop();
	
}

/////////////////////////////////////////////////////////////////////////////
// Codage des fonctions pour OSV2
/////////////////////////////////////////////////////////////////////////////

void SendZero(void)
{
	SEND_HIGH();
	wait_us(OSV2_TIME);
	SEND_LOW();
	wait_us(OSV2_TWOTIME);
	SEND_HIGH();
	wait_us(OSV2_TIME);
}
void SendOne(void)
{
	SEND_LOW();
	wait_us(OSV2_TIME);
	SEND_HIGH();
	wait_us(OSV2_TWOTIME);
	SEND_LOW();
	wait_us(OSV2_TIME);
}
void SendQuarterMSB(unsigned char data)
{
	(bitRead(data, 4)) ? SendOne() : SendZero();
  (bitRead(data, 5)) ? SendOne() : SendZero();
  (bitRead(data, 6)) ? SendOne() : SendZero();
  (bitRead(data, 7)) ? SendOne() : SendZero();
}
void SendQuarterLSB(unsigned char data)
{
	(bitRead(data, 0)) ? SendOne() : SendZero();
  (bitRead(data, 1)) ? SendOne() : SendZero();
  (bitRead(data, 2)) ? SendOne() : SendZero();
  (bitRead(data, 3)) ? SendOne() : SendZero();
}
void SendData(unsigned char *data, unsigned char size)
{
	for(int i = 0; i < size; ++i)
  {
    SendQuarterLSB(data[i]);
    SendQuarterMSB(data[i]);
  }
}
void SendOregon(unsigned char *data, unsigned char size)
{
	SendPreamble();
  SendData(data, size);
  SendPostamble();
}
void SendPreamble(void)
{
	unsigned char PREAMBLE[]={0xFF,0xFF};
  SendData(PREAMBLE, 2);
}

void SendPostamble(void)
{
	unsigned char POSTAMBLE[]={0x00};
  SendData(POSTAMBLE, 1);
}

void SendSync(void)
{
	SendQuarterLSB(0xA);
}

void SetType(unsigned char *data, unsigned char *type)
{
	data[0] = type[0];
  data[1] = type[1];
}

void SetChannel(unsigned char *data, unsigned char channel)
{
	data[2] = channel;
}

void SetId(unsigned char *data, unsigned char ID)
{
	data[3] = ID;
}

void SetBatteryLevel(unsigned char *data, unsigned char level)
{
	if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}

void SetTemperature(unsigned char *data, float Temp)
{
	data[6] = (Temp <0) ? 0x08 : 0x00;
	
	int temp=(int)(abs(Temp)*10);
	data[5]=(temp/100) << 4;
	data[5]=data[5] | (((temp)/10) % 10);
	data[4]=(temp % 10) << 4;
}

void SetHumidity(unsigned char *data,unsigned char Hum)
{
	data[7] = (Hum / 10);
	data[6] |= (Hum % 10) << 4;
}

void SetPressure(unsigned char *data, float pres) 
{
  if ((pres > 850) && (pres < 1100)) 
		{
			data[8] = (int)(pres) - 856;
			data[9] = 0xC0;  
		}
}

unsigned char Sum(unsigned char count, unsigned char *data)
{
	int s = 0;
 
  for(int i = 0; i<count ; i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}

void calculateAndSetChecksum_THR(unsigned char *data)
{
	data[8] = ((Sum(8, data) - 0xa) & 0xFF);
}

void calculateAndSetChecksum_THRP(unsigned char *data)
{
	data[10] = ((Sum(10, data) - 0xa) & 0xFF);
}

void calculateAndSetChecksum_T(unsigned char *data)
{
	int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);
	data[6] |= (s&0x0F) << 4;
	data[7] = (s&0xF0) >> 4;
}

////////////////////////////////////////////////////////////////////////////
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
		//FTDI.printf("\r\nTX=%s | RX=%s",buffer,reponse);
		//FTDI.printf("\r\nTimeout=%fs",t.read() - debut);
		
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
// Codage des fonctions IT et HC12
/////////////////////////////////////////////////////////////////////////////
//Fonction IT associee au HC12
void HC12_IT_RX(void)
{
	//Allumage de la LED
	//LED_ON_Off=!LED_ON_Off;
	//LED_ON_Off=1;
	if(!First_IT_HC12) { 
											HC_12.getc();
											First_IT_HC12=true;
										 }
	else
		{
			do
			{
				Tab_OSV23Binaire.Tab_TU[CPT_IT_HC12]=HC_12.getc();
				// FTDI.printf("\r\nLecture de : %#x", Tab_OSV23Binaire.Tab_TU[CPT_IT_HC12]);
				CPT_IT_HC12++;
			}
			while( HC_12.readable() || CPT_IT_HC12!=sizeof(TU_DB));

	
	HC12_IT_Flag=true;
	
	//FTDI.printf("\r\nCPT_IT_HC12=%d",CPT_IT_HC12);
	
	//LED_ON_Off=0;
			
			
		}
	
	
}


