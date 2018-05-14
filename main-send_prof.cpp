// Teensy 3.2 USB Serial, test IT par Ticker

// Unplug then plug back in the Teensy after programing to reactivate the Teensy USB serial port.
// if your terminal program can't see the Teensy

#include "mbed.h"
#include "WakeUp.h"

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
// Fonction pour la gestion du mode SLEEP de la Tennsy 3.2
/////////////////////////////////////////////////////////////////////////////
void Teensy32_Back2Speed_After_PowerDown(void);

/////////////////////////////////////////////////////////////////////////////
// Sortie pour la gestion du HC12 par commande AT
/////////////////////////////////////////////////////////////////////////////
DigitalOut CS_HC12(PTB19,1);
//HC12-433
//						TX   RX
Serial HC_12(PTB17, PTB16);
 
//FTDI @ 921600bauds
//          TX,RX
Serial FTDI(PTD3,PTD2);


//Commande de la LED sur PTC5
DigitalOut LED_ON_Off(PTC5,0);





//---------------------------------------------------------------------------

//Pour tester l'emission en HC12
float VAL_TempC;
unsigned int VAL_HR,VAL_UV;



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

/////////////////////////////////////////////////////////////////////////////
// Declaration pour controler le niveau eau
/////////////////////////////////////////////////////////////////////////////
//Entree pour le signal periodique lorsque le niveau eau est atteind
InterruptIn Niv_Eau(PTA4);

//Fonction pour la gestion du niveau d'eau
void IT_Niveau_Eau(void);
//Variable pour le niveau d'eau
volatile bool IT_NIV_EAU=false;
volatile int CPT_IT_Niveau_Eau=0;

#define FIRST_IT_NIV_EAU 1
#define VAL_MAX_IT_NIV_EAU 2




int main(void) 
{

	unsigned int i=0;
	
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
	
	//Test pour transmettre en binaire de transmission en HC12 puis encodage en OSV2 et OSV3:
	//Struct associee aux champs - taille donnee par sizeof : 36
	//T_DB OSV23Binaire;
	//Union associee aux champs - taille donnee par sizeof : 36
	TU_DB Tab_OSV23Binaire;
	//Union reserve au niveau eau par IT
	TU_DB Tab_OSV23Binaire_IT;
	
/////////////////////////////////////////////////////////////////////////////
// Declaration pour controler le niveau eau
/////////////////////////////////////////////////////////////////////////////
//bool modifie par le programme de transmission
	bool Transmission_en_cours_HC12=false;
	bool Transmission_Niv_Eau_A_Faire_HC12=false;
	
//Configuration de l'IT pour le niveau eau
	Niv_Eau.rise(&IT_Niveau_Eau);
	Niv_Eau.mode(PullNone);
	
	
	
	FTDI.printf("\r\n\n Presser une touche pour continuer - HC12 TX avant OSV2_3 et SLEEP Tennsy 3.2.");	
	int c=FTDI.getc();
	
	FTDI.printf("\r\n\n Taille du type T_DB=%d | TU_DB=%d | float=%d | time_t=%d | int=%d | unsigned char=%d | enum list_OSV23=%d\r\n", sizeof(T_DB), sizeof(TU_DB), sizeof(float), sizeof(time_t), sizeof(int), sizeof(unsigned char), sizeof(T_Enum));
	
	FTDI.printf("\r\n\n Taille de Tab_OSV23Binaire=%d | Tab_OSV23Binaire.Tab_TU[]=%d\r\n", sizeof(Tab_OSV23Binaire), sizeof(Tab_OSV23Binaire.Tab_TU));	
	
	
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
												
												//Entrer en mode SLEEP -> "AT+SLEEP"
												sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
												sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
												if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+SLEEP");
												else FTDI.printf("\r\n sendATcommand=ERROR");
			
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
					
											
											
			
		//Acquisition des grandeurs :
		VAL_TempC=-27.2;
		VAL_HR=88;
		VAL_UV=5;
		float Pression=1031;
		float debut; 

	while(true)
	{
		
		if(IT_NIV_EAU) {
				
												switch(CPT_IT_Niveau_Eau)
												{
													case FIRST_IT_NIV_EAU 	:	//Memoriser une transmission à faire pour niveau eau
																										FTDI.printf("\r\n Memorisation transmission 1ere trame Alerte à faire");
																										Transmission_Niv_Eau_A_Faire_HC12=true;
																																								 
																										break;
																																								 
													case VAL_MAX_IT_NIV_EAU :	//Derniere IT autorisee
																										Niv_Eau.rise(NULL);
																										CPT_IT_Niveau_Eau=0;
																										FTDI.printf("\r\n Valeur de CPT_IT_Niveau_Eau atteinte");
																										//Nouvelle transmission trame niveau eau à faire par securite
																										Transmission_Niv_Eau_A_Faire_HC12=true;
													
																										break;
																							
													
												}
											 
											IT_NIV_EAU=false;
									}
		
			
			
			if (Transmission_Niv_Eau_A_Faire_HC12 && !Transmission_en_cours_HC12) 
			{//Transmettre trame
				//Reveil HC12
				//Transmission trame
				//Sleep HC12
				
				//Transmission d'une temperature de 75°C
				seconds=time(NULL);
				Tab_OSV23Binaire_IT.Data_Capteur.HeureAcqui=seconds;
				
				VAL_TempC=75;
				
				Tab_OSV23Binaire_IT.Data_Capteur.Temperature=VAL_TempC;
				//Rolling code specifique au niveau eau
				Tab_OSV23Binaire_IT.Data_Capteur.RollingCode=98;
				Tab_OSV23Binaire_IT.Data_Capteur.type_capteur=THN132N;
				FTDI.printf("\r\n Trame de THN132N et i=%d",i);
				
				//Transmission de la trame 
				FTDI.printf("\r\n Reveil du HC12");
				Enter_CMD_mode();
				Leave_CMD_mode();
				FTDI.printf("\r\n Fin Reveil du HC12");

				for(int i=0 ; i<sizeof(Tab_OSV23Binaire.Tab_TU) ; i++)
				{
					//Attente Buffer libre
					while(!HC_12.writeable());
					//Emission d'un caratere
					HC_12.putc(Tab_OSV23Binaire_IT.Tab_TU[i]);
					//Affichage par le maitre de l'emission
					//FTDI.printf("\r\nEmission de %c",(char)Tab_OSV23Binaire.Tab_TU[i]);
				}
				
				//Fin de Emission des données HC12
				//Mise en SLEEP du HC12
				//Attente 0.5s pour la fin transmission - 300ms max à partir de la mesure par Timer
				wait(0.5);
				
				FTDI.printf("\r\n Mise en veille du HC12");
				Enter_CMD_mode();
				
				
				//Entrer en mode AT -> "AT"
				sprintf(tab_HC_12_CMD_AT, HC12_AT);
				sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
				if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand en 1200bauds = OK");
				else FTDI.printf("\r\n sendATcommand=ERROR");
				
				
				//Entrer en mode SLEEP -> "AT+SLEEP"
				sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
				sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
				
				if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+SLEEP");
				else FTDI.printf("\r\n sendATcommand=ERROR");
				
				Leave_CMD_mode();
				
				
				FTDI.printf("\r\n Transmission trame Alerte faite");
				
				Transmission_Niv_Eau_A_Faire_HC12=false;
			}
			
																																						
																																						
																																						
																																						
		//Configuration du mode SLEEP pour la Teensy 3.2
		WakeUp::calibrate();
		
		//Reveil dans 10s
    WakeUp::set_ms(10000);
		
		//Apres cette instruction le uC s'arrete
		// FTDI.printf("\r\nSLEEP de la Teensy 3.2");
		// deepsleep();
		
		//Activation de l'IT au prochain reveil si desactive par ARRET_IT_Niv_Eau suite transmission x2 de la trame alerte niveau atteint.
		Niv_Eau.rise(&IT_Niveau_Eau);
		
		// Teensy32_Back2Speed_After_PowerDown();
		// FTDI.printf("\r\nReveil de la Tennsy 3.2 apres 10s");
		
		
		
		if(IT_NIV_EAU) {
				
												switch(CPT_IT_Niveau_Eau)
												{
													case FIRST_IT_NIV_EAU 	:	//Memoriser une transmission à faire pour niveau eau
																										FTDI.printf("\r\n Memorisation transmission 1ere trame Alerte à faire");
																										Transmission_Niv_Eau_A_Faire_HC12=true;
																																								 
																										break;
																																								 
													case VAL_MAX_IT_NIV_EAU :	//Derniere IT autorisee
																										Niv_Eau.rise(NULL);
																										CPT_IT_Niveau_Eau=0;
																										FTDI.printf("\r\n Valeur de CPT_IT_Niveau_Eau atteinte");
																										//Nouvelle transmission trame niveau eau à faire par securite
																										Transmission_Niv_Eau_A_Faire_HC12=true;
													
																										break;
																							
													
												}
											 
											IT_NIV_EAU=false;
									}
		
		
	if(Transmission_Niv_Eau_A_Faire_HC12 && !Transmission_en_cours_HC12) 	{//Transmettre trame
																																						 //Reveil HC12
																																						 //Transmission trame
																																						 //Sleep HC12
				
																																						 //Transmission d'une temperature de 75°C
																																						 seconds=time(NULL);
																																						 Tab_OSV23Binaire_IT.Data_Capteur.HeureAcqui=seconds;
											
																																						 VAL_TempC=75;
											
																																						 Tab_OSV23Binaire_IT.Data_Capteur.Temperature=VAL_TempC;
																																						 //Rolling code specifique au niveau eau
																																						 Tab_OSV23Binaire_IT.Data_Capteur.RollingCode=98;
																																						 Tab_OSV23Binaire_IT.Data_Capteur.type_capteur=THN132N;
																																						 FTDI.printf("\r\n Trame de THN132N et i=%d",i);
				
																																						 //Transmission de la trame 
																																						 FTDI.printf("\r\n Reveil du HC12");
																																						 Enter_CMD_mode();
																																						 Leave_CMD_mode();
																																						 FTDI.printf("\r\n Fin Reveil du HC12");
		
		
		
																																						 for(int i=0 ; i<sizeof(Tab_OSV23Binaire.Tab_TU) ; i++)
																																							{
																																								//Attente Buffer libre
																																								while(!HC_12.writeable());
																																								//Emission d'un caratere
																																								HC_12.putc(Tab_OSV23Binaire_IT.Tab_TU[i]);
																																								//Affichage par le maitre de l'emission
																																								//FTDI.printf("\r\nEmission de %c",(char)Tab_OSV23Binaire.Tab_TU[i]);
																																							}
		
																																						//Fin de Emission des données HC12
																																						//Mise en SLEEP du HC12
																																						//Attente 0.5s pour la fin transmission - 300ms max à partir de la mesure par Timer
																																						wait(0.5);
		
																																						FTDI.printf("\r\n Mise en veille du HC12");
																																						Enter_CMD_mode();
			
			
																																						//Entrer en mode AT -> "AT"
																																						sprintf(tab_HC_12_CMD_AT, HC12_AT);
																																						sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
																																						if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand en 1200bauds = OK");
																																						else FTDI.printf("\r\n sendATcommand=ERROR");
		
			
																																						//Entrer en mode SLEEP -> "AT+SLEEP"
																																						sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
																																						sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
																																						if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+SLEEP");
																																						else FTDI.printf("\r\n sendATcommand=ERROR");
			
																																						Leave_CMD_mode();
				
				
																																						FTDI.printf("\r\n Transmission trame Alerte faite");
																																						
																																						Transmission_Niv_Eau_A_Faire_HC12=false;
																																						}
		
		
		//Selection du capteur à transmettre
		switch (i++)
		{
			case 1 :	Tab_OSV23Binaire.Data_Capteur.type_capteur=THGR810;
								break;
			case 2 :	Tab_OSV23Binaire.Data_Capteur.type_capteur=THGR2228N;
								break;
			case 3 :	Tab_OSV23Binaire.Data_Capteur.type_capteur=UVN800;
								break;
			case 4 :	Tab_OSV23Binaire.Data_Capteur.type_capteur=BTHR918N;
								break;
			case 5 :	Tab_OSV23Binaire.Data_Capteur.type_capteur=THN132N;
								break;
			default : i=1;
								break;
		}
		
		
		switch (Tab_OSV23Binaire.Data_Capteur.type_capteur)
		{
			case THGR810 : 	seconds=time(NULL);
											Tab_OSV23Binaire.Data_Capteur.HeureAcqui=seconds;
			
											VAL_TempC=1.2;
											VAL_HR=21;
			
											Tab_OSV23Binaire.Data_Capteur.Temperature=VAL_TempC;
											Tab_OSV23Binaire.Data_Capteur.HR=VAL_HR;
											Tab_OSV23Binaire.Data_Capteur.RollingCode=162;
											Tab_OSV23Binaire.Data_Capteur.type_capteur=THGR810;
											FTDI.printf("\r\n Trame de THGR810 et i=%d",i);
											break;
			
			case THGR2228N : seconds=time(NULL);
											 Tab_OSV23Binaire.Data_Capteur.HeureAcqui=seconds;
			
											 VAL_TempC=6.5;
											 VAL_HR=87;
			
											 Tab_OSV23Binaire.Data_Capteur.Temperature=VAL_TempC;
											 Tab_OSV23Binaire.Data_Capteur.HR=VAL_HR;
											 Tab_OSV23Binaire.Data_Capteur.RollingCode=63;
											 Tab_OSV23Binaire.Data_Capteur.type_capteur=THGR2228N;
											 FTDI.printf("\r\n Trame de THGR2228N et i=%d",i);
											break;
											
			case UVN800 : 	 seconds=time(NULL);
											 Tab_OSV23Binaire.Data_Capteur.HeureAcqui=seconds;
											 
											 VAL_UV=4;
											 
											 Tab_OSV23Binaire.Data_Capteur.UV=VAL_UV;
											 Tab_OSV23Binaire.Data_Capteur.RollingCode=71;
											 Tab_OSV23Binaire.Data_Capteur.type_capteur=UVN800;
											 FTDI.printf("\r\n Trame de UVN800 et i=%d",i);
											break;
											
			case BTHR918N : seconds=time(NULL);
											Tab_OSV23Binaire.Data_Capteur.HeureAcqui=seconds;
											
											VAL_TempC=-13.6;
											VAL_HR=51;
											Pression=1022;
											
											Tab_OSV23Binaire.Data_Capteur.Temperature=VAL_TempC;
											Tab_OSV23Binaire.Data_Capteur.HR=VAL_HR;
											Tab_OSV23Binaire.Data_Capteur.Pression=Pression;
											Tab_OSV23Binaire.Data_Capteur.RollingCode=131;
											Tab_OSV23Binaire.Data_Capteur.type_capteur=BTHR918N;
											FTDI.printf("\r\n Trame de BTHR918N et i=%d",i);
											break;
									
			case THN132N :  seconds=time(NULL);
											Tab_OSV23Binaire.Data_Capteur.HeureAcqui=seconds;
											
											VAL_TempC=1.9;
											
											Tab_OSV23Binaire.Data_Capteur.Temperature=VAL_TempC;
											Tab_OSV23Binaire.Data_Capteur.RollingCode=29;
											Tab_OSV23Binaire.Data_Capteur.type_capteur=THN132N;
											FTDI.printf("\r\n Trame de THN132N et i=%d",i);
											break;
		  
		}
		
		
		
		//Construction de la trame par la valeur des capteurs
		//Struct associee aux champs
		//T_DB OSV23Binaire;
		//Union associee aux champs
		//TU_DB Tab_OSV23Binaire;
		//									T HR			T HR			UV			 T HR P		 T
		//enum list_OSV23 {THGR810, THGR2228N, UVN800 , BTHR918N, THN132N};
		//Acquisition de l'heure
		//Acquisition de l'heure interne
		//seconds=time(NULL);
		//OSV23Binaire.HeureAcqui=seconds;
		//OSV23Binaire.Temperature=VAL_TempC;
		//OSV23Binaire.HR=VAL_HR;
		//OSV23Binaire.RollingCode=162;
		//OSV23Binaire.numero=2;
		//OSV23Binaire.type_capteur=THGR810;
		
		//Emission des données HC12
		//Reveil du HC12
		//Cf. Doc.
		
		//Indicateur de debut de transmission
		//Necessaire pour le Niveau Eau
		Transmission_en_cours_HC12=true;
		
		FTDI.printf("\r\n Reveil du HC12");
		Enter_CMD_mode();
		Leave_CMD_mode();
		FTDI.printf("\r\n Fin Reveil du HC12");
		
		//Mesure de la duree de transmission de la trame
		Timer t;
		t.reset();
		t.start();
	
    debut = t.read();
		//Allumage de la LED
		LED_ON_Off=!LED_ON_Off;
		
		for(int i=0 ; i<sizeof(Tab_OSV23Binaire.Tab_TU) ; i++)
			{
				//Attente Buffer libre
				while(!HC_12.writeable());
				//Emission d'un caratere
				HC_12.putc(Tab_OSV23Binaire.Tab_TU[i]);
				//Affichage par le maitre de l'emission
				//FTDI.printf("\r\nEmission de %c",(char)Tab_OSV23Binaire.Tab_TU[i]);
			}
		
		//Fin de Emission des données HC12
		t.stop();
		FTDI.printf("\r\n Duree transmission de la trame : %f s",t.read()-debut);
			
			
		//Test esxtraction des elements transmis du tableau
		//float VAL_TempC;
		//unsigned int VAL_HR,VAL_UV;
		//int Pression=1031;
			
		/*
		//Test en pre remplissant :
		seconds=time(NULL);
		Tab_OSV23Binaire.Data_Capteur.HeureAcqui=seconds;
		Tab_OSV23Binaire.Data_Capteur.Temperature=-15.2;
		Tab_OSV23Binaire.Data_Capteur.HR=21;
		Tab_OSV23Binaire.Data_Capteur.Pression=1023;
		Tab_OSV23Binaire.Data_Capteur.RollingCode=123;
		Tab_OSV23Binaire.Data_Capteur.type_capteur=THN132N;
		*/
		
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
		FTDI.printf("\r\n OK1");
			
		//Extinction de la LED
		LED_ON_Off=!LED_ON_Off;
		FTDI.printf("\r\n OK2");
		//Mise en SLEEP du HC12
		//Attente 0.5s pour la fin transmission - 300ms max à partir de la mesure par Timer
		// wait_ms(500);
		
		FTDI.printf("\r\n OK3");
		//Test avec SLEEP
			
		FTDI.printf("\r\n Mise en veille du HC12");
		Enter_CMD_mode();
			
			
		//Entrer en mode AT -> "AT"
		sprintf(tab_HC_12_CMD_AT, HC12_AT);
		sprintf(tab_HC_12_reponse_AT, HC12_RESP_OK);
			
		if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand en 1200bauds = OK");
		else FTDI.printf("\r\n sendATcommand=ERROR");
		
			
		//Entrer en mode SLEEP -> "AT+SLEEP"
		sprintf(tab_HC_12_CMD_AT, HC12_SLEEP);
		sprintf(tab_HC_12_reponse_AT, HC12_SLEEP_Rep);
			
		if(sendATcommand(tab_HC_12_CMD_AT, tab_HC_12_reponse_AT , HC12_TIMEOUT)) FTDI.printf("\r\n sendATcommand=AT+SLEEP");
		else FTDI.printf("\r\n sendATcommand=ERROR");
			
		Leave_CMD_mode();
		
		//Indicateur de fin de transmission
		//Necessaire pour le Niveau Eau
		Transmission_en_cours_HC12=false;
								
		
	}
	  
}

//---------------------------------------------------------------------------

//Codage des fonctions

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


////////////////////////////////////////////////////////////////////////////
// Codage des fonctions pour le niveau eau
/////////////////////////////////////////////////////////////////////////////


//Fonction pour la gestion du niveau d'eau
void IT_Niveau_Eau(void)
{
	//Allumage de la LED pour visualiser entree en IT 
	
	CPT_IT_Niveau_Eau++;
	
	IT_NIV_EAU=true;
}