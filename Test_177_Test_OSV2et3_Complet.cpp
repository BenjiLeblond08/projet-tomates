/**
 * Teensy 3.2 USB Serial, test IT par Ticker
 * 
 * Unplug then plug back in the Teensy after programing to reactivate the Teensy USB serial port.
 * if your terminal program can't see the Teensy
 */

#include "mbed.h"
#include "WakeUp.h"

// include to check/display clock rates
#include "clk_freqs.h"
//FTDI.printf("\r\n core %d", SystemCoreClock);
//FTDI.printf("\n Bus  %d", bus_frequency());
//FTDI.printf("\n Osc  %d", extosc_frequency());


/******************************************************************************
 * Gestion du 433MHz en Manchester
 *****************************************************************************/
#define OSV3_CRC8_M433_GP 0x107//x^8+x^2+x+1
#define OSV3_CRC8_M433_DI 0x07
// Taille du tableau pour les données du capteur 
// Le capteur est temperature en °C et HR en % -> protocole Oregon Scientific V3 : THGR810
#define OSV3_TAILLE_DATA_CAPTEUR_THR 13
// Le capteur d'UV  -> protocole Oregon Scientific V3 : UVN800
#define OSV3_TAILLE_DATA_CAPTEUR_UV 12
// Possibilité de definir un canal de 1 à 15 : Oregon de 1 à 3
#define OSV3_CANAL_TEMP_HR 2
#define OSV3_CANAL_TEMP_HRP 2
#define OSV3_CANAL_HAUTEUR_EAU 3
#define DUREE_ENTRE_CHAQUE_MESURE_OSV3 9

/******************************************************************************
 * Gestion du 433MHz en Manchester - encodage OSV2
 *****************************************************************************/
#define OSV2_TIME 512
#define OSV2_TWOTIME OSV2_TIME*2
#define SEND_HIGH() Data_433 = 1;
#define SEND_LOW() Data_433 = 0;
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

// Virtual serial port over USB
//USBSerial PC;
//FTDI @ 921600bauds
//          TX, RX
Serial FTDI(PTD3, PTD2);


// Commande de la LED sur PTC5
DigitalOut LED_ON_Off(PTC5, 0);

// Declaration du Ticker toutes les 30s
//Ticker Toutes_30s;



/******************************************************************************
 * Gestion du 433MHz en Manchester - encodage OSV2
 *****************************************************************************/
Timer TimeManchester;
// Entrée analogique pour le CAN
AnalogIn AIN(PTC8);

/******************************************************************************
 * Gestion du 433MHz en Manchester - encodage OSV2
 *****************************************************************************/
DigitalOut Data_433(PTC2, 0);

/******************************************************************************
 * Gestion du 433MHz en Manchester - encodage OSV2
 *****************************************************************************/
// NL0 pour test module 433 -> coupure alimentation
DigitalOut POWER(PTD7, 0);


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

//---------------------------------------------------------------------------

/**
 * Gestion du 433MHz en Manchester -> variables globales
 * 
 * https://sourceforge.net/p/wmrx00/discussion/855159/thread/3358e4e8/
 * 
 * I got a BTHGN129 Temp, Hum and Pressure sensor, signal is reconize by WXSheild :
 * 12/01/2015 08:13:14 (UTC) - *OS2:5D5322F008108605D1295, 3FF, 00
 * https://github.com/deennoo/rtl_osv21
 * http://www.mattlary.com/2012/06/23/weather-station-project/
 * https://gist.github.com/RouquinBlanc/5cb6ff88cd02e68d48ea
 * https://www.weather-watch.com/smf/index.php?topic = 56997.0
 * OSV3 - UVN800
 * https://github.com/letscontrolit/NodoClassic/blob/master/RFLinkNRF/Plugins/Plugin_048.c
 * FF FF FF AD 87 41 64 80 0D 60 64 11
 * 
 *  packet[0] = 0xFF;
 *  packet[1] = 0xFF;
 *  packet[2] = 0xFF;
 *  packet[3] = 0xAD;
 *  packet[4] = 0x87;
 *  packet[5] = 0x41;
 *  packet[6] = 0x64;    // rolling code
 *  packet[7] = 0x80;
 *  packet[8] = 0x0D;
 *  packet[9] = 0x60;
 *  packet[10] = 0x64;   // checksum
 *  packet[11] = 0x11;   // not included in the checksum
 * 
 */

// Tableau pour stocker tout le protole OSV3
unsigned char OSV3_TAB_DATA_HR[OSV3_TAILLE_DATA_CAPTEUR_THR];
unsigned char OSV3_TAB_DATA_UV[OSV3_TAILLE_DATA_CAPTEUR_UV] ={0xFF, 0xFF, 0xFF, 0xAD, 0x87, 0x41, 0x57, 0x80, 0x0D, 0x60, 0x64, 0x11};
unsigned char OSV3_TAB_CRC_INIT[256];
unsigned char VAL_ROLLING_CODE;
bool Fait_Init_TAB_CRC = false;
// Pour tester l'emission en Manchester et 433MHz
float VAL_TempC, VAL_UV;
unsigned int VAL_HR;

// Pour codage OSV2
// Capteur T : THN132N
unsigned char OSV2_MessageBuffer_T[8];
// Capteur T et HR : THGR2228N
unsigned char OSV2_MessageBuffer_THR[9];
// Capteur T et HR et P : BTHR918N
unsigned char OSV2_MessageBuffer_THRP[11];
// ID pour THN132N
unsigned char ID_T[] ={0xEA, 0x4C};
// ID pour THGR2228N
unsigned char ID_THR[] ={0x1A, 0x2D};
// ID pour BTHR918N
unsigned char ID_THRP[] ={0x5A, 0x6D};

// FLIPFLOP
bool FlipFlop = true;


int main(void) 
{

	// Association de la fonction d'IT avec le Ticker toutes les 1s
	//Toutes_30s.attach(&IT_LED, DUREE_ENTRE_CHAQUE_MESURE_OSV3);

	//unsigned int i = 1;

	// Configuration vitesse FTDI
	FTDI.baud(921600);
	FTDI.format(8, SerialBase::None, 1);



	FTDI.printf("\r\n\n Presser une touche pour continuer - 433MHz - Manchester.");
	int c = FTDI.getc();

	//FTDI.printf("\r\n\n Entrer la valeur de T et HR : 23.5 41 ->");
	//FTDI.scanf("%f %d", &VAL_TempC, &VAL_HR);
	VAL_TempC =-12.5;
	VAL_HR = 47;
	VAL_UV = 9;
	float Pression = 1040;

	VAL_ROLLING_CODE = OSV3_ROLLING_CODE();
	FTDI.printf("\r\n\n Calcul du ROLLING Code ->%d", VAL_ROLLING_CODE);
	VAL_ROLLING_CODE = 144;
	FTDI.printf("\r\n\n ROLLING Code fixe implante->%d", VAL_ROLLING_CODE);

	// Boucle attente et affichage
	while(true)
	{
		WakeUp::calibrate();
		// Reveil dans 10s
		WakeUp::set_ms(8000);
		//wait(30);
		// Boucle attente
		//while(FLAG_IT_MESURE);
		//FLAG_IT_MESURE = true;

		// Pour attendre dans le while(FLAG);
		//FLAG_IT_MESURE = true;

		// Apres cette instruction le uC s'arrete
		deepsleep();

		Teensy32_Back2Speed_After_PowerDown();

		//LED1_ON_Off =!LED1_ON_Off;

		POWER = 1;

		wait(1);

		//FTDI.printf("\r\n core %d", SystemCoreClock);
		//FTDI.printf("\r\n Bus  %d", bus_frequency());
		//FTDI.printf("\r\n Osc  %d", extosc_frequency());

		LED_ON_Off = LED_ON_Off^1;
		//FTDI.printf("\r\n\n Construction de la trame :%d", i++);

		if(FlipFlop) {
			FTDI.printf("\r\n\n Transmission de la trame en OSV2 :T =%3.2f°C | HR =%d%%", VAL_TempC, VAL_HR);

			SEND_LOW();

			SetType(OSV2_MessageBuffer_THR, ID_THR);
			SetChannel(OSV2_MessageBuffer_THR, 0x70);
			SetId(OSV2_MessageBuffer_THR, 0xCB); //0xCC
			SetBatteryLevel(OSV2_MessageBuffer_THR, 1);
			SetTemperature(OSV2_MessageBuffer_THR, VAL_TempC);
			SetHumidity(OSV2_MessageBuffer_THR, VAL_HR);
			calculateAndSetChecksum_THR(OSV2_MessageBuffer_THR);
			SendOregon(OSV2_MessageBuffer_THR, sizeof(OSV2_MessageBuffer_THR));

			SEND_LOW();

			wait_us(OSV2_TWOTIME*8);
			//SendOregon(OSV2_MessageBuffer, sizeof(OSV2_MessageBuffer));

			SEND_LOW();

			// Affichage complet
			FTDI.printf("\r\n THGR2228N: FF|FF|%x|%x|%x|%x|%x|%x|%x|%x|%x|00", 
				OSV2_MessageBuffer_THR[0], 
				OSV2_MessageBuffer_THR[1], 
				OSV2_MessageBuffer_THR[2], 
				OSV2_MessageBuffer_THR[3], 
				OSV2_MessageBuffer_THR[4], 
				OSV2_MessageBuffer_THR[5], 
				OSV2_MessageBuffer_THR[6], 
				OSV2_MessageBuffer_THR[7], 
				OSV2_MessageBuffer_THR[8]
			);

			wait(1);

			OSV3_CONSTRUIRE_TRAME_THR(VAL_TempC, VAL_HR);
			FTDI.printf("\r\n\n Transmission de la trame en OSV3 :T =%3.2f°C | HR =%d%%", VAL_TempC, VAL_HR);

			// Affichage complet
			FTDI.printf("\r\n THGR810: %x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x", 
				OSV3_TAB_DATA_HR[0], 
				OSV3_TAB_DATA_HR[1], 
				OSV3_TAB_DATA_HR[2], 
				OSV3_TAB_DATA_HR[3], 
				OSV3_TAB_DATA_HR[4], 
				OSV3_TAB_DATA_HR[5], 
				OSV3_TAB_DATA_HR[6], 
				OSV3_TAB_DATA_HR[7], 
				OSV3_TAB_DATA_HR[8], 
				OSV3_TAB_DATA_HR[9], 
				OSV3_TAB_DATA_HR[10], 
				OSV3_TAB_DATA_HR[11], 
				OSV3_TAB_DATA_HR[12]
			);

			OSV3_MANCHESTER_SEND_DATA_THR();

			FTDI.printf("\r\n\n Transmission de la trame en OSV2 :T =%3.2f°C", VAL_TempC);

			SEND_LOW();

			SetType(OSV2_MessageBuffer_T, ID_T);
			SetChannel(OSV2_MessageBuffer_T, 0x70);
			SetId(OSV2_MessageBuffer_T, 0xCB); //0xCC
			SetBatteryLevel(OSV2_MessageBuffer_T, 1);
			SetTemperature(OSV2_MessageBuffer_T, VAL_TempC);
			calculateAndSetChecksum_T(OSV2_MessageBuffer_T);
			SendOregon(OSV2_MessageBuffer_T, sizeof(OSV2_MessageBuffer_T));

			SEND_LOW();

			wait_us(OSV2_TWOTIME*8);
			//SendOregon(OSV2_MessageBuffer, sizeof(OSV2_MessageBuffer));

			SEND_LOW();

			// Affichage complet
			FTDI.printf("\r\n THN132N: FF|FF|%x|%x|%x|%x|%x|%x|%x|%x|00", 
				OSV2_MessageBuffer_THR[0], 
				OSV2_MessageBuffer_THR[1], 
				OSV2_MessageBuffer_THR[2], 
				OSV2_MessageBuffer_THR[3], 
				OSV2_MessageBuffer_THR[4], 
				OSV2_MessageBuffer_THR[5], 
				OSV2_MessageBuffer_THR[6], 
				OSV2_MessageBuffer_THR[7]
			);

			wait(1);

		}
		else{
			FTDI.printf("\r\n\n Transmission de la trame en OSV2 :T =%3.2f°C | HR =%d%% | P =%4.0f", VAL_TempC, VAL_HR, Pression);

			SEND_LOW();

			SetType(OSV2_MessageBuffer_THRP, ID_THRP);
			SetChannel(OSV2_MessageBuffer_THRP, 0x70);
			SetId(OSV2_MessageBuffer_THRP, 0xCB); //0xCC
			SetBatteryLevel(OSV2_MessageBuffer_THRP, 1);
			SetTemperature(OSV2_MessageBuffer_THRP, VAL_TempC);
			SetHumidity(OSV2_MessageBuffer_THRP, VAL_HR);
			SetPressure(OSV2_MessageBuffer_THRP, Pression);
			calculateAndSetChecksum_THRP(OSV2_MessageBuffer_THRP);
			SendOregon(OSV2_MessageBuffer_THRP, sizeof(OSV2_MessageBuffer_THRP));

			SEND_LOW();

			wait_us(OSV2_TWOTIME*8);
			//SendOregon(OSV2_MessageBuffer_THRP, sizeof(OSV2_MessageBuffer_THRP));

			SEND_LOW();

			// Affichage complet
			FTDI.printf("\r\n BTHR918N: FF|FF|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|00", OSV2_MessageBuffer_THRP[0], OSV2_MessageBuffer_THRP[1], OSV2_MessageBuffer_THRP[2], OSV2_MessageBuffer_THRP[3], OSV2_MessageBuffer_THRP[4], OSV2_MessageBuffer_THRP[5], OSV2_MessageBuffer_THRP[6], OSV2_MessageBuffer_THRP[7], OSV2_MessageBuffer_THRP[8], OSV2_MessageBuffer_THRP[9], OSV2_MessageBuffer_THRP[10]);

			wait(1);

/*
			OSV3_CONSTRUIRE_TRAME_THRP(VAL_TempC, VAL_HR, Pression);
			FTDI.printf("\r\n\n Transmission de la trame en OSV3 :T =%3.2f°C | HR =%d%% | P =%4.0f", VAL_TempC, VAL_HR, Pression);

			// Affichage complet
			FTDI.printf("\r\n %x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|", OSV3_TAB_DATA_HRP[0], OSV3_TAB_DATA_HRP[1], OSV3_TAB_DATA_HRP[2], OSV3_TAB_DATA_HRP[3], OSV3_TAB_DATA_HRP[4], OSV3_TAB_DATA_HRP[5], OSV3_TAB_DATA_HRP[6], OSV3_TAB_DATA_HRP[7], OSV3_TAB_DATA_HRP[8], OSV3_TAB_DATA_HRP[9], OSV3_TAB_DATA_HRP[10], OSV3_TAB_DATA_HRP[11], OSV3_TAB_DATA_HRP[12], OSV3_TAB_DATA_HRP[13], OSV3_TAB_DATA_HRP[14]);

			OSV3_MANCHESTER_SEND_DATA_THRP();
*/

			// Transmission en OSV3 pour UVN800
			//OSV3_TAB_DATA_UV[OSV3_TAILLE_DATA_CAPTEUR_UV] ={0xFF, 0xFF, 0xFF, 0xAD, 0x87, 0x41, 0x64, 0x80, 0x0D, 0x60, 0x64, 0x11};
			//unsigned char OSV3_TAB_DATA_UV[OSV3_TAILLE_DATA_CAPTEUR_UV] ={0xFF, 0xFF, 0xFF, 0xAD, 0x87, 0x41, 0x64, 0x80, 0x0D, 0x60, 0x64, 0x11};

			FTDI.printf("\r\n\n Transmission de la trame en OSV3 :UV =%3f", VAL_UV);
			OSV3_TAB_DATA_UV[7]|=((int)VAL_UV)&0x0F;
			OSV3_TAB_DATA_UV[10] = OSV3_CALC_CHECKSUM_UV();
			OSV3_TAB_DATA_UV[11] = OSV3_CALC_CRC_UV(&Fait_Init_TAB_CRC);
			OSV3_MANCHESTER_SEND_DATA_UV();

			// Affichage complet
			FTDI.printf("\r\n UVN800: %x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x", OSV3_TAB_DATA_UV[0], OSV3_TAB_DATA_UV[1], OSV3_TAB_DATA_UV[2], OSV3_TAB_DATA_UV[3], OSV3_TAB_DATA_UV[4], OSV3_TAB_DATA_UV[5], OSV3_TAB_DATA_UV[6], OSV3_TAB_DATA_UV[7], OSV3_TAB_DATA_UV[8], OSV3_TAB_DATA_UV[9], OSV3_TAB_DATA_UV[10], OSV3_TAB_DATA_UV[11]);

			wait(1);
		}

		FlipFlop =!FlipFlop;

		LED_ON_Off = LED_ON_Off^1;

		//wait(1);
		POWER = 0;

		//LED1_ON_Off =!LED1_ON_Off;

		// Boucle attente
		//while(FLAG);

		// Pour attendre dans le while(FLAG);
		//FLAG = TRUE;

		// Traitement
		//if(LED1_ON_Off) FTDI.printf("LED1 allumée.\n\r");
		//else FTDI.printf("LED1 éteinte.\n\r");

	} // End While(true)

} // End main()

//---------------------------------------------------------------------------


/**
 * Codage des fonctions
 */

void IT_LED(void)
{
//LED_ON_Off = LED_ON_Off^1;
	FLAG_IT_MESURE = false;
}

/**
 * Fonction pour obtenir la configuration initiale des frequences de la Teensy 3.2
 * Suite au mode PowerDown, la Teensy ne fonctionne qu'à 72MHz
 */
void Teensy32_Back2Speed_After_PowerDown(void)
{
/* SIM->CLKDIV1: OUTDIV1 = 0, OUTDIV2 = 1, OUTDIV4 = 3 Set Prescalers 96MHz cpu, 48MHz bus, 24MHz flash*/
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) |  SIM_CLKDIV1_OUTDIV4(3);
  /* SIM->CLKDIV2: USBDIV = 2, Divide 96MHz system clock for USB 48MHz */
	SIM->CLKDIV2 = SIM_CLKDIV2_USBDIV(1);  
  /* OSC0->CR: ERCLKEN = 0, EREFSTEN = 0, SC2P = 1, SC4P = 0, SC8P = 1, SC16P = 0 10pF loading capacitors for 16MHz system oscillator*/
	OSC0->CR = OSC_CR_SC8P_MASK | OSC_CR_SC2P_MASK;
  /* Switch to FBE Mode */
  /* MCG->C7: OSCSEL = 0 */
	MCG->C7 = (uint8_t)0x00u;
  /* MCG->C2: LOCKRE0 = 0, RANGE0 = 2, HGO = 0, EREFS = 1, LP = 0, IRCS = 0 */
	MCG->C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS0_MASK;
  //MCG->C2 = (uint8_t)0x24u;
  /* MCG->C1: CLKS = 2, FRDIV = 3, IREFS = 0, IRCLKEN = 1, IREFSTEN = 0 */
	MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3) | MCG_C1_IRCLKEN_MASK;
  /* MCG->C4: DMX32 = 0, DRST_DRS = 0, FCTRIM = 0, SCFTRIM = 0 */  
	MCG->C4 &= (uint8_t)~(uint8_t)0xE0u; 
  /* MCG->C5: PLLCLKEN = 0, PLLSTEN = 0, PRDIV0 = 7 */
	MCG->C5 = MCG_C5_PRDIV0(7);
  /* MCG->C6: LOLIE = 0, PLLS = 0, CME = 0, VDIV0 = 0 */
	MCG->C6 = (uint8_t)0x00u;
  while((MCG->S & MCG_S_OSCINIT0_MASK) == 0u) { } /* Check that the oscillator is running */
  while((MCG->S & 0x0Cu) != 0x08u) { } /* Wait until external reference clock is selected as MCG output */
  /* Switch to PBE Mode */
  /* MCG_C5: PLLCLKEN = 0, PLLSTEN = 0, PRDIV0 = 5 */
  MCG->C5 = MCG_C5_PRDIV0(3); // config PLL input for 16 MHz Crystal / 4 = 4 MHz
  /* MCG->C6: LOLIE = 0, PLLS = 1, CME = 0, VDIV0 = 3 */
  MCG->C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0);// config PLL for 96 MHz output
  while((MCG->S & MCG_S_PLLST_MASK) == 0u) { } /* Wait until the source of the PLLS clock has switched to the PLL */
  while((MCG->S & MCG_S_LOCK0_MASK) == 0u) { } /* Wait until locked */
  /* Switch to PEE Mode */
  /* MCG->C1: CLKS = 0, FRDIV = 2, IREFS = 0, IRCLKEN = 1, IREFSTEN = 0 */
  MCG->C1 = MCG_C1_FRDIV(2) | MCG_C1_IRCLKEN_MASK;
  while((MCG->S & 0x0Cu) != 0x0Cu) { } /* Wait until output of the PLL is selected */
  while((MCG->S & MCG_S_LOCK0_MASK) == 0u) { } /* Wait until locked */
}


/**
 * Fonction pour OSV3
 */
unsigned char OSV3_CALC_CRC(bool *InitFait)
{
	unsigned char CRC = 0;

	// Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	// Inclure les octets de l'indice 4 à 10. Ne pas inclure le CHECKSUM à l'indice 11

	OSV3_INIT_CRC(InitFait);

	CRC = OSV3_TAB_CRC_INIT[OSV3_TAB_DATA_HR[3] & 0x0F];
	// ne sert à rien
	//CRC = CRC & 0xFF;

	for (int i = 4 ; i<11 ; i++)
	{ 
		CRC = OSV3_TAB_CRC_INIT[CRC ^ OSV3_TAB_DATA_HR[i]];
		// ne sert à rien
		//CRC = CRC & 0xFF;
	}
	// Permutation des NIBBLES
	return( ((CRC & 0x0F)<<4) | ((CRC & 0xF0)>>4) );

}


/**
 * Capteur T, HR et P
 */
unsigned char OSV3_CALC_CRC_UV(bool *InitFait)
{
	unsigned char CRC = 0;

	// Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA
	// Inclure les octets de l'indice 4 à 9. Ne pas inclure le CHECKSUM à l'indice 10

	OSV3_INIT_CRC(InitFait);

	CRC = OSV3_TAB_CRC_INIT[OSV3_TAB_DATA_UV[3] & 0x0F];
	// ne sert à rien
	//CRC = CRC & 0xFF;

	for (int i = 4 ; i<10 ; i++)
	{ 
		CRC = OSV3_TAB_CRC_INIT[CRC ^ OSV3_TAB_DATA_UV[i]];
	// ne sert à rien
	//CRC = CRC & 0xFF;
	}
	// Permutation des NIBBLES
	return( ((CRC & 0x0F)<<4) | ((CRC & 0xF0)>>4) );

}

void OSV3_INIT_CRC(bool *InitFait)
{
	// A faire avant de calculer le CRC -> Fait lors du 1er calcul
	unsigned char CRC;

	if(!*InitFait) {
		for(int i = 0 ; i<256 ; i++)
		{
			CRC = i;

			for(int j = 0 ; j<8 ; j++)
			{
				CRC =(CRC<<1)^((CRC & 0x80) ? OSV3_CRC8_M433_DI : 0);
			}

			// ne sert à rien ici le & bit à bit
			//OSV3_TAB_CRC_INIT[i] = CRC & 0xFF; 
			OSV3_TAB_CRC_INIT[i] = CRC;
		}

		*InitFait = true;
	}

}

/**
 * Capteur T et HR
 */
unsigned char OSV3_CALC_CHECKSUM_THR(void)
{
	unsigned char CheckSum;
	// Ne pinclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA

	CheckSum = ( OSV3_TAB_DATA_HR[3] & 0x0F );

	// Inure les octets de 4 à 10
	for(int i = 4 ; i <= OSV3_TAILLE_DATA_CAPTEUR_THR-3 ; i++)
	{
		CheckSum = CheckSum + (OSV3_TAB_DATA_HR[i] & 0x0F) + ((OSV3_TAB_DATA_HR[i]>>4) & 0x0F);
	}

	// Peration des NIBBLES
	return( ((CheckSum & 0x0F)<<4) | ((CheckSum & 0xF0)>>4) );

}

/**
 * Capteur UVN800
 */
unsigned char OSV3_CALC_CHECKSUM_UV(void)
{
	unsigned char CheckSum;
	// Ne pas inclure tout l'octet d'indice 3 : ne pas tenir compte des 4 bits de syncro -> 0xA

	CheckSum = ( OSV3_TAB_DATA_UV[3] & 0x0F );

	// Inclure les octets de 4 à 10
	for(int i = 4 ; i <= OSV3_TAILLE_DATA_CAPTEUR_UV-3 ; i++)
	{
		CheckSum = CheckSum + (OSV3_TAB_DATA_UV[i] & 0x0F) + ((OSV3_TAB_DATA_UV[i]>>4) & 0x0F);
	}

	// Permutation des NIBBLES
	return( ((CheckSum & 0x0F)<<4) | ((CheckSum & 0xF0)>>4) );

}



/**
 * Trame pour capteur T et HR
 */
void OSV3_CONSTRUIRE_TRAME_THR(float Temp_f, int HumiHR)
{
	// Les nibbles sont envoyés LSB first

	// Preambule du protocole OSV3
	// 24 bits à 1 -> 6 nibbles
	OSV3_TAB_DATA_HR[0] = 0xFF;
	OSV3_TAB_DATA_HR[1] = 0xFF;
	OSV3_TAB_DATA_HR[2] = 0xFF;
	// nibble de synchro -> 0101 -> LSB en 1er soit 0xA0
	OSV3_TAB_DATA_HR[3] = 0xA0;

	// Trame de données du capteur THGR810 -> payload
	// les nibbles 0..3 sont l'ID du capteur qui est unique pour chaque capteur ou commun pour
	// un groupe de capteur.
	// Ici ID du capteur est F824 dans l'ordre de reception
	OSV3_TAB_DATA_HR[3] = OSV3_TAB_DATA_HR[3] | 0x0F;
	OSV3_TAB_DATA_HR[4] = 0x82;
	OSV3_TAB_DATA_HR[5] = 0x40;

	// le nibble 4 pour le CANAL de 1 à 15  
	// Insertion du CANAL
	OSV3_TAB_DATA_HR[5] = OSV3_TAB_DATA_HR[5] | OSV3_CANAL_TEMP_HR;

	// Les nibbles 5..6 pour le code tournant dont la valeur est aleatoire
	// à chaque reset du capteur : exemple changement de piles.
	// OSV3_TAB_DATA[6] = OSV3_ROLLING_CODE();
	OSV3_TAB_DATA_HR[6] = VAL_ROLLING_CODE;
	// Capteur avec bit d'etat de la batterie -> toujours à 0 pour batterie chargée
	// valeur à 1 lorsque la batterie est à changer
	// A changer par une variable pour evolution.
	OSV3_TAB_DATA_HR[7] = 0x80;

	// Les nibbles 8..[N-5] sont les données du capteur
	// Les nibbles 10..8 sont la temperature avec 1 LSB qui represente 0.1 °C
	// exemple : un float de 23.5°C est à transformer en entier de 235
	int temp =(int)(Temp_f*10);
	// Extraction de 5 de 23.5°C
	OSV3_TAB_DATA_HR[7] = OSV3_TAB_DATA_HR[7] | ( (abs(temp) % 10) & 0x0F );
	// Extraction de 3 de 23.5°C
	OSV3_TAB_DATA_HR[8] =((abs(temp)/10) % 10) << 4;
	// Extraction de 2 de 23.5°C
	OSV3_TAB_DATA_HR[8] = OSV3_TAB_DATA_HR[8] | ((abs(temp)/100) & 0x0F);
	// Le nibble 11 represente le signe de la temperature -> une valeur differente de 0 est 
	// une temperature negative
	OSV3_TAB_DATA_HR[9] =(Temp_f <0) ? 0x80 : 0;
	// Extraction de HD en %
	OSV3_TAB_DATA_HR[9] = OSV3_TAB_DATA_HR[9] | ((HumiHR % 10) & 0x0F);
	OSV3_TAB_DATA_HR[10] =((HumiHR /10) % 10) <<4 ;
	// Placement du CHECKSUM
	// Le resultat de la somme sur 8 bits des nibbles 0..[N-5]
	// Le CHECKSUM est placé dans [N-3]..[N-4]
	OSV3_TAB_DATA_HR[11] = OSV3_CALC_CHECKSUM_THR();
	// Placement du CRC
	OSV3_TAB_DATA_HR[12] = OSV3_CALC_CRC(&Fait_Init_TAB_CRC);
}

unsigned char OSV3_ROLLING_CODE(void)
{

	// Lecture d'une entrée analogique sur 16 bits sur la teensy 3.2
	// Ici l'entrée analogique est PTC0 soit A1.
	unsigned short VCAN = AIN.read_u16();
	// Initialisation du generateur aléatoire
	srand(VCAN);

	// Nombre aleatoire entre 1 et 254.
	return( (rand() % 253) +1 );

}


void OSV3_MANCHESTER_ENCODE(unsigned char Octet_Encode, bool Fin)
{
	unsigned char MASK = 0x10;
	// Timer pour la gestion du temps.
	// Lecture de la durée
	static int TimeBase = TimeManchester.read_us();
	// Bouleen pour tester le dernier bit
	static bool LastBit = false;

	// OSV3 emet à la frequence de 1024Hz ou 1024bit/s
	// Prevoir un ajustement en fonction du temp de traitement par le uC
	// En mesure : 1020Hz et donc 490us
	const unsigned int DureeDesire = 490;
	// Mode PowerDown -> plus lent et on mesure 764, 5Hz au lieu de 1020Hz
	// Au reveil, la Teensy est à 72MHz au lieu de 96MHz -> 96/72 = 1.33
	// const unsigned int DureeDesire = 367;
	// Valeur ajustemet du au temps de traitement
	// Reduction de DureeDesiree
	const unsigned int ReduireDe = 32;
	// Mode PowerDown -> plus lent et on mesure 764, 5Hz au lieu de 1020Hz
	// const unsigned int ReduireDe = 24;

	// Les bits sont transmis de 4 à 7 puis de 0 à 3
	for(int i = 0 ; i<8 ; i++)
	{
		TimeBase = TimeBase+DureeDesire;

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
			Data_433 = 0;
			wait_us(DureeDesire-ReduireDe);
			Data_433 = 1;

			// Test si dernier bit
			// Dans ce cas pas de retard long apres la transition de 0 à 1 
			// pour indiquer que plus de donnée à suivre
			if(Fin) wait_us(DureeDesire);
			LastBit = false;

		}
		else {
			Data_433 = 1;
			wait_us(DureeDesire-ReduireDe);
			Data_433 = 0;
			LastBit = true;
		}

		if(MASK == 0x80) MASK = 0x01;
		else MASK = MASK<<1;

		TimeBase = TimeBase+DureeDesire;
	}
}

void OSV3_MANCHESTER_SEND_DATA_THR(void)
{
	// Est-ce necessaire de faire un reset du Timer ?
	TimeManchester.reset();
	TimeManchester.start();

	// Ajouter mise sous tension du module TX en 433MHz
	// Prevoir une tempo de 60ms

	// Mise à 0 de la broche
	//Data_433 = 0;

	for(int i = 0 ; i<OSV3_TAILLE_DATA_CAPTEUR_THR ; i++)
	{
		OSV3_MANCHESTER_ENCODE(OSV3_TAB_DATA_HR[i], i+1 == OSV3_TAILLE_DATA_CAPTEUR_THR);

	}

	// Ajouter mise hors tension du module TX en 433MHz
	// Mise à 0 de la broche
	//Data_433 = 0;

	// Arret du Timer
	// A tester si necessaire suite à PowerDown de la Teensy 3.2
	TimeManchester.stop();

}



void OSV3_MANCHESTER_SEND_DATA_UV(void)
{
	// Est-ce necessaire de faire un reset du Timer ?
	TimeManchester.reset();
	TimeManchester.start();

	// Ajouter mise sous tension du module TX en 433MHz
	// Prevoir une tempo de 60ms

	// Mise à 0 de la broche
	//Data_433 = 0;

	for(int i = 0 ; i<OSV3_TAILLE_DATA_CAPTEUR_UV ; i++)
	{
		OSV3_MANCHESTER_ENCODE(OSV3_TAB_DATA_UV[i], i+1 == OSV3_TAILLE_DATA_CAPTEUR_UV);
	}

	// Ajouter mise hors tension du module TX en 433MHz
	// Mise à 0 de la broche
	//Data_433 = 0;

	// Arret du Timer
	// A tester si necessaire suite à PowerDown de la Teensy 3.2
	TimeManchester.stop();

}

/**
 * Codage des fonctions pour OSV2
 */

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
	// THGR122NX - valide
	//unsigned char Test1[] ={0x1A, 0x2D, 0x40, 0x4D, 0x92, 0x22, 0x30, 0x64, 0x41, 0xFC};
	// BTHR968 
	//unsigned char Test2[] ={0x5A, 0x6D, 0x00, 0x7A, 0x10, 0x23, 0x30, 0x83, 0x86, 0x31};
	// BTHGN129 - T, Hum, P
	//unsigned char Test3[] ={0x5D, 0x53, 0x22, 0xF0, 0x08, 0x10, 0x86, 0x05, 0xD1, 0x29, 0x5};
	// BTHR918N
	//unsigned char Test4[] ={0x0D, 0x54, 0x02, 0x00, 0xCB, 0x02, 0x00, 0x6E, 0x34, 0x00, 0x03, 0xF5, 0x01, 0x79};
	//unsigned char Test5[] ={0x0D, 0x54, 0x02, 0x00, 0xCB, 0x02, 0x00, 0x6E, 0x34, 0x00, 0x03, 0xF5, 0x01};

	SendPreamble();
	SendData(data, size);
	//SendData(Test1, sizeof(Test1));
	//SendData(Test2, sizeof(Test2));
	//SendData(Test3, sizeof(Test3));
	//SendData(Test4, sizeof(Test4));
	//SendData(Test5, sizeof(Test5));
	SendPostamble();
}
void SendPreamble(void)
{
	unsigned char PREAMBLE[] ={0xFF, 0xFF};
	SendData(PREAMBLE, 2);
}

void SendPostamble(void)
{
	unsigned char POSTAMBLE[] ={0x00};
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
	if(Temp < 0)
	{
		data[6] = 0x08;
		Temp *= -1;  
	}
	else
	{
		data[6] = 0x00;
	}

	// Determine decimal and float part
	int tempInt = (int)Temp;
	int td = (int)(tempInt / 10);
	int tf = (int)((float)((float)tempInt/10 - (float)td) * 10);

	int tempFloat =  (int)((float)(Temp - (float)tempInt) * 10);

	// Set temperature decimal part
	data[5] = (td << 4);
	data[5] |= tf;

	// Set temperature float part
	data[4] |= (tempFloat << 4);
}

void SetHumidity(unsigned char *data, unsigned char Hum)
{
	data[7] = (Hum/10);
	data[6] |= (Hum - data[7]*10) << 4;
}

void SetPressure(unsigned char *data, float pres) 
{
	if ((pres > 850) && (pres < 1100)) {
		data[8] = (int)(pres) - 856;
		data[9] = 0xC0;  
	}
}

unsigned char Sum(unsigned char count, unsigned char *data)
{
	int s = 0;

	for(int i = 0; i<count ;i++)
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
	data[6] |=  (s&0x0F) << 4;     data[7] =  (s&0xF0) >> 4;
}
