
#include "mbed.h"
#include "WakeUp.h"
// include to check/display clock rates
#include "clk_freqs.h"



//Definition pour I2C
#define I2C_ACK     1
#define I2C_NACK    0
#define I2C_TIMEOUT	2

//Adresse du capteur I2C
//Ici 8bits car attente de 8bits pour l'adresse en I2C
#define ADDR_LUX 0b01000110
//Resolution de 0.5lx et attente de 120ms au minimum
#define Resolution_05LUX 0b00100001


/////////////////////////////////////////////////////////////////////////////
// Gestion du VEML6075 pour UVA - UVB et UVC en I2C0
/////////////////////////////////////////////////////////////////////////////
#define ADDRESSE_VEML6075 0x10<<1
#define VEML6075_DEVID 0x26
/* VEML6075 SLAVE ADDRESS AND FUNCTION DESCRIPTION */
#define VEML6075_REG_UV_CONF       0x00 // Configuration register (options below)
#define VEML6075_REG_Reserved01    0x01
#define VEML6075_REG_Reserved02    0x02
#define VEML6075_REG_Reserved03    0x03
#define VEML6075_REG_Reserved04    0x04
#define VEML6075_REG_Reserved05    0x05
#define VEML6075_REG_Reserved06    0x06
#define VEML6075_REG_UVA_Data      0x07 // UVA register
#define VEML6075_REG_UVD_Data      0x08 // Dark current register
#define VEML6075_REG_UVB_Data      0x09 // UVB register
#define VEML6075_REG_UVCOMP1_Data  0x0A // Visible compensation register
#define VEML6075_REG_UVCOMP2_Data  0x0B // IR compensation register
#define VEML6075_REG_DEVID         0x0C // Device ID register

#define VEML6075_CONF_IT_50MS      0x00 // Integration time = 50ms (default)
#define VEML6075_CONF_IT_100MS     0x10 // Integration time = 100ms
#define VEML6075_CONF_IT_200MS     0x20 // Integration time = 200ms
#define VEML6075_CONF_IT_400MS     0x30 // Integration time = 400ms
#define VEML6075_CONF_IT_800MS     0x40 // Integration time = 800ms
#define VEML6075_CONF_IT_MASK      0x8F // Mask off other config bits

#define VEML6075_CONF_HD_NORM      0x00 // Normal dynamic seetting (default)
#define VEML6075_CONF_HD_HIGH      0x08 // High dynamic seetting

#define VEML6075_CONF_TRIG         0x04 // Trigger measurement, clears by itself

#define VEML6075_CONF_AF_OFF       0x00 // Active force mode disabled (default)
#define VEML6075_CONF_AF_ON        0x02 // Active force mode enabled (?)

#define VEML6075_CONF_SD_OFF       0x00 // Power up
#define VEML6075_CONF_SD_ON        0x01 // Power down
 
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

//Configuration du bus I2C sur les broches : SCL0 et SDA0
//Configuration I2C
//					 SDA0  SCL0
I2C Capt_I2C0(PTB3,PTB2);

//Configuration du bus I2C sur les broches : SCL1 et SDA1
//Configuration I2C
//					 SDA1  SCL1
I2C Capt_I2C1(PTC11,PTC10);

/////////////////////////////////////////////////////////////////////////////
// Declaration objet pour SPI Optima
/////////////////////////////////////////////////////////////////////////////

//Mesure du temps de conversion du capteur DS18B20
Timer t;

////////////////////////////////////////////////////////////////////
// Variables globales
/////////////////////////////////////////////////////////////////////////////

volatile bool FLAG=true;

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
	unsigned short dig_T1;
    short          dig_T2, dig_T3;
    unsigned short dig_P1;
    short          dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
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
#define ADRESSE_SELECTION_BMP280 0x77<<1 
	
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
#define BMP280_FILTER_OFF   0b000    	// filter off
#define BMP280_FILTER_2     0b001    	// filter 2
#define BMP280_FILTER_5     0b010   	// filter 5
#define BMP280_FILTER_11    0b011   	// filter 11
#define BMP280_FILTER_22    0b100    	// filter 22

#define BMP280_OSSR_SKIP    0x00    // Skipped output set to 0x80000
#define BMP280_OSSR_OV1     0x01    // oversampling x1
#define BMP280_OSSR_OV2     0x02    // oversampling x2
#define BMP280_OSSR_OV4     0x03    // oversampling x4
#define BMP280_OSSR_OV8     0x04    // oversampling x8
#define BMP280_OSSR_OV16    0x05    // oversampling x16

// wait to read out time
// T_SB
#define BMP280_T_SB0        0x00    // 0.5ms
#define BMP280_T_SB62       0x01    // 62.5ms
#define BMP280_T_SB125      0x02    // 125ms
#define BMP280_T_SB250      0x03    // 250ms
#define BMP280_T_SB500      0x04    // 500ms
#define BMP280_T_SB1000     0x05    // 1000ms
#define BMP280_T_SB2000     0x06    // 2000ms
#define BMP280_T_SB4000     0x07    // 4000ms

// Power Mode
#define BMP280_POWER_SLEEP   0b00
#define BMP280_POWER_FORCE   0b01
#define BMP280_POWER_NORMAL  0b11

// OSSR Data soit pour osrs_p (3 bits) et osrs_t (3 bits)
//osrs_t :
//000 -> pas de mesure de T
//001 -> 16bits / 0.0050°C
//010 -> 17bits / 0.0025°C
//011 -> 18bits / 0.0012°C
//100 -> 19bits / 0.0005°C
//101 -> 20bits / 0.0003°C
#define OSRS_Pas_Mesure_T   0b000
#define OSRS_T_16bits       0b001
#define OSRS_T_17bits       0b010
#define OSRS_T_18bits       0b011
#define OSRS_T_19bits       0b100
#define OSRS_T_20bits       0b101

//osrs_p :
//000 -> pas de mesure de P
//001 -> 16bits / 2.62 Pa
//010 -> 17bits / 1.31 Pa
//011 -> 18bits / 0.66 Pa
//100 -> 19bits / 0.33 Pa
//101 -> 20bits / 0.16 Pa
#define OSRS_Pas_Mesure_P   0b000
#define OSRS_P_16bits       0b001
#define OSRS_P_17bits       0b010
#define OSRS_P_18bits       0b011
#define OSRS_P_19bits       0b100
#define OSRS_P_20bits       0b101

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

#define CTRL_MEAS   (OSRS_T_20bits <<5) | (OSRS_P_20bits << 2) | (BMP280_POWER_FORCE)
#define CONFIG      (BMP280_OSSR_OV16 <<5) | (BMP280_FILTER_22 <<2) | (0b00)

#define SEALEVELHPA 1013.25

//Ce qui donne :
//#define CTRL_MEAS		0b101 101 01 soit 0xb5
//#define CONFIG			0b101 100 00 soit 0xb0

//---------------------------------------------------------------------------

int main(void) 
{
	
	FTDI.baud(9600);
	FTDI.format(8, SerialBase::None, 1);
	
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
			
			FTDI.printf("\r\ndig_T = 0x%x, 0x%x, 0x%x\r\n", BMP280_Data.dig_T1, BMP280_Data.dig_T2, BMP280_Data.dig_T3);
			
			//Attente fin mesure
			while(!BMP280_Fin_Mesure(ADRESSE_SELECTION_BMP280, &TimeOut_Ended));
			
			//init(ADRESSE_SELECTION_BMP280, &BMP280_Data, 0x6F, 0x70);
			BMP280_readData(ADRESSE_SELECTION_BMP280, BMP280_Data, &BMP280_Temp, &BMP280_Pressure, &TimeOut_Ended);
			FTDI.printf("\r\nTemp: %4.4f °C, Pression: %4.2f hPa",BMP280_Temp, BMP280_Pressure/100.0);
			
			FTDI.printf("\r\nAltitude=%8.4fm",BMP280_readAltitude(SEALEVELHPA, BMP280_Pressure));

			wait(1);
		}
	}
}





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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
				case 14 : 
					wait_us((int)(RH_14bits*1000*3));
				break;
				case 11 : 
					wait_us((int)(RH_11bits*1000*3));
				break;
				case 8 	: 
					wait_us((int)(RH_8bits*1000*3));
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
	FTDI.printf("\r\nadc_T %d, adc_p %d\r\n", adc_T, adc_P);
	FTDI.printf("\r\ndig_T = 0x%x, 0x%x, 0x%x\r\n", BMP280_Data.dig_T1, BMP280_Data.dig_T2, BMP280_Data.dig_T3);
    
    var1=((((adc_T>>3)-((int)(BMP280_Data).dig_T1<<1)))*((int)(BMP280_Data).dig_T2))>>11;
    var2=(((((adc_T>>4)-((int)(BMP280_Data).dig_T1))*((adc_T>>4)-((int)(BMP280_Data).dig_T1)))>>12)*((int)(BMP280_Data).dig_T3))>>14;
    t_fine=var1+var2;
	FTDI.printf("\r\nt_fine %d", t_fine);
    T=(t_fine*5+128)>>8;
    *tempC = (float)(T/100.0);
	FTDI.printf("\r\nBMP280 T %d", T);
 
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
