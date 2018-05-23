
#include "mbed.h"

//Definition pour I2C
#define I2C_ACK     1
#define I2C_NACK    0
#define I2C_TIMEOUT	2

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
#define VEML6075_CONF_SD_ON      0x01 // Power down
 
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


bool I2C_Duree_Time_Out_Ecoulee=false;

Serial FTDI(PTD3,PTD2);
I2C Capt_I2C0(PTB3,PTB2);

/////////////////////////////////////////////////////////////////////////////
// Gestion du VEML6075 pour UVA - UVB et UVC en I2C0
/////////////////////////////////////////////////////////////////////////////
int VEML6075_GetID(int ADDR_VEML6075, bool *TimeOut_Ended);
int VEML6075_GetRauUVABD(int ADDR_VEML6075, char SelectABD, bool *TimeOut_Ended);
void VEML6075_WriteConfig(int ADDR_VEML6075, char Config, bool *TimeOut_Ended);
float VEML6075_GetUVA(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uva, unsigned int raw_dark);
float VEML6075_GetUVB(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uvb, unsigned int raw_dark);
float VEML6075_GetUVIndex(unsigned int raw_vis, unsigned int raw_ir, unsigned int raw_uva,unsigned int raw_uvb, unsigned int raw_dark);



int main(int argc, char const *argv[])
{
	FTDI.printf("\r\n");
	FTDI.printf("\r\n");
	FTDI.printf("\r\n");
	wait(1);
	while(true)
	{
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
		wait(1);
	}
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
			if((t.read()-debut)>I2C_TIMEOUT)
			{
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
