/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * Test transfert en binaire par HC-12
 * Par un struct dans un union
 * 
 */


#include "mbed.h"
#include "HC12.h"


void XBee_IT_RX(void)
{
	do {
		data_RX_XBee[CPT_RX_XBee++] = XBee_SX.getc();
	} while(XBee_SX.readable() || CPT_RX_XBee != sizeof(TU_DB));

	IT_RX_XBee_OK=true;
}

void LCD_NEXTION_IT_RX(void)
{
	do {
		data_RX_LCD_Nextion[CPT_RX_LCD_Nextion++] = LCD_HMI.getc();
	} while(LCD_HMI.readable());
	
	data_RX_LCD_Nextion[CPT_RX_LCD_Nextion] = '\0';

	IT_RX_LCD_Nextion_OK = true;

}

void Acq_Car_IT(void)
{
	// Mise à 1 de P19
	Mes_Duree_P19 = 1;

	do {
		Car_Recu = Xbee_BT.getc();
		data_RX_BT[CPT_I++] = Car_Recu;
	} while(Xbee_BT.readable() || Car_Recu != '?');

	data_RX_BT[CPT_I] = '\0';

	CPT_I = 0;

	Acq_Terminee = TRUE;
	// Le '!' et le '?' sont inclus dans data_RX
}

void Copie_de_Tableau(volatile char *s, char *d)
{
	while((*d++=*s++)!='\0');
}
