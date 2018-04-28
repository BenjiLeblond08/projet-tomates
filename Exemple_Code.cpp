

//Test transfert en binaire par HC-12
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
								 unsigned char RollingCode; //1 octet
								 enum list_OSV3 type_capteur; //4 octets
								 
							 } T_DB; //sizeof donne 37 octets

typedef union {
								T_DB Data_Capteur;
								char Tab_TU[37]; //meme taille que T_DB et au même emplacement mémoire.
	
							} TU_DB;
							
							
//Codage fonction IT

//Codage des fonctions d'IT pour reception XBee par IT suite TX en binaire
//Aucun caractere de perdu
void XBee_IT_RX(void)
{

	do
	{
		data_RX_XBee[CPT_RX_XBee++]=XBee_SX.getc();
	}
	while(XBee_SX.readable() || CPT_RX_XBee!=sizeof(TU_DB));

	
	IT_RX_XBee_OK=true;
}

							
							
							
Dans le main() :					
							
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
			
			
			
			//Test transmission binaire par HC-12
			for(int i=0;i<37;i++)
			{
				//Attente Buffer libre
				while(!HC_12.writeable());
				//Emission d'un caratere
				HC_12.putc(UnionOSV3.Tab_TU[i]);
				//Affichage par le maitre de l'emission
				FTDI.printf("\r\nEmission de %c",UnionOSV3.Tab_TU[i]);
				
			}
			
			
			//Calcul d'un seul ROLLING CODE pour simuler changement de piles
			if(!Fait_ROOLING_CODE) 
			{
				//VAL_ROLLING_CODE=OSV3_ROLLING_CODE();
				VAL_ROLLING_CODE=65;
				Fait_ROOLING_CODE=true;
			}
			
			
			
			
			
			
			
//-------------------------------------------------------------------------------------

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



//Fonction d'IT associée au BT
 void Acq_Car_IT(void)
{
	//Mise à 1 de P19
	Mes_Duree_P19=1;
	
	do
	{
		Car_Recu=Xbee_BT.getc();
		data_RX_BT[CPT_I++]=Car_Recu;
	}
	while(Xbee_BT.readable() || Car_Recu!='?');
	
	data_RX_BT[CPT_I]='\0';

	CPT_I=0;
	
	Acq_Terminee=TRUE;
	//Le '!' et le '?' sont inclus dans data_RX
}


//Copie du tableau data_RX dans le tableau Copie_data_RX
//Le compilateur ne supporte pas de travailler directement avec un type volatile dans la fonction Extraction_Data
void Copie_de_Tableau(volatile char *s, char *d)
{
	while((*d++=*s++)!='\0');
}	