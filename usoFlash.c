
/*																																*
 * 				PROGETTO MISURA SFORZO IN CAMMINATA											*
 * 				REALIZZATO PER FACOLTA' SC. MOTORIE											*
 * 				URBINO																									*
 * 
 * 				DATA: 12/04/2012																				*
 * 				REV: 00.  REL 00: acq da singolo sensore								*
 * 																																*
 * 				usoflash																								*
 * 																																*/
 
 #include "msp430x54xA.h"
 #include "init.h"
 
 extern unsigned char cellBuffer[DIM_BUFF + DIM_SEC_BUFF];
 //volatile unsigned char	blocco = 0;
 extern volatile __eepromData eepromData;
 
 /// metto alcune funzioni di gestione della flash
 
 ///
 /// estrae la prima pagina libera e restituisce il numero del blocco.
 /// una pagina e' considerata libera quando contiene tutti i byte a 0xFF 
 /// Per la memoria da 128kB, le pagine sono 128*1024/128 = 1024 pagine
 ///
 unsigned int leggiPagineScritte(void){
 
 	unsigned int numPag = 0, address = 0, valore = 0;
 	char stato, chBlock = 0;
 	int i;
 	P8OUT = 1;
 	//char stato;
 	__bic_SR_register(GIE);		
 	do{
 		/// reset the value
		valore = 0;
	 	/// read 128 byte from one page of memory
	 	//stato = readI2C_N_Byte(address, 128, buffer);
	 									// stop interrupt
	 	do{
	 		stato = readI2C_N_Byte(address, 128, cellBuffer);
	 	} while (stato != OK);
	 	address += 128;
	 	for (i = 0; i < 128; i++)
			valore += cellBuffer[i];
		/// divisione per 128
		valore >>= 7;
		if (valore != 255)
		/// All cells don't contain 255, we look at new page 
			numPag++;
		if (numPag >= 512){
			  /// accendo il led 6
			  P8OUT |= 32;
			  /// e spengo il led 2
			  P8OUT &= 0x7D;		/// 111 | 1101
				if(chBlock == 0){
				/// sono arrivato al limite del blocco
				/// ed occorre reimpostare l'indirizzo nella flash
					chBlock = 1; /// segnalo che ho modificato il blocco e quindi non devo più entrarvi
											 /// questo flag viene resettato o al rientro nella funzione o dopo 
											 /// il raggiungimento di 1024
					if (eepromData.blocco == 0){
						eepromData.blocco = 1;
						eepromData.address = 0;
						initI2C_B0(eepromData.blocco);
					}	
			}
		}
		else
			eepromData.blocco = 0;
 	} while(valore != 255 && numPag < 1024);
 	/// occorre controllare con che codice esco, ovvero con quale numero di pagina
 	if (numPag >= 1024){
 		/// si ricomincia dal blocco 0
		eepromData.blocco = 0;	
		numPag = 0;
		chBlock = 0;
		eepromData.address = 0;
		initI2C_B0(eepromData.blocco);
		P8OUT = 8;
		__bis_SR_register(GIE);										// enable interrupt
		return FLASH_FULL;
 	}
 	P8OUT = 0;
 	__bis_SR_register(GIE);										// enable interrupt
 	return numPag;
 }
