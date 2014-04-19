

/*																																*
 * 				PROGETTO MISURA SFORZO IN CAMMINATA											*
 * 				REALIZZATO PER FACOLTA' SC. MOTORIE											*
 * 				URBINO																									*
 * 
 * 				DATA: 12/04/2012																				*
 * 				REV: 00.  REL 00: acq da singolo sensore								*
 * 				DATA: 15/05/2012
 *  			REV: 01.  REL 00: 
 * 				mod. gestione tasto stop acquisizione
 * 				agg. serial scheda al comando POLL
 * 				set sampling time
 * 																																*
 * 				REV: 02. REL 00:																				*
 * 				modificato bottone stop: ora e' start - stop						*
 * 				abilitato autooffset																		*
 * 				main.c						
 * 				
 * 				REV: 03. REL 00:
 * 				gestione cella 100kg su bastone													*
 * 				portato clock a 4.1943MHz																*
 *        aggiunta comunicazione diretta verso pc									*
 * 
 * 				REV: 03. REL 01:																				*
 *      	trigger comunicazione pc tramite tasto a bordo scheda   *
 * 																																*
 * 				REV: 05. REL 01:																				*
 *				GPS FASTRAX management										              *
 *        comandi tramite software dedicato oppure tramite        *
 *        hyperterminal																						*
 * 																																*
 * 				REV: 06. REL 01:																				*
 *				GPS FASTRAX management										              *
 *        comandi tramite software dedicato 							        *
 *        																												**/
 
#include <string.h>
 
#include "msp430x54xA.h"
#include "init.h"
#include "gps.h"
 
 
 
/// variabile impostata in UsoFlash.c che tiene conto del blocco 
/// scritto (valori possibili 0 1)
//extern volatile unsigned char blocco;
volatile __eepromData eepromData;

 
/// buffer for cell read
volatile unsigned char cellBuffer[DIM_BUFF + DIM_SEC_BUFF], CELL_BUFF_PTR = 0;
/// buffer for received pc command
volatile unsigned char uartBuff[DIM_UART_BUFF], RX_PTR = 0, TX_PTR = 0;
/// sincronizza il reset della flash
volatile unsigned char resetBlock = 0, pausa, SEND_DATA_TO_PC = 0, FIRST_BLOCK_WRITTEN = 0;
/// serve solo in questo file
volatile unsigned char  pagineScritte = 0, WRITE_FLASH = 0;
/// time tic
volatile unsigned int TIC = 0, CONT = 0, BUTTON_PRESS = 0;
/// SOC and interprocess com: ADC interrupt and timer interrupt
volatile unsigned char SAMPLE = 0, memoryFull = 0, CONTEGGIO_SAMPLING = 0;
/// to pc or to built-in flash
//volatile unsigned char dirConn = 0;
volatile unsigned char dirConn = 0, tentativiGPSON = 0, stopDirConnAcq = 1;

volatile int letturaADC = 0;

extern volatile unsigned char FINE_GPS_MESS, T0;
extern volatile unsigned char uart1RXBUFF[], NUM_MESS;


 void main(){
  unsigned char stato, numCamp = 0, stTasto, STledEEProm;
  unsigned int offsetADC, campioniInviati;
  
  char cmd[DIM_GPS_CMD], st1[DIM_GPS_CMD];
  volatile unsigned int ritardo;
  volatile unsigned char conteggio = 0;
  unsigned char cksum;
  
  data GPSdata;
  GPSstatus gpsSt;
  GPSmessStatus GPSmST;
  
  __MCUstatus  MCUstatus;										//contains the status of MCU
  
 	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  setDCO(FDCO);																// set up dco clock
	 	
 	__bic_SR_register(GIE);										// stop interrupt
 	
 	//strcpy((char*) uart1RXBUFF, "$GPGGA,114353.000,6016.3245,N,02458.3270,E,1,10,0.81,35.2,M,19.5,M,,*50\r\n$PMTK001,000,3*30\r\n");
 	init_PORT();															// init port
 	//init_ADC(TEMP_SENS);										// init ADC with internal temp source
 	init_ADC_int(CELL_1);
 	//init_ADC(CELL_1);													// init ADC
 	//init_ADC_int(TEMP_SENS);
 	init_UART0(115200);												// init UART0 for PC comm
 	init_UART1(9600);													// init UART1 for GPS comm
 	init_TIMER();															// init timer
 	init_MCUstatus(&MCUstatus);
 	init_GPSstatus(&gpsSt, &GPSmST);
 	
	#ifndef _HAS_FLASH_
	#pragma  warning "flash not present"
	#endif 

 	#ifdef _HAS_FLASH_
  initEepromData();
  //buffer1[10] = 'a';
  //buffer[20] = buffer1[10];
  initI2C_B0(eepromData.blocco);
  #endif
 	//eepromData.blocco = 0;
 	__bis_SR_register(GIE);										// enable interrupt
 
 	P8OUT |= 32;			///LED secondo blocco on	
	P8OUT &= 0x7D;		///LED primo blocco off
	P8OUT |= 2;
	P8OUT &= 0x5F;
 
  /// il sistema parte in stato di pausa
  pausa = 1;
  stTasto = 0;
  /// deve stabilire il numero di pagine impegnate in flash
  /// questo serve perche' puo' saltare l'alimentazione oppure
  /// i circuito e' stato spento senza aver letto i dati dalla flash
  
  #ifdef _HAS_FLASH_
  pagineScritte = 1;
  eepromData.numPage = leggiPagineScritte();
  pagineScritte = 0;
  
  if (eepromData.numPage == FLASH_FULL){
  /*
   * non posso continuare perche' la flash e' piena*/
		memoryFull = 1;
		eepromData.blocco 		= 0;
		eepromData.address 		= 0;
		eepromData.numPage 		= 0;
		eepromData.readAddr 	= 0;
		eepromData.readBlocco = 0;
		initI2C_B0(eepromData.blocco);	
  }
  else{
  	/// inutile continuare se la flash e' piena
	  if( eepromData.numPage > 511){
	  	//we change the block
	  	eepromData.blocco = 1;
	  	/// non serve perché già impostato nella funzione leggiPagineScritte()
	  	//initI2C_B0(eepromData.blocco);	
	  	P8OUT |= 32;			///LED secondo blocco on	
			P8OUT &= 0x7D;		///LED primo blocco off
	  }
	  else{
	  	P8OUT |= 2;
			P8OUT &= 0x5F;
	  }
	  /// numPage contains thee first available free page
	  eepromData.address = (eepromData.numPage << 7);
  }
  #endif
  
  /// setup adc
 	offsetADC = setupADC();
 	/// ciclo principale;
  setPort(1);
  
 	offsetADC = 0;
 	testGPS(cmd, &GPSmST);
  PRINTF(cmd);
 	/// qui attendo la risposta alla presenza del gps e dopo un po' di tempo vado avanti
 	/// dicendo che non c'è
 	while(offsetADC++ < 20 && gpsSt.status == GPS_OFF){
 		unsigned long int i,j;
 		//PRINTF(cmd);
 		j = 1;
 		while (j){
 			for (i = 0; i < 80000;)
  			i++;
 			j = parseGPSMess( &GPSdata, &gpsSt, &GPSmST);
 		}
 	}
 	tentativiGPSON++;
// 	if (gpsSt.status == GPS_OFF){
// 		PRINTF0("GPS non presente \r\n\0");
// 	}
// 	else
// 	 	PRINTF0("GPS presente\r\n\0");

  campioniInviati = 0;
 	for(;;){
 		/// check if some messagge arrived
  	if (RX_PTR != TX_PTR)
  		replyToMessage(&MCUstatus);
  	/// controllo i tasti in ogni caso, cioe' con dirConn == 1 e dirConn == 0
 			
		P8OUT &= 0x76;
	/// an automa to understand the switch
 		switch(stTasto){
 			case 0:
 			/// test on switch
	 			if (!(P2IN & 1)){
	 			/// switch pressed
	 				ritardo = 5000;
	 				do{ ritardo--;
	 				}while (ritardo > 1);
	 				if (!(P2IN & 1)){
	 					stTasto = 1;
	 					BUTTON_PRESS = 0;
	 				} 
	 				else
	 					stTasto = 0;
	 			}
 			else
 				stTasto = 0;
			break;
			
			case 1:
				if (!(P2IN & 1)){
	 			/// switch pressed
	 				stTasto = 1;
	 				if (BUTTON_PRESS > 20)
	 					P8OUT ^= 64;
	 				} 
	 			else
	 			/// switch released
	 				stTasto = 2;
	 			
			break;
			
			case 2:
				
//				if (BUTTON_PRESS > 20){
//				/// we reset the flash
//					//clearFlash(255);
//					;
//				}
				stTasto = 0;
				BUTTON_PRESS = 0;
			break;
			 			
 		}
 		/// test se il tasto di stop e' premuto
		if (stTasto == 2) {
			/// tasto premuto provenendo dallo stato di stop: avvio l'acquisizione
			if (pausa == 1){
				pausa = 0;
				/// faccio partire l'acquisizione in modalita' connessione diretta
				if (dirConn)
					stopDirConnAcq = 0;
			}
			else {
				/// if we came from a start condition, we stop
				pausa = 1;
				P8OUT &= 0x6E;  ///spegne bit0 e bit 4
			}
			/// se provengo da DIRECT_CONNECTION fermo l'acquisizione
			if (pausa == 1 && dirConn == 1){
				stopDirConnAcq = 1;
				campioniInviati = 0;
			}
			/// occorre fermare il micro e caricare almeno 8 byte di separazione
			if (pausa == 1 && !dirConn){
				if (conteggio % 2)
					conteggio++;
				if (conteggio >= 120)
					conteggio = 120; 		 	
				for (; conteggio < 128; conteggio += 2){
				/// mark di fine acquisizione
					cellBuffer[conteggio] = 1;						/// simbolo #
					cellBuffer[conteggio + 1] = 255;
				}
				/// segnala che deve scrivere in flash
				WRITE_FLASH = 1;
			}
 		}

		if (pausa == 0 &&  memoryFull == 0 && !dirConn){
			if (SAMPLE){
		/// e' ora di raccogliere un campione
			  letturaADC = 0;
				ADC12CTL0 |= ADC12SC;					
  			SAMPLE = 0;
			}
  		if (resetBlock == 1){
  			/// ho resettato la flash, quindi resetto anche il blocco
  			resetBlock = 0;
  			conteggio = 0;
  			CELL_BUFF_PTR = 0;
  		}
			conteggio = CELL_BUFF_PTR;
		}
		/// scrittura in flash, che avviene ad interruzioni abilitate.
		/// non dovrebbe costituire problema per la ricezione di messaggi da GPS
		/// se cosi' non fosse occorrerebbe scrivere nel tempo tra 2 mess gps (da fare solo SE NECESSARIO!)
		/// tempi: 128B * 3us/b = 128 * 24us 	 = 3,1ms
#ifdef _HAS_FLASH_
			//ccc;
			if((WRITE_FLASH == 1) && (memoryFull == 0 )){
				WRITE_FLASH = 0;
				/// bisogna tenere traccia di quante pagine abbiamo gia' scritto in flash
				/// per questo usa la variabile num. pag. che viene ricaricata all'inizio di ogni 
				/// sessione e scorre la flash finche' non trova la prima cella libera.
				do{
					stato = writeI2C_N_Byte(eepromData.address, eepromData.blocco, DIM_BUFF, cellBuffer );
					} while (stato != OK);
				eepromData.address += 128;
				if (eepromData.address == 0){
				/// ho terminato gli indirizzi disponibili in un blocco e quindi
				/// occorre passare alla pagina successiva se e' possibile
					if (eepromData.blocco == 1){
						memoryFull = 1;
					}
					else{
						eepromData.blocco = 1;
						initI2C_B0(eepromData.blocco);
						P8OUT |= 32;			///LED secondo blocco on	
						P8OUT &= 0x7D;		///LED primo blocco off
					}
				}
				conteggio = 0;
				/// flag che indica la fine dell'operazione di scrittura e serve alla routine di interruzione
				/// che gestisce il convertitore AD.
				FIRST_BLOCK_WRITTEN = 1;
			}
#endif
 		/// END of if(!dirConn)
 		if (dirConn && !stopDirConnAcq){
 			/// spengo la segnalazione dei banchi di EEPROM
 			P8OUT &= 0x5D;
 			/// nel collegamento diretto si caricano i campioni senza avvisare 
			if (SAMPLE){
			/// e' ora di raccogliere un campione
				  letturaADC = 0;
					ADC12CTL0 |= ADC12SC;					
	  			SAMPLE = 0;
				}
			/* ************************************ */
			/* QUI DECIDO SE DEVO INVIARE I DATI    *
			 *  AL PC            										*/
			/* ************************************ */
			if (SEND_DATA_TO_PC){ 
				P4OUT |= 0x20;
//			 	while (!(UCA0IFG&UCTXIFG));
//			 	UCA0TXBUF = cellBuffer[CELL_BUFF_PTR - 1];
			/// provo ad inviare il numero di campione al programma che gira sotto windows
				//while (!(UCA0IFG & UCTXIFG));
				//UCA0TXBUF = (campioniInviati >> 8);
				//while (!(UCA0IFG & UCTXIFG));
				//UCA0TXBUF = (campioniInviati & 0xFF);
				while (!(UCA0IFG & UCTXIFG));
				UCA0TXBUF =  cellBuffer[CELL_BUFF_PTR - 1];
				CELL_BUFF_PTR &= 0x80;
				//campioniInviati++;
				//PRINTFU(cellBuffer[CELL_BUFF_PTR - 1]);
			 	/// finche' non c'e' il nuovo dato non passa di qui
			 	SEND_DATA_TO_PC = 0;
			  P4OUT &= 0xDF;

					 /*						ANALISI MESSAGGIO DAL GPS 						*/
				if (gpsSt.status == GPS_ON){
					P8OUT |= 1;								/// segnala presenza di GPS
					while(NUM_MESS)
					/// gira finche' non sono esauriti tutti i messaggi delimitati da $ *
						parseGPSMess(&GPSdata, &gpsSt, &GPSmST);
					/// all'uscita ho le stringhe oppure il comando di risposta in GPSdata
	//				if (gpsSt.GLL == VALID){
	//					/// inviare i dati significativi di GGL;
	//					gpsSt.GLL = NOTVALID;
	//				}
					if (gpsSt.GGA == VALID){
						P8OUT ^= 0x8;				/// segnala lettura dati GPS: blink di un led con cadenza 1s (t lettura gps)
						/*****************************************************************/
						/// bisogna bloccare la lettura ad interrupt del convertitore ADC
						/// inviare i dati significativi di MCE
						gpsSt.GGA = NOTVALID;
						PRINTF0("time\tlatitude\tNS\tlongitude\tEW\r\n\0");
						PRINTF0(GPSdata.UTCtime);
						PRINTF0("\t");
						PRINTF0(GPSdata.latitude);
						PRINTF0("\t");
						PRINTF0(GPSdata.NS);
						PRINTF0("\t");
						PRINTF0(GPSdata.longitude);
						PRINTF0("\t");
						PRINTF0(GPSdata.EW);
						PRINTF0("\r\n\0");
						PRINTF0("altitude\tHDOP\tspeed knots\r\n\0");
						PRINTF0(GPSdata.altitude);
						PRINTF0(" m\t");
						PRINTF0(GPSdata.HDOP);
						PRINTF0("\t");
						
						if ((gpsSt.RMC & 1) == VALID){
							/// inviare i dati significativi di MCE
							gpsSt.RMC &= NOTVALID;
							PRINTF0(GPSdata.speedKn);
		//					PRINTF0(GPSdata.longitude);
						}
						PRINTF0("\n\n\r\n\0");
					}
						
				} 	
				else{
					P8OUT &= 0xFE;					/// segnala assenza GPS;
					if (tentativiGPSON < 5){
						/// non c'e' il gps
						offsetADC = 0;
						testGPS(cmd, &GPSmST);
						PRINTF(cmd);
						/// qui attendo la risposta alla presenza del gps e dopo un po' di tempo vado avanti
						/// dicendo che non c'è
						while(offsetADC++ < 40 && gpsSt.status == GPS_OFF){
							unsigned long int i,j;
							//PRINTF(cmd);
							j = 1;
							while (j){
								for (i = 0; i < 80000;)
									i++;
								j = parseGPSMess( &GPSdata, &gpsSt, &GPSmST);
							}
						}
						tentativiGPSON++;
//						if (gpsSt.status == GPS_OFF){
//							PRINTF0("GPS non presente \r\n\0");
//						}
//						else
//						 	PRINTF0("GPS presente\r\n\0");
					}
				}
 			}
 		} /// END of  (if dirConn && !stopDirConnAcq)
// 		else{
// 			campioniInviati = 0;
// 		
// 		}

	} /// end of infinyte cycle
}

 
 
 /// 
 /// interrupt routine for timer 0
 ///
 // Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
	/// this call doesn't exit LPM mode because change in CPUOFF bit
	/// is not done in restored SR reg when exit from interrupt.
	/// we need to change CPUOFF bit in SR reg stored onto the stack.
	//__bic_SR_register(LPM0_bits);       		// do we need wake up???
	/// this call acts onto stack and release LPM mode when exit from 
	/// interrupt.
	//LPM0_EXIT; /// =  _bic_SR_register_on_exit(LPM0_bits), intrinsic into compiler!
	/// see also main.asm file to understand how compiler translate
	
	/// solo per debug, poi togliere
	P4OUT ^= 0x10;
	//memcpy((void *) cellBuffer, (void *) &cellBuffer[128], 5);
	/// it is possible to start data acquisition
		if (pausa == 0 && memoryFull == 0)
			/// acquisizione dei 4 campioni per formare la lettura (SOC)
			  		ADC12CTL0 |= ADC12SC;				
		  		
	BUTTON_PRESS++;
	CONT++;
	TIC++;
	SAMPLE = 1;			/// feed back to main infinite cycle
	/// pin P4.6 (pin 52) blink to debug timer A
	P4OUT ^= 0x40;
	if (dirConn == 1 && pausa == 1){
	/// lampeggia lentamente e controlla 
		if (CONT >= T0){
			P8OUT ^= 16;
			CONT = 0;
		}
	}
	else if ((pagineScritte == 0) && (pausa == 0) && (memoryFull == 0)){

		P8OUT ^= 16;
//		if (CONT >= 16){
//			P8OUT |= 1;
//			CONT = 0;
//		}
//		else
//			P8OUT &= 0xFE;
	}
	else 
	if (memoryFull == 1)
	  P8OUT = 32 + 2;
	//else
	//  P8OUT &= 0x5D;   ///  101 | 1101
	/// solo per debug, poi togliere
	P4OUT ^= 0x10;
}



//// Echo back RXed character, confirm TX buffer is ready first
//#pragma vector=USCI_A0_VECTOR
//__interrupt void USCI_A0_ISR(void)
//{
//  switch(__even_in_range(UCA0IV,4))
//  {
//  case 0:break;                             // Vector 0 - no interrupt
//  case 2:  
//    //P8OUT ^= 1;                             // Vector 2 - RXIFG
//    /// read byte
//    uartBuff[TX_PTR++] = UCA0RXBUF;
//    /// mod 64
//    TX_PTR &= DIM_UART_BUFF - 1;	
//    //while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
//    //UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
//    //P8OUT ^= 1;
//    break;
//  case 4:break;                             // Vector 4 - TXIFG
//  default: break;
//  }
//}

//// Echo back RXed character, confirm TX buffer is ready first
//#pragma vector=USCI_A1_VECTOR
//__interrupt void USCI_A1_ISR(void)
//{
//	unsigned char c;
//	switch(__even_in_range(UCA1IV,4))
//  {
//  case 0:break;                             // Vector 0 - no interrupt
//  case 2:  
//    //P8OUT ^= 1;                             // Vector 2 - RXIFG
//    /// read byte
//    c = UCA1RXBUF;
//    uart1RXBUFF[TX1_PTR++] = c;
//    if (c == '*'){
//    	FINE_GPS_MESS = 1;
//    	INIZIO_GPS_MESS == 0;
//    }
//    else if (c == '$'){
//    	INIZIO_GPS_MESS == 1;
//    	FINE_GPS_MESS = 0;
//    }
//    /// mod 64
//    TX1_PTR &= DIM_UART1_RX_BUFF - 1;	
//    //while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
//    //UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
//    //P8OUT ^= 1;
//    break;
//  case 4:
//  	// transmit a byte through serial1
//  	UCA1TXBUF = uart1TXBUFF[UTX1_PTR++];
//  	/// mod 32
//  	UTX1_PTR &= DIM_UART1_TX_BUFF - 1;
//  	/// when \0 is found it stop to tansmit
//  	if ( uart1TXBUFF[UTX1_PTR]  == 0){
//  	/// stop transmission
//  		UCA1IE = UCRXIE;                         // Enable USCI_A1 RX interrupt
//  		UTX1_PTR = 0;
//  	}
//  break;                             // Vector 4 - TXIFG
//  default: break;
//  }
//}

