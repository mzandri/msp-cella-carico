
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
#include "gps.h"
 
extern volatile __eepromData eepromData;
extern volatile unsigned char uartBuff[DIM_UART_BUFF], RX_PTR, TX_PTR;
extern volatile unsigned char memoryFull;
extern volatile unsigned char cellBuffer[DIM_BUFF];
static unsigned char state = 0, comando = 0;
extern unsigned char releaseSW[], releaseHW[];
extern volatile unsigned char resetBlock;
extern volatile unsigned char dirConn;

extern volatile unsigned int U1_Rec_PTR, U1_Read_PTR;
extern volatile unsigned char U1_TX1_PTR;

void transmitData(void);
void setHWkey(unsigned char []);
unsigned char setSampTime(void);

 
 ///
 /// this is a non blocking function that parse the message
 /// came from uart. If transfer is needed, it call a blocking
 /// function
 ///
void replyToMessage(__MCUstatus *st){
	
	unsigned char CRC, chiaviHw[10], ris = 0;
	if (st->stato == EMB_SW){
		switch(state){
		
			case 0:
				if (uartBuff[RX_PTR++] == SYNC)
					state = 1;
				RX_PTR &= DIM_UART_BUFF - 1;
			break;
			
			case 1:
				/// here the sync is ok and we read the message
				comando = uartBuff[RX_PTR++];
				RX_PTR &= DIM_UART_BUFF - 1;
				state = 2;
			break;
		
			case 2:
				CRC = uartBuff[RX_PTR++];
				RX_PTR &= DIM_UART_BUFF - 1;
				if (CRC == ((SYNC + comando) ^ CHECKSUM)){
					/// received a valid command; we parse
					switch(comando){
						case POLL:
							pollReply();
						break;
						case DOWNLOAD:
							transmitData();
						break;
						case DIRECT_CONNECT:
							directConnect();
						break;
						case GET_RELEASE:
						break;
						case CLEAR_FLASH:
						break;
						case SET_FLASH:
						break;
						case SET_HW_REL:
						break;
						case TEST_GPS:
							send_test_cmd();
						break;
						case GPS_ON:
						/// enable GPS communications
							gpsOnOFF(GPS_ON);
						break;
						case GPS_OFF:
						/// disable GPS communications
							gpsOnOFF(GPS_OFF);
						break;
						
						case SET_SAMPLING:
							do{
								ris = setSampTime();
							}while (ris != 0);
							
						 	WDTCTL = WDTPW+WDTCNTCL+WDTIS2+WDTIS0;
						 	while(1);
						break;
						
						default:
						break;
							
					}
					//transmitAllFlash();
					if (comando == GET_RELEASE)
						getRelease();
					if (comando == CLEAR_FLASH)
						clearFlash(0);
					if (comando == SET_FLASH){
						clearFlash(255);
						WDTCTL = WDTPW+WDTCNTCL+WDTIS2+WDTIS0;
					 	while(1);
					}
					if (comando == SET_HW_REL){
						///devono arrivare 7 byte della relase hw
						setHWkey(chiaviHw);
					}
					state = 0;
				}
			break;	
		}
	} //if (st->stato == EMB_SW)
	else{
		// it receive hyperterminal command
		
		// TO DO: add echo and other code!!!!
	}
}


/// reply to poll request
///
void pollReply(void){
	unsigned char check, i;
	/// transmit POLL command and checksum
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = POLL;
	check = POLL ^ CHECKSUM;
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = check;
	/// transmit a hw key
	check = 0;
	for ( i = 0; i < 7; i++){
		while (!(UCA0IFG&UCTXIFG));
		UCA0TXBUF = releaseSW[i];
		check += releaseSW[i];
	}
	check ^= CHECKSUM;
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = check;
	while (!(UCA0IFG&UCTXIFG));
		
}


/// return sw and hw release
void getRelease(void){
	unsigned char check;
	char i;
	check = 0;
	for ( i = 0; i < 8; i++){
		while (!(UCA0IFG&UCTXIFG));
		UCA0TXBUF = releaseSW[i];
		check += releaseSW[i];
	}
	check += CHECKSUM;
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = check;
	while (!(UCA0IFG&UCTXIFG));
}

///
/// routine di trasmissione della flash
/// all packet is 8 bytes long + 1 byte CHECKSUM
/// first 2 byte are number of page in big endian format
/// MSB first, LSB last
void transmitAllFlash(void){
	
}


///
/// transmit block of 128 bytes
void transmitData(void){
	unsigned char dato, i;
	//unsigned int contatore;
	
	/*if (eepromData.readAddr < 256)
		initI2C_B0(1);
	else 
		initI2C_B0(0);*/
	
	/// transmit command
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = DOWNLOAD;
	dato = DOWNLOAD ^ CHECKSUM;
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = dato;
	while (!(UCA0IFG&UCTXIFG));
	dato = 0;
	/// if memoryFull == 1
	/// we unblock leds 
	if (eepromData.readBlocco == 0){
		P8OUT |= 2;			///LED primo blocco on
		P8OUT &= 0x5F;	///LED secondo blocco off
	}
	else{
		P8OUT |= 32;			///LED secondo blocco on	
		P8OUT &= 0x7D;		///LED primo blocco off
	}
	
	///
	// if (eepromData.readBlocco == 0 && eepromData.blocco == 1)
	// we write in II block but we read in first block
	if (eepromData.readBlocco == 0 && eepromData.blocco == 1){
		setBank(0);
		P8OUT |= 2;			///LED primo blocco on
		P8OUT &= 0x5F;	///LED secondo blocco off
	}
	/*
 * /// initialize eeprom structure
  eepromData.blocco = 0;
  eepromData.address = 0;
  eepromData.numPage = 0;
  eepromData.readAddr = 0;
 * 
 * */
 	/// flash reset
 	resetBlock = 1;
 	/// read block
 	do{
 		dato = readI2C_N_Byte(eepromData.readAddr, 128, cellBuffer);
 	} while (dato != OK);
 	eepromData.readAddr += 128;
 	if (eepromData.readAddr == 0){
 		/// we are at the end of the block 
 		if (eepromData.readBlocco == 0){
 			/// we are at the end of block 0
   		/// we restar from the beginnig so we must change block
   		setBank(1);
   		eepromData.readBlocco = 1;
   		P8OUT |= 32;			///LED secondo blocco on	
			P8OUT &= 0x7D;		///LED primo blocco off
 		}
 		else{
 			/// we are at the end of block 1
 			/// we must come back to block 0  ? boh! Yes because we can read again the data
 			setBank(0);
   		eepromData.readBlocco = 0;
   		P8OUT |= 2;			///LED primo blocco on
			P8OUT &= 0x5F;	///LED secondo blocco off
 		}
 	}
 	else{
 		/// we are inside the block and if block are different we need
 		/// to switch to eepromData.blocco
 		if (eepromData.readBlocco == 0 && eepromData.blocco == 1){
 			setBank(1);
 			P8OUT |= 32;			///LED secondo blocco on	
			P8OUT &= 0x7D;		///LED primo blocco off
 		}
 	}
 	
	/// transim block
	dato = 0;
	for (i = 0; i < 128; i++){
		while (!(UCA0IFG&UCTXIFG));
		UCA0TXBUF = cellBuffer[i];
		dato += cellBuffer[i];
	}
	dato ^= CHECKSUM;
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = dato;
	dato = 0;

}

///
/// connect directly MCU with PC
void directConnect(void){
	unsigned char dato, i;
	/// set flag on off
	dirConn ^= 1;
	/// re transmit command
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = DIRECT_CONNECT;
	dato = DIRECT_CONNECT ^ CHECKSUM;
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = dato;
	while (!(UCA0IFG&UCTXIFG));
	dato = 0;
}

///
/// reset all byte of flash
void clearFlash(unsigned char valore){
	unsigned char i, dato, statoLed;
	unsigned int cont;
	eepromData.address = 0;
	eepromData.blocco = 0;
	eepromData.numPage = 0;
  eepromData.readAddr = 0;
 
  /// transmit command
	while (!(UCA0IFG&UCTXIFG));
	if (valore == 0){
		UCA0TXBUF = CLEAR_FLASH;
		dato = CLEAR_FLASH ^ CHECKSUM;
	}
	else{
		UCA0TXBUF = SET_FLASH;
		dato = SET_FLASH ^ CHECKSUM;
	}
	while (!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = dato;
	while (!(UCA0IFG&UCTXIFG));
	dato = 0;
  
  for ( i = 0; i < 128; i++)
  	cellBuffer[i] = valore;
  __bic_SR_register(GIE);										// stop interrupt
  /// indica che occorrerà resettare il blocco parzialmente caricato
  resetBlock = 1;
  statoLed = P8OUT;
  P8OUT = 0;
  P8OUT = 64;
  /// inizializzo il primo blocco della flash
  initI2C_B0(eepromData.blocco);
  for (cont = 0; cont < 512; cont++){
  	/// giro su tutta la flash
		do{
			i = writeI2C_N_Byte(eepromData.address, eepromData.blocco, DIM_BUFF, cellBuffer );
		} while (i != OK);
		eepromData.address += 128;
		eepromData.numPage++;
  }
  //__bis_SR_register(GIE);										// enable interrupt
  eepromData.blocco = 1;
  eepromData.address = 0;
  /// inizializzo il secondo blocco della flash
  initI2C_B0(eepromData.blocco);
  for (cont = 0; cont < 512; cont++){
  	/// giro su tutta la flash
		do{
			i = writeI2C_N_Byte(eepromData.address, eepromData.blocco, DIM_BUFF, cellBuffer );
		} while (i != OK);
		eepromData.address += 128;
		eepromData.numPage++;
  }
  /// clrea some interrupt request: UART1, ADC, TIMER0: after reset of flash some sources will be cleared
  
  __bis_SR_register(GIE);									// enable interrupt
  P8OUT = statoLed;
	eepromData.blocco = 0;
  eepromData.address = 0;
  eepromData.numPage = 0;
  eepromData.readAddr = 0;
  eepromData.readBlocco = 0;
  initI2C_B0(eepromData.blocco);
  /// the memory is clear
  memoryFull = 0;
}

///
/// write some parameters in flash
///
void writeHWrelInFlash(unsigned char revisione[]){

unsigned int i;
  char *Flash_ptrD;
  Flash_ptrD = (char *) 0x1800;             // Initialize Flash segment D ptr

  __disable_interrupt();                    // 5xx Workaround: Disable global
                                            // interrupt while erasing. Re-Enable
                                            // GIE if needed
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                      // Set Erase bit
  *Flash_ptrD = 0;                          // Dummy write to erase Flash seg D
  FCTL1 = FWKEY+WRT;                        // Set WRT bit for write operation

  for (i = 0; i < 7; i++)
  {
    *Flash_ptrD++ = revisione[i];          // copy value segment C to seg D
  }

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                       // Set LOCK bi
  __bis_SR_register(GIE);										// enable interrupt
}

///
/// legge la release hw dalla flash
///
void readHWrelfromFlash(char revisione[]){
	char *Flash_ptrD, i;
  Flash_ptrD = (char *) 0x1800;             // Initialize Flash segment D ptr
  for (i = 0; i < 7; i++){
    revisione[i] = *Flash_ptrD++ ;          
  }
}

///
/// legge la release della board impostata dall'utente e la scrive in flash
///
void setHWkey(unsigned char chiaveHW[]){
	unsigned char i, check;
	i = check = 0;
	while ((i < 7) && (TX_PTR != RX_PTR)){
	  chiaveHW[i] = uartBuff[RX_PTR++];
	  RX_PTR &= DIM_UART_BUFF - 1;
	  check += chiaveHW[i];
	  i++;
	}
	/// check sum
	while(TX_PTR == RX_PTR);
	i = uartBuff[RX_PTR++];
	RX_PTR &= DIM_UART_BUFF - 1;
	if ((check ^ CHECKSUM) == i)
		/// ok, il check sum è corretto
		writeHWrelInFlash(chiaveHW);
}


/// 
/// memorizza il sampling time
///
unsigned char setSampTime(void){
	
	unsigned char valore, check;
  char *Flash_ptrC;
  while (TX_PTR == RX_PTR);
  valore = uartBuff[RX_PTR++];
  RX_PTR &= DIM_UART_BUFF - 1;
  while (TX_PTR == RX_PTR);
  check = uartBuff[RX_PTR++];
  RX_PTR &= DIM_UART_BUFF - 1;
  __disable_interrupt();
  //valore ^= CHECKSUM;
  if ((valore ^ CHECKSUM) != check)
  /// errore di checksum
  	return 255;
  
  Flash_ptrC = (char *) 0x1880;             // Initialize Flash segment C ptr
  __disable_interrupt();                    // 5xx Workaround: Disable global
                                            // interrupt while erasing. Re-Enable
                                            // GIE if needed
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                      // Set Erase bit
  *Flash_ptrC = 0;                          // Dummy write to erase Flash seg D
  FCTL1 = FWKEY+WRT;                        // Set WRT bit for write operation

  
  *Flash_ptrC++ = valore;          // copy value segment C to seg D
  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                       // Set LOCK bi
  __bis_SR_register(GIE);										// enable interrupt
  return 0;
}

///
///read sampling time
///
unsigned char readSampTime(void){
	
	char *Flash_ptrC;
  unsigned char ris;
  Flash_ptrC = (char *) 0x1880;             // Initialize Flash segment C ptr
  ris = *Flash_ptrC;
  return ris;
}

///
/// invia il comando "000" detto anceh TEST
///
void send_test_cmd(){
	char cmd[32];
	composeCMD(cmd, PREAMBLE, TEST, 3);
	checksum(cmd);
	append(cmd, "\r\n", 2);
	PRINTF(cmd);
	/// spedito il comando  occorre attendere la risposta, ma non qui, nel ciclo principale, gestendo i 
	/// flag FINE_GPS_MESS;
}


///
/// enable or disable GPS communications
///
void gpsOnOFF(char statoGPS){
	unsigned char dato;
	U1_Rec_PTR = U1_Read_PTR = 0;
	U1_TX1_PTR = 0;
	UCA1IE |= UCRXIE; 
	/// retransmit the command
	while (!(UCA0IFG & UCTXIFG));
	UCA0TXBUF = statoGPS;
	dato = statoGPS ^ CHECKSUM;
	while (!(UCA0IFG & UCTXIFG));
	UCA0TXBUF = dato;
}


