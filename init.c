
/*																																*
 * 				PROGETTO MISURA SFORZA IN CAMMINATA											*
 * 				REALIZZATO PER FACOLTA' SC. MOTORIE											*
 * 				URBINO																									*
 * 
 * 				DATA: 12/04/2012																				*
 * 				REV: 00.  REL 00: acq da singolo sensore								*
 * 																																*
 * 				init.c																									*
 * 																																
 * 				inizializzazione porte e periferiche										*
 * 																																*/
 
#include "msp430x54xA.h"
#include "init.h"
#include "gps.h"
 
extern volatile __eepromData eepromData; 
volatile unsigned char T0;

///
/// store if board is connect to terminal or to embedded sw
///
void init_MCUstatus(__MCUstatus *st){
	st->stato = EMB_SW;
	/// reserved
	st->speed = 0;
}

///
/// inizializza le porte
///
void init_PORT(void){
	
	P8DIR	=	0x7F;		/// porta 8 in uscita:  111 | 1111
	P8OUT = 0;			/// led spenti.

	P6SEL |= BIT0;	/// funzione alternativa su P6.0?
	P6SEL |= BIT1;	/// Funzione alternativa su P6.1: convertitore AD
	
	/// for timer debug we set up P4.6 (pin 52) and P4.5 (pin 48) as blinking pin
	P4DIR |= 0x70;	/// porta 4: 0 1 11|0000
	P4OUT = 0;
	/*
	P2DIR = 0;
	 * */
}


/// 
/// inizializza il timer per verificare che la scheda sta funzionando
/// e fornire un riferimento alla frequenza di campionamento
///
void init_TIMER(void){
	
	char * Flash_ptrC;
	unsigned int valore, ris;
	Flash_ptrC = (char *) 0x1880;   
	TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
  /// Uso del timer A1 associato al registro di fine conteggio CCR0
  if (*Flash_ptrC == 255)
  /// non inizializzata. campiono a 100 ms
  	//TA1CCR0 = 26214;
  	//TA0CCR0 = 46080;
  	TA0CCR0 = 37800;
  else{
  	ris = *Flash_ptrC & 255;
  	valore = ris * 378;
  	TA0CCR0 = valore;
  }
  T0 = 37800 / TA0CCR0;
  /// da NON commentare quando si usa SMCLK = 29.4912MHz
  /// SMCLK e' diviso per 64 e quindi si arriva a 29491200 / 64 = 460800
  /// impostando TA1CCR0 = 46080 si generano intervalli di 100 ms
  /// vedere init.h per la velocita' attuale del clock
  /// es.: a 24192000 si ha:24192000 / 64 = 378000 => TA1CCR0 = 37800 => 100ms
  TA0EX0 = TAIDEX_7;												// divide il clock ulteriormente per 8
  TA0CTL = TASSEL_2 + MC_1 + TACLR + ID__8; // SMCLK, upmode, clear TAR, divide clk 8 time
}

///
/// inizializza il canale 0 del convertitore AD
///
void init_ADC(unsigned char channel){
  
  /* Initialize the shared reference module */ 
  REFCTL0 |= REFMSTR + REFVSEL_0 + REFON;    // Enable internal 1.5V reference
  
  /* Initialize ADC12_A */ 
  //ADC12CTL0 = ADC12SHT0_8 + ADC12ON;				// 256 ciycles set sample time 
  ADC12CTL0 = ADC12SHT0_3 + ADC12ON;				// 32 ciycles set sample time
  
  // Enable sample timer from suorce timer and select ADC12SC as S.O.C.
  // clkc source is ADC12OSC, clk non divided
  ADC12CTL1 = ADC12SHP;     
  
  switch(channel){
  	case 1:
  	/// by this reg we select: reference ( 1 => + = Vref, - = AVSS )              
  	ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_6;  // ADC input ch A6 => op amp from load cell 
  	break;
  	
  	case 10:
  		ADC12CTL0 = ADC12SHT0_8 + ADC12ON;				// 256 ciycles set sample time 
  		/// by this reg we select: reference ( 1 => + = Vref, - = AVSS )              
  		ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_10;  // ADC input ch A10 => temp sense Tsampe = 30us
  	break;
  	
  	case 11:
  		ADC12CTL0 = ADC12SHT0_2 + ADC12ON;				// 16 ciycles set sample time
  		ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_11;  // ADC input ch A11 => Vmid. Tsample = 1us
  		
  	break;
  	
  	default:
  	break;
  
  }  
  /// deafult is 12 bit res, and we explicitely set 12 bit res
  ADC12CTL2 |= ADC12RES_2;
  /// deafult is 12 bit res, and we explicitely set 8 bit res
  //ADC12CTL2 |= ADC12RES_0;
  
  //ADC12IE = 0x001;                          // ADC_IFG upon conv result-ADCMEMO
  
  __delay_cycles(75);                       // 35us delay to allow Ref to settle
                                            // based on default DCO frequency.
                                            // See Datasheet for typical settle
                                            // time.
  /// Enable of conversion
  ADC12CTL0 |= ADC12ENC;

	/*P7DIR |= BIT2 + BIT3;
	P7OUT = BIT3;
	P7SEL |= BIT6;*/
}

///
/// adc used with interrupt
///
void init_ADC_int(unsigned char channel){
  
  /* Initialize the shared reference module */ 
  REFCTL0 |= REFMSTR + REFVSEL_0 + REFON;    // Enable internal 1.5V reference
  
  /* Initialize ADC12_A */ 
  ADC12CTL0 = ADC12SHT0_8 + ADC12ON;				// 256 ciycles set sample time at least 30us @ 29MHz
  //ADC12CTL0 = ADC12SHT0_3 + ADC12ON;				// 32 ciycles set sample time
  ADC12CTL0 = ADC12SHT0_4 + ADC12ON;				// 64 ciycles set sample time
  
  // Enable sample timer from suorce timer and select ADC12SC as S.O.C.
  // clkc source is ADC12OSC, clk non divided
  ADC12CTL1 = ADC12SHP;     
  
  switch(channel){
  	case 1:
  	/// by this reg we select: reference ( 1 => + = Vref, - = AVSS )              
  	ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_6;  // ADC input ch A6 => op amp from load cell 
  	break;
  	
  	case 10:
  		/// by this reg we select: reference ( 1 => + = Vref, - = AVSS )              
  		ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_10;  // ADC input ch A10 => temp sense 
  	break;
  	
  	default:
  	break;
  
  }  
  /// deafult is 12 bit res, and we explicitely set 12 bit res
  ADC12CTL2 |= ADC12RES_2;
  /// deafult is 12 bit res, and we explicitely set 8 bit res
  //ADC12CTL2 |= ADC12RES_0;
  
  ADC12IE = 0x001;                          // ADC_IFG upon conv result-ADCMEMO
  
  //__delay_cycles(75);                       // 35us delay to allow Ref to settle
  __delay_cycles(3000);                       // 35us delay to allow Ref to settle @ 29.4MHz
                                            // based on default DCO frequency.
                                            // See Datasheet for typical settle
                                            // time.
  /// Enable of conversion
  ADC12CTL0 |= ADC12ENC;

}


///
/// setup ADC; misura l'offset e lo restituisce
///
unsigned int setupADC(void) {

	unsigned int risultato;
	__bic_SR_register(GIE);	
	// Sampling and conversion start
	ADC12CTL0 |= ADC12SC;                   
	/// try with polling
	while(ADC12CTL1 & ADC12BUSY);
	risultato = ADC12MEM0;
	__delay_cycles(100); 
	ADC12IFG = 0;
	__bis_SR_register(GIE);	
	return risultato;
	
}

///
/// inizializza la uart
///
void init_UART0(unsigned long int value){
	unsigned int baud;
	P3SEL = 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
 
	baud = FDCO / value;											// SEE init.h for actual value 
	UCA0BR0 = baud & 0xFF;
	UCA0BR1 = (baud >> 8) & 0xFF;
	
	
/// 115200 b/s when smck = 29.491200MHz.   VERIFICATA: funziona
//	  UCA0BR0 = 0x00;															// 29.491200MHz => 115200 bit/s
//	  UCA0BR1 = 0x1;															// 29.491200MHz => 115200 bit/s
//	  UCA0MCTL |= UCBRS_0 + UCBRF_0;            // Modulation UCBRSx=0, UCBRFx=0 
	
  
/// 115200 b/s when smck = 4.1943MHz
//  UCA0BR0 = 36;															// 4.1943MHz => 115200 bit/s
//  UCA0BR1 = 0;															// 4.1943MHz => 115200 bit/s
//  UCA0MCTL |= UCBRS_3 + UCBRF_0;            // Modulation UCBRSx=3, UCBRFx=0

/// 9600 b/s when smck = 29.491200MHz			VERIFICATA: funziona
//	  UCA0BR0 = 0x00;															// 29.491200MHz => 9600 bit/s
//	  UCA0BR1 = 0xC;															// 29.491200MHz => 9600 bit/s
//	  UCA0MCTL |= UCBRS_0 + UCBRF_0;            // Modulation UCBRSx=0, UCBRFx=0 
	
 
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

 /*P3SEL = 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 9;                              // 1MHz 115200 (see User's Guide)
  UCA0BR1 = 0;                              // 1MHz 115200
  UCA0MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  */

///
/// inizializza la uart
/// for UP501 GPS default is 9600b/s
/// see UP501_datasheet page 5
///
void init_UART1(unsigned long int value){
	unsigned int baud;
	P5SEL = 0xC0;                             // P5.7,6 = USCI_A1 TXD/RXD
	/// UCA1CTL0, bit 5 set MSB first or LSB first. see user guide pag. 594 
  UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA1CTL1 |= UCSSEL_2;                     // SMCLK
 
//  UCA0BR0 = 9;                              // 1MHz 115200 (see User's Guide)
//  UCA0BR1 = 0;                              // 1MHz 115200
   // RIMUOVERE IL COMMENTO PER SMCLK = 4.1943 MHz
//  UCA1BR0 = 36;															// 4.1943MHz => 115200 bit/s
//  UCA1BR1 = 0;															// 4.1943MHz => 115200 bit/s
//  //UCA0MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
//  // RIMUOVERE IL COMMENTO PER SMCLK = 4.1943 MHz
//  UCA1MCTL |= UCBRS_3 + UCBRF_0;            // Modulation UCBRSx=3, UCBRFx=0
  
  /// 9600 b/s when smck = 16.7772MHz
//  UCA1BR0 = 0xD3;															// 16.7772MHz => 9600 bit/s
//  UCA1BR1 = 0x6;															// 16.7772MHz => 9600 bit/s
//  UCA1MCTL |= UCBRS_5 + UCBRF_0;            // Modulation UCBRSx=5, UCBRFx=0
  
  /// 115200 b/s when smck = 16.7772MHz
//  UCA1BR0 = 0x91;															// 16.7772MHz => 115200 bit/s
//  UCA1BR1 = 0x0;															// 16.7772MHz => 115200 bit/s
//  UCA1MCTL |= UCBRS_5 + UCBRF_0;            // Modulation UCBRSx=5, UCBRFx=0 
 
/// 115200 b/s when smck = 19.988480MHz
//  UCA1BR0 = 0xAD;															// 19.988480MHz => 115200 bit/s
//  UCA1BR1 = 0x0;															// 19.988480MHz => 115200 bit/s
//  UCA1MCTL |= UCBRS_5 + UCBRF_0;            // Modulation UCBRSx=5, UCBRFx=0 
   
  /// 9600 b/s when smck = 19.988480MHz
//  UCA1BR0 = 0x22;															// 19.988480MHz => 9600 bit/s
//  UCA1BR1 = 0x8;															// 19.988480MHz => 9600 bit/s
//  UCA1MCTL |= UCBRS_3 + UCBRF_0;            // Modulation UCBRSx=3, UCBRFx=0 
  
/// 115200 b/s when smck = 29.491200MHz
//  UCA1BR0 = 0x00;															// 29.491200MHz => 115200 bit/s
//  UCA1BR1 = 0x1;															// 29.491200MHz => 115200 bit/s
//  UCA1MCTL |= UCBRS_0 + UCBRF_0;            // Modulation UCBRSx=0, UCBRFx=0 
  
/// 9600 b/s when smck = 29.491200MHz
//  UCA1BR0 = 0x00;															// 29.491200MHz => 9600 bit/s
//  UCA1BR1 = 0xC;															// 29.491200MHz => 9600 bit/s
//  UCA1MCTL |= UCBRS_0 + UCBRF_0;            // Modulation UCBRSx=0, UCBRFx=0 

	baud = FDCO / value;											// SEE init.h for actual value (29491200 Hz)
	UCA1BR0 = baud & 0xFF;
	UCA1BR1 = (baud >> 8) & 0xFF;
  
//  UCA1BR0 = 0xB4;															// 4.1943MHz => 9600 bit/s
//  UCA1BR1 = 0x1;															// 4.1943MHz => 9600 bit/s
//  UCA1MCTL |= UCBRS_7 + UCBRF_0;            // Modulation UCBRSx=7, UCBRFx=0
  
  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
}


  void initEepromData(){
  	/// initialize eeprom structure
  	eepromData.blocco 		= 0;			// blocco attuale
  	eepromData.address 		= 0;			// indirizzo dell'ultimo dato scritto
  	eepromData.numPage 		= 0;			// numero di pagine scritte 
  	eepromData.readAddr 	= 0;			// indirizzo dell'ultimo dato letto
  	eepromData.readBlocco = 0;			// puntatore all'ultimo blocco letto
  	//eepromData.lastBlock  = 0;
  
  
}

///
/// init GPS data struct
///
void init_GPSstatus(GPSstatus *gpsSt, GPSmessStatus *gpsMSt){
	gpsSt->status = GPS_OFF;
	gpsSt->mode = NOTVALID;
	gpsSt->GGA = NOTVALID;
	gpsSt->GLL = NOTVALID;
	gpsSt->GSA = NOTVALID;
	gpsSt->GSV = NOTVALID;
	gpsSt->RMC = NOTVALID;
	gpsSt->VTG = NOTVALID;
	gpsSt->ZDA = NOTVALID;
	
	gpsMSt->test = NOTESTMESS;
}

