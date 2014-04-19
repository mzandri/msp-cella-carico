
#include "msp430x54xA.h"
#include "init.h"
#include "gps.h"

volatile unsigned char uart1RXBUFF[DIM_UART1_RX_BUFF];
volatile unsigned int U1_Rec_PTR = 0, U1_Read_PTR = 0, U1_end_MESS = 0;
volatile unsigned char FINE_GPS_MESS = 0, INIZIO_GPS_MESS = 0, MATCH_DELIMITER = FALSE, NUM_MESS = 0;
extern volatile unsigned char uart1TXBUFF[], U1_TX1_PTR;
extern data GPSdata;


///
/// routine che serve le comunicazioni con il modulo GPS
///
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
	unsigned char c;
	switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:  
    //P8OUT ^= 1;                             // Vector 2 - RXIFG
    /// read byte
    c = UCA1RXBUF;
//    if (MATCH_DELIMITER)
//    	uart1RXBUFF[U1_Rec_PTR++] = c;
    uart1RXBUFF[U1_Rec_PTR++] = c;
    if (c == '\n' && MATCH_DELIMITER == TRUE){
    	FINE_GPS_MESS = 1;
    	INIZIO_GPS_MESS == 0;
    	NUM_MESS++;
    	MATCH_DELIMITER = FALSE;
    	U1_end_MESS = U1_Rec_PTR - 1;
    }
    else if (c == '$'){
    	INIZIO_GPS_MESS == 1;
    	FINE_GPS_MESS = 0;
    	MATCH_DELIMITER = TRUE;
    }
    /// mod 512
    //U1_Rec_PTR &= DIM_UART1_RX_BUFF - 1;	
    if (U1_Rec_PTR > 511)
    	U1_Rec_PTR = 0;
    //while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
    //UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
    //P8OUT ^= 1;
    break;
  case 4:
  	// transmit a byte through serial1
  	UCA1TXBUF = uart1TXBUFF[U1_TX1_PTR++];
  	/// mod DIM_UART1_TX_BUFF
  	U1_TX1_PTR &= DIM_UART1_TX_BUFF - 1;
  	/// when \0 is found it stop to tansmit
  	if ( uart1TXBUFF[U1_TX1_PTR]  == 0){
  	/// stop transmission
  		UCA1IE = UCRXIE;                         // Enable USCI_A1 RX interrupt
  		U1_TX1_PTR = 0;
  	}
  break;                             // Vector 4 - TXIFG
  default: break;
  }
}


///
///
///
extern volatile unsigned char uartBuff[], TX_PTR;

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:  
    //P8OUT ^= 1;                             // Vector 2 - RXIFG
    /// read byte
    uartBuff[TX_PTR++] = UCA0RXBUF;
    /// mod 64
    TX_PTR &= DIM_UART_BUFF - 1;	
    /// echo, solo per debug con hyperterminal
//    while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
//    UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
    //P8OUT ^= 1;
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}



extern volatile unsigned char cellBuffer[], CELL_BUFF_PTR, WRITE_FLASH;
extern volatile int letturaADC, SEND_DATA_TO_PC, FIRST_BLOCK_WRITTEN;
static volatile unsigned int contatore = 0;
static volatile unsigned char cell_buff_adj = 0;
///
/// AD converter
///
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
    letturaADC += ADC12MEM0;                       // Move results, IFG is cleared
    contatore++;
    if (contatore < 4)
    	ADC12CTL0 |= ADC12SC;                   // Sampling and conversion start
    else {
    	letturaADC >>= 2;
    	//letturaADC -= offsetADC;							// potra' essere utile eliminare l'offset
    	
    	SEND_DATA_TO_PC = 1;
    	/// se CELL_BUFF_PTR > 127 significa che si ha una pagina da scrivere in flash
    	/// in questo caso, si comunica che è ora ma si continuerà nella parte superiore
    	/// della ram ricopiando una volta che è avvenuta la scrittura.
//    	if (CELL_BUFF_PTR < 128){
    		cellBuffer[CELL_BUFF_PTR] = (unsigned char) (letturaADC >> 4);
    		CELL_BUFF_PTR++;
    		cell_buff_adj = 0;
    		if (CELL_BUFF_PTR >= DIM_BUFF){
					WRITE_FLASH = 1;
					//CELL_BUFF_PTR = 0;
	    	}
//    	else {
    		
//    		if (FIRST_BLOCK_WRITTEN == 0x0){
//    		/// non ha ancora scritto il blocco da 128 bytes e quindi slavo nella seconda parte del buffer
//    		cellBuffer[128 + cell_buff_adj] = (unsigned char) letturaADC;
//    		cell_buff_adj++;
//    		cell_buff_adj &= DIM_SEC_BUFF - 1;	//la seconda parte del buffer e' modulo 128
    		
//    		}
//    		else{
//    			/// FIRST_BLOCK_WRITTEN == 1
//    			/// copio la seconda parte nella prima, resetto il flag, imposto CELL_BUFF_PTR
//    			memcpy((void *) cellBuffer, (void *) &cellBuffer[128], cell_buff_adj);
//    			/// CELL_BUFF_PTR torna al valore salvato nel second oblocco.
//    			CELL_BUFF_PTR = cell_buff_adj - 1;
//    			FIRST_BLOCK_WRITTEN = 0;
//    			}
    	
    	contatore = 0;
    	
    }
    break;
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}
