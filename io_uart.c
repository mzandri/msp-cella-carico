
#include "msp430x54xA.h"
#include "init.h"

static unsigned char numPorta;
//volatile extern unsigned char uart1TX_BUF[16], UTX1_PTR = 0;
//extern volatile unsigned char uart1TXBUFF[], UTX1_PTR;
volatile unsigned char uart1TXBUFF[DIM_UART1_TX_BUFF], U1_TX1_PTR = 0;


void setPort(unsigned char nP){

	numPorta = nP;
}

//void PRINTF(char *st){
//	//unsigned char cont = 0;
//	if (numPorta == 1){
//		/// scan and transmit the string null terminated
//		while (*st != 0){
//			while (!(UCA1IFG & UCTXIFG));
//			UCA1TXBUF = *st;
//			st++;
//		}
//	}
//	else{
//		/// scan and transmit the string null terminated
//		while (*st != 0){
//			while (!(UCA0IFG & UCTXIFG));
//			UCA0TXBUF = *st;
//			st++;
//		}
//	}
//}

///
/// scrive sulla seriale numero 1
///
char PRINTF(char *st){
	
	unsigned char cont = 0;
	char status;
	if ((UCA1IE & UCTXIE) == 0){
		/// tutto il precedente e' stato trasmesso
		if (numPorta == 1){
			while (*st != 0 && cont < 16){
				uart1TXBUFF[cont++] = *st;
				st++;
			}
			/// terminate the string
			uart1TXBUFF[cont] = 0;
			/// start TX by interrupt
			UCA1IE |= UCTXIE; 
		}
		status = OK;
	}
	else
		status = 1;
	
	return status;
}


/// 
/// scrive sulla seriale 0
///
void PRINTF0(char *st){
	unsigned char cont = 0; 
	while (*st != 0 && cont < 24){
		while (!(UCA0IFG & UCTXIFG));
		UCA0TXBUF =  *st;
		st++;
		cont++;
	}		 	
}

/// 
/// stampa un unsigned char in codifica ascii
///
void PRINTFU(char st){
	unsigned char r[3]; 
	r[0] = st % 10 + '0';
	st = st / 10;
	r[1] = st % 10 + '0';
	st = st / 10;
	r[2] = st % 10+ '0';
	for ( st = 0; st < 3 ; st++){
		while (!(UCA0IFG & UCTXIFG));
		UCA0TXBUF =  r[2 - st];
	}
	PRINTF0("\r\n\0");		 	
}
