
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

///
/// Registers set for UCB0 I2C interface
/// 2 wires
///
/*
 
	P3.1 : UCB0SDA				<-->
	P3.2 : UCB0SCL				--->
	
	P2.5 : digital : write protect, active high
	
	address: A2 - A1 : 0 0;   A0 : not connected in 1Mb flash 
*/

#include "msp430x54xa.h"
#include "init.h"

///
/// set memory bank
/// only for flash of 128kb size
///
void setBank(unsigned char block){
	initI2C_B0(block);
}
///
/// this function initialize I2C on B0 UART and select what block 
/// on 128kb flash
void initI2C_B0(unsigned char block){
	
	/// P2.5 is connected to WP and is active high.
	P2DIR |= BIT5;
	P2OUT |= BIT5;														// write protect
	/// P3.2 and P3.1 in alternative mode
	//P3SEL |= 0x06;                            // Assign I2C pins to USCI_B0
	if (UCB0STAT & UCBBUSY)
		UCB0CTL1 = 0;
	UCB0CTL1 |= UCSWRST;                      // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  //UCB0BR0 = 3;                             // fSCL = SMCLK/3 = ~333kHz
  //UCB0BR0 = 5;                             // fSCL = SMCLK/5 = ~200kHz
  //UCB0BR0 = 10;                             // fSCL = SMCLK/10 = ~100kHz
  // UCB0BR0 = 100;													// fSCL = SMCLK/100 = ~295kHz @ 29.4912MHz
  // UCB0BR0 = 80;													// fSCL = SMCLK/80 = ~369kHz @ 29.4912MHz
  // UCB0BR0 = 60;													// fSCL = SMCLK/60 = ~403kHz @24.192000 MHz
  // UCB0BR0 = 40;													// fSCL = SMCLK/40 = ~605kHz @24.192000 MHz
  UCB0BR0 = FDCO / 990000;
  //UCB0BR0 = 40;                             
  UCB0BR1 = 0;
  
  
  /* we need to set correctly this address following 64kb or 128kb flash */
#ifdef __EEPROM128K__
	UCB0I2CSA = 0x50 | (block & 1);						// Slave address is 5xh:   1010 00x
#else
  UCB0I2CSA = 0x51;                         // Slave Address is 051h:  1010 001 in WSN board
//  pippo
#endif
	/// P3.2 and P3.1 in alternative mode
	P3SEL |= 0x06;                            // Assign I2C pins to USCI_B0
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  __delay_cycles(5);
  //UCB0IE |= UCRXIE;                         // Enable RX interrupt
}

///
/// this function read a byte at stored address
unsigned char readI2CByte(){

	unsigned char tmp, flag;
	
	/// send STOP
	//UCB0CTL1 |= UCTXSTP;
	flag = 1;
	do	{
		//while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
		UCB0CTL1 &= ~UCTR;											// clear tx mode
	  UCB0CTL1 |= UCTXSTT;                    // I2C start condition
	  /// wait start transmission
	  while (UCB0CTL1 & UCTXSTT);
	  /// and send stop condition (page 629 of UG)
	  UCB0CTL1 |= UCTXSTP;                		// Generate I2C stop condition
		while (!(UCB0IFG & UCRXIFG)){
			/// a NACK is produced?
			if (UCB0IFG & UCNACKIFG){
				UCB0IFG &= ~UCNACKIFG;
				/// if yes, reset and repeat the cycle
				break;
			}
			else
				flag = 0;
		}						/// wait a char
	}while(flag);
	
	tmp = UCB0RXBUF;
	return tmp;
}

///
/// this function read a byte at specified address
unsigned char readI2CByteFromAddress(unsigned int address, int *status){

	unsigned char tmp = (address >> 8);
	*status = 0;
	/// write enable
	P2OUT |= BIT5;
	
	while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
	/// start with a write session
	UCB0CTL1 |= UCTR + UCTXSTT;												// transmit
	//UCB0CTL1 |= UCTXSTT;                    // I2C start condition
	/// wait start transmission
	/// As stated in UG, page 627, UCTXSTT is cleared when address is transmitted
	/// from master to slave AND slave ACKNOWLEDGE the address
  //while (UCB0CTL1 & UCTXSTT);
  
  /// wait tx buffer free (it already free)
  while(!(UCB0IFG & UCTXIFG));
  /// transmit addr_h byte
  UCB0TXBUF = tmp;
  /// a NACK is arrived?  Write cycle isn't finished
  if (UCB0IFG & UCNACKIFG){
  	/// send a STOP
  	UCB0CTL1 |= UCTXSTP;
  	UCB0IFG &= ~UCNACKIFG;
  	*status = NACK_ERR;
  	return 0;
  }
  tmp = (address & 0xff);
  /// wait tx buffer free
  while(!(UCB0IFG & UCTXIFG));
  /// transmit addr_h byte
  UCB0TXBUF = tmp;
  
  /// as soon data is transferred from buffer to tx reg
  /// it send a RESTART SIGNAL
  while(!(UCB0IFG & UCTXIFG));
  UCB0CTL1 &= ~UCTR;											// receive
	UCB0CTL1 |= UCTXSTT;                    // I2C start condition
  UCB0IFG &= ~UCTXIFG;										// clear tx flag
  
  /// wait start transmission
  while (UCB0CTL1 & UCTXSTT);
  /// and send stop condition (page 629 of UG)
  UCB0CTL1 |= UCTXSTP;                		// Generate I2C stop condition
	while (!(UCB0IFG & UCRXIFG));						/// wait a response
	
	tmp = UCB0RXBUF;
	*status = OK;
	return tmp;
}

///
/// this function read N bytes at specified address
char readI2C_N_Byte( unsigned int address, unsigned char numElm, volatile unsigned char buff[] ){

	unsigned char tmp = (address >> 8), i;
	unsigned int numGiri = 0;
	/// write enable
	P2OUT |= BIT5;
	//UCB0CTL1 |= UCTXSTP;                			// Generate I2C stop condition
	//__delay_cycles(750);
	/// prima di tutto occorre vedere se il bus e' impegnato
	if (UCB0STAT & UCBBUSY){
		//// e' impegnato, qualcosa e' andato storto
		P3SEL &= 0xF9;	/// rimetto SCL e SDA in funzione digitale 
		P3DIR &= 0xFD;  /// SDA in ingresso
		P3OUT &= 0xFB;  /// metto SCL a 0;
		P3DIR |= 0x4;		/// P3.2 basso (SCL)
		/// in che stato e' SDA?
		while(!(P3IN & BIT1)){
		/// un po' di colpetti su SCL per sbloccare l'automa dello slave
			for (i = 0; i < 10; i++){
				__delay_cycles(500);
				P3OUT ^= BIT2;
			}
			/// SDA alto:
			P3OUT |= BIT1;
			P3DIR |= BIT1;
			__delay_cycles(10);
			P3DIR &= 0xFD;	/// SDA in ingresso
		}
		P3SEL |= 0x6;		/// SCL + SDA
		return BUS_BUSY;
	}
	
	while (UCB0CTL1 & UCTXSTP);             	// Ensure stop condition got sent
	/// start with a write session
	UCB0CTL1 |= UCTR + UCTXSTT;								// transmit
	//UCB0CTL1 |= UCTXSTT;                    // I2C start condition
	/// wait start transmission
	/// As stated in UG, page 627, UCTXSTT is cleared when address is transmitted
	/// from master to slave AND slave ACKNOWLEDGE the address
  //while (UCB0CTL1 & UCTXSTT);
  
  /// wait tx buffer free (it already free)
  while(!(UCB0IFG & UCTXIFG));
  /// transmit addr_h byte
  UCB0TXBUF = tmp;
  /// a NACK is arrived?  Write cycle isn't finished
  if (UCB0IFG & UCNACKIFG){
  	/// send a STOP
  	UCB0CTL1 |= UCTXSTP;
  	UCB0IFG &= ~UCNACKIFG;
  	return NACK_ERR;
  }
  tmp = (address & 0xff);
  /// wait tx buffer free
  while(!(UCB0IFG & UCTXIFG)){
  	
  	if ((++numGiri > 1000) && (UCB0STAT & UCBBUSY)){
  		/// so cazzi perché mo' me so intruppato
  		P3SEL &= 0xFB;	/// rimetto SCL in funzione digitale 
  		P3OUT &= 0xFB;
  		P3OUT |= 0x4;		/// P3.2 basso (SCL)
  		P3SEL |= 0x6;		/// SCL + SDA
  		return BUS_BUSY;
  	}
  }
  /// transmit addr_h byte
  UCB0TXBUF = tmp;
  
  /// as soon data is transferred from buffer to tx reg
  /// it send a RESTART SIGNAL
  while(!(UCB0IFG & UCTXIFG));
  UCB0CTL1 &= ~UCTR;											// receive
	UCB0CTL1 |= UCTXSTT;                    // I2C start condition
  UCB0IFG &= ~UCTXIFG;										// clear tx flag
  
  /// wait start transmission
  while (UCB0CTL1 & UCTXSTT);
  for ( i = 0; i < numElm; i++){
  	while (!(UCB0IFG & UCRXIFG));						/// wait a response
  	buff[i] = UCB0RXBUF;
  	if ( i == numElm - 2)
  		UCB0CTL1 |= UCTXSTP;                		// Generate I2C stop condition
  }
  /// and send stop condition (page 629 of UG)
  //UCB0CTL1 |= UCTXSTP;                		// Generate I2C stop condition
	
	return OK;
}

/* 																																				*
 * 															WRITE FUNCTIONS														*
 * 																																				*/
///
/// this function write a byte at specified address
 char writeI2CByte(unsigned char data, unsigned int address){

	/// as stated in 24AA512 datasheet, the sequence is:
	/// start byte, addr_h, addr_l, data
	unsigned char tmp;
	
	tmp = (address >> 8);
	P2OUT &= ~BIT5;													// write enable
	
	while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
	UCB0CTL1 |= UCTXSTT + UCTR;             // I2C start condition
	/// wait start transmission
	/// As stated in UG, page 627, UCTXSTT is cleared when address is transmitted
	/// from master to slave AND slave ACKNOWLEDGE the address
  //while (UCB0CTL1 & UCTXSTT);
  
  /// wait tx buffer free (it already free because as shown in fig. 28.12,
  /// UCTXIFG is set after a start command 
  while(!(UCB0IFG & UCTXIFG));
  /// transmit addr_h byte
  UCB0TXBUF = tmp;
  tmp = (address & 0xff);
  /// however a NACK could happen, if a previous write cycle is not finished,
  /// so wait tx buffer free will be an infinite cycle
  while (!(UCB0IFG & UCTXIFG)){
	 	/// if NACK is produced, UCTXIFG = 0 and thus we are here
  	if (UCB0IFG & UCNACKIFG){
  		/// there is a NACK. Fig. 28.12 assert that stop signal must be done
  		UCB0IFG &= ~UCNACKIFG;
  		// A stop condition is built and we check it next time we call this function
  		UCB0CTL1 |= UCTXSTP;
  		///Exit from this while cycle and try again
  		return NACK_ERR;
  	}
  }
	//while(!(UCB0IFG & UCTXIFG));
	/// ok, an ACK is arrived and we can transmit
	/// transmit addr_h byte
	UCB0TXBUF = tmp;
 
  /// transmit data
  while(!(UCB0IFG & UCTXIFG)){
	 	/// if NACK is produced, UCTXIFG = 0 and thus we are here
  	if (UCB0IFG & UCNACKIFG){
  		/// there is a NACK. Fig. 28.12 assert that stop signal must be done
  		UCB0IFG &= ~UCNACKIFG;
  		// A stop condition is built and we check it next time we call this function
  		UCB0CTL1 |= UCTXSTP;
  		///Exit from this while cycle and try again
  		return NACK_ERR;
  	}
  }
  /// transmit addr_h byte
  UCB0TXBUF = data;
  
  /// check if data is shifted from buffer to tx register
  /// as soon data is shifted in tx register, it send a STOP signal
  while(!(UCB0IFG & UCTXIFG));
  /// and now it must send stop 
  UCB0CTL1 |= UCTXSTP;                // Generate I2C stop condition
  return OK;
	
}

///
/// This function write N byte at specified address
/// or execute a write page mode
/// 
 char writeI2C_N_Byte(unsigned int address, unsigned char block, 
 											unsigned char numElem, volatile unsigned char buff[] ){

	unsigned int numGiri = 0;
	/// NOTICE: CHECK SIDE EFFECT WITH TIMER INTERRUPT.
	//setBank(block);
	initI2C_B0(block);
	/// If we use this function to write a page, numElem = 128 and  address doesn't use 
	/// the 7 LSB bit of address.
	if (numElem == 128)
		address &= 0xFF80; 
	/// as stated in 24AA512 datasheet, the sequence is:
	/// start byte, addr_h, addr_l, data
	unsigned char tmp = (address >> 8), i;
	
	P2OUT &= ~BIT5;													// write enable
	if (UCB0STAT & UCBBUSY){
		//// e' impegnato, qualcosa e' andato storto
		P3SEL &= 0xF9;	/// rimetto SCL e SDA in funzione digitale 
		P3DIR &= 0xFD;  /// SDA in ingresso
		P3OUT &= 0xFB;  /// metto SCL a 0;
		P3DIR |= 0x4;		/// P3.2 basso (SCL)
		/// in che stato e' SDA?
		while(!(P3IN & BIT1)){
		/// un po' di colpetti su SCL per sbloccare l'automa dello slave
			for (i = 0; i < 10; i++){
				__delay_cycles(500);
				P3OUT ^= BIT2;
			}
			/// SDA alto:
			P3OUT |= BIT1;
			P3DIR |= BIT1;
			__delay_cycles(10);
			P3DIR &= 0xFD;	/// SDA in ingresso
		}

		P3SEL |= 0x6;		/// SCL + SDA
		return BUS_BUSY;
	}
	
	while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
	UCB0CTL1 |= UCTXSTT + UCTR;             // I2C start condition
	/// wait start transmission
	/// As stated in UG, page 627, UCTXSTT is cleared when address is transmitted
	/// from master to slave AND slave ACKNOWLEDGE the address
  //while (UCB0CTL1 & UCTXSTT);
  
  /// wait tx buffer free (it already free)
  while(!(UCB0IFG & UCTXIFG));
  /// transmit addr_h byte
  UCB0TXBUF = tmp;
  tmp = (address & 0xff);
  /// wait tx buffer free
  while(!(UCB0IFG & UCTXIFG)){
  /// if NACK is produced, UCTXIFG = 0 and thus we are here
  	if (UCB0IFG & UCNACKIFG){
  		/// there is a NACK. Fig. 28.12 assert that stop signal must be done
  		UCB0IFG &= ~UCNACKIFG;
  		// A stop condition is built and we check it next time we call this function
  		UCB0CTL1 |= UCTXSTP;
  		///Exit from this while cycle and try again
  		return NACK_ERR;
  	}
  	numGiri++;
  	if (numGiri > 1000 && (UCB0STAT & UCBBUSY)){
  	/// so cazzi perché mo' me so intruppato
  		P3SEL &= 0xFB;	/// rimetto SCL in funzione digitale 
  		P3OUT &= 0xFB;
  		P3OUT |= 0x4;		/// P3.2 basso (SCL)
  		P3SEL |= 0x6;		/// SCL + SDA
  		return BUS_BUSY;
  	}
  }
  /// transmit addr_h byte
  UCB0TXBUF = tmp;
  
  /// transmit data
  for (i = 0; i < numElem; i++){
	  while(!(UCB0IFG & UCTXIFG)){
		  /// if NACK is produced, UCTXIFG = 0 and thus we are here
	  	if (UCB0IFG & UCNACKIFG){
	  		/// there is a NACK. Fig. 28.12 assert that stop signal must be done
	  		UCB0IFG &= ~UCNACKIFG;
	  		// A stop condition is built and we check it next time we call this function
	  		UCB0CTL1 |= UCTXSTP;
	  		///Exit from this while cycle and try again
	  		return NACK_ERR;
	  	}
	  }
	  /// transmit byte
	  UCB0TXBUF = buff[i];
  }
  
  /// check if data is shifted from buffer to tx register
  /// as soon data is shifted in tx register, it send a STOP signal
  while(!(UCB0IFG & UCTXIFG));
  /// and now it must send stop 
  UCB0CTL1 |= UCTXSTP;                // Generate I2C stop condition
  //__delay_cycles(1);
  while (UCB0CTL1 & UCTXSTP); 
  return OK;
}
