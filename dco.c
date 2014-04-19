

#include "msp430x54xA.h"
#include "HAL_PMM.h"
#include "init.h"


/*
 /// se non vengono impostati i registri UCS, il funzionamento
  /// predefinito è il seguente:
  /// si parte con XT1, (SELREF = 0) in UCSCTL3, ma se tale
  /// quarzo non è montato viene generata una eccezione ed automaticamente
  /// si passa a SELREF = 2 cioe' al riferimento interno da 32768Hz.
  /// Siccome FLLN in UCSCTL2 e' uguale a 31, si ottiene un fattore moltiplicativo
  /// di 31 + 1 = 32 => 32768*32 = 1048576 e fDCOCLOCK = 1048576 * 2
  /// Affinche' venga generata la opportuna frequenza, il DCO deve essere portato a funzionare
  /// nell'intervallo desiderato. cfr. pag. 51 del manuale SLASS655B e pag. 100 della user guide
  /// Il valore di DCOSEL = 2 che significa un intervallo da 300 kHz a 3.7MHz, il che torna con quanto
  /// precedentemente scritto.
  /// Per aumentare la frequenza sopra gli 8-9MHz, occorre impostare i registri di alimentazione del nucleo del
  /// processore.
  
  UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
  UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

  __bis_SR_register(SCG0);                  // Disable the FLL control loop
  UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
  UCSCTL1 = DCORSEL_7;                      // Select DCO range 24MHz operation
  UCSCTL2 = FLLD_0 + 610;		            // Set DCO Multiplier for 12MHz
                                            // (N + 1) * FLLRef = Fdco
                                            // (366 + 1) * 32768 = 12MHz
                                            // Set FLL Div = fDCOCLK/2
  __bic_SR_register(SCG0);                  // Enable the FLL control loop
  //UCSCTL0 |= DCO0 + DCO1 + DCO2  + DCO3;
  //UCSCTL0 |= DCO4 + DCO0;
  // Worst-case settling time for the DCO when the DCO range bits have been
  // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
  // UG for optimization.
  // 32 x 32 x 12 MHz / 32,768 Hz = 375000 = MCLK cycles for DCO to settle
  __delay_cycles(1375000);
	
  // Loop until XT1,XT2 & DCO fault flag is cleared
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
                                            // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
 
 */

///
/// set dco features
///
void setDCO(unsigned long int mhz){
	unsigned long int risultato;
	/// set almost high value in V_CORE
 	///risultato = SetVCore(PMMCOREV_2);
 	/// set highest value of V_CORE
 	if (mhz < 2000000)
 		risultato = SetVCore(PMMCOREV_0);
 	else if (mhz < 10000000)
 		risultato = SetVCore(PMMCOREV_2);
 		else 
 		 	risultato = SetVCore(PMMCOREV_3);
 		 	
 	risultato =  mhz / 32768;
 	
 	UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
 	UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
  __bis_SR_register(SCG0);                  // Disable the FLL control loop
  UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
  
  //UCSCTL1 = DCORSEL_2;                      // Select DCO range 0.3MHz - 7MHz operation
  /// set DCO range: 2.5 - 54 MHz
  UCSCTL1 = DCORSEL_5;
  
  //UCSCTL2 = FLLD_0 + 127;		            		// Set DCO Multiplier for 4.1943MHz
                                            // (127 + 1) * FLLRef = Fdco
                                            // (127 + 1) * 32768 = 4.1943MHz
                                            // Set FLL Div = fDCOCLK/1
  //UCSCTL2 = FLLD_0 + 511;		            		// Set DCO Multiplier for 16.777216MHz
                                            // (511 + 1) * FLLRef = Fdco
                                            // (511 + 1) * 32768 = 16.777216MHz
                                            // Set FLL Div = fDCOCLK/1
  UCSCTL2 = FLLD_0 + risultato - 1;      		// Set DCO Multiplier for 29.491200MHz
                                            // (899 + 1) * FLLRef = Fdco
                                            // (899 + 1) * 32768 = 29.491200MHz
                                            // Set FLL Div = fDCOCLK/1
  __bic_SR_register(SCG0);                  // Enable the FLL control loop
 	__delay_cycles(1375000);
  // Loop until XT1,XT2 & DCO fault flag is cleared
  
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
                                            // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
  }while (SFRIFG1&OFIFG);      

	/// fine dell'impostazione del clock.
}
