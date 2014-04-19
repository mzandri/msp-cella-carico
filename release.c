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
 * 				set up highest V_CORE to run CPU up to 30MHz            *
 *        fck = 24.192000MHz, UART0 and 1 = 115200b/s and TIMER A *
 *        equals to 100 ms.																				*/

#include "init.h"

const unsigned char releaseSW[8] = "1.0.7.1";
const unsigned char releaseHW[8] = "1.0.1.B";

