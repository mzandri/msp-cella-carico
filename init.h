#ifndef INIT_H_
#define INIT_H_

#include "gps.h"

typedef struct{
	unsigned char blocco;
	unsigned int address;
	unsigned int numPage;
	unsigned int readAddr;
	//unsigned char lastBlock;
	unsigned char readBlocco;
} __eepromData;

typedef struct {
	unsigned char stato;
	unsigned char speed;
} __MCUstatus;

//extern typedef struct GPSstatus ;

void setDCO(unsigned long int);
void init_MCUstatus(__MCUstatus *);
void init_GPSstatus(GPSstatus *, GPSmessStatus * );
void init_PORT(void);
void init_TIMER(void);
void init_ADC(unsigned char);
void init_ADC_int(unsigned char);
void init_UART0(unsigned long int);
void init_UART1(unsigned long int);
void setPort(unsigned char);
char PRINTF(char *);
void PRINTF0(char *st);
void PRINTFU(char);
void initEepromData(void);
void gpsOnOFF(char statoGPS);

void initI2C_B0(unsigned char);
void setBank(unsigned char block);
char readI2C_N_Byte( unsigned int address, unsigned char numElm, volatile unsigned char buff[] );
char writeI2C_N_Byte(unsigned int address, unsigned char block, unsigned char numElm, volatile unsigned char buff[] );
unsigned int leggiPagineScritte(void);
void clearFlash(unsigned char);

/// rel hw management
void readHWrelfromFlash(char []);
void writeHWrelInFlash(unsigned char []);

/// MCU command
void transmitAllFlash(void);
void directConnect(void);
void pollReply(void);
void getRelease(void);

unsigned int setupADC(void);

void replyToMessage(__MCUstatus *);

void copia1(char st1[], char st2[]);
void send_test_cmd(void);

/*********/
//#define		FDCO							29491200
#define		FDCO							24192000

/*********/
#define		NACK_ERR					0
#define   BUS_BUSY					255
#define 	OK								1
#define 	TRUE							1
#define 	FALSE							0

#define		DIM_BUFF					128
#define		DIM_SEC_BUFF			128
#define		DIM_UART_BUFF			32
#define   DIM_UART1_TX_BUFF	32
#define		DIM_GPS_CMD				64
#define		DIM_GPS_BUFF			128
#define   DIM_UART1_RX_BUFF	512


#define		FLASH_FULL				0xFFFF

#define		SYNC							0xAA
#define   DOWNLOAD					0x11
#define   POLL							0x10
#define   GET_RELEASE				0x80
#define   CLEAR_FLASH				0x18
#define   SET_FLASH					0x19
#define		SET_HW_REL        0x21
#define   DIRECT_CONNECT    0x14
#define		GPS_ON						0x31
#define		GPS_OFF						0x32

#define	  SET_SAMPLING			0x41

#define		CHECKSUM					0x5B


#define		TEMP_SENS					0x0A
#define   CELL_1						0x01

/** GPS DEFINE AND COMMANS **/
#define		HYPERTERMINAL			0x00
#define		EMB_SW						0xC0
#define 	TEST_GPS					0xC1
#define		SET_SAMP_FREQ			0xC2
#define		SET_MSG_TYPE			0xC4

#endif /*INIT_H_*/
