/************************************************************************/
/*																		*/
/*	spi.h	--  SPI Interface Declarations                              */
/*																		*/
/************************************************************************/
/*	Author: Michael T. Alexander										*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
/*																		*/
/*  This header file contains interface declarations for SPI            */
/*  communication.                                                      */
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*  05/21/2009 (MichaelA): created                                      */
/*																		*/
/************************************************************************/

#if !defined(_SPI_INC)
#define _SPI_INC

#include "config.h"
#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*					Miscellaneous Declarations					*/
/* ------------------------------------------------------------ */

#if OPT_HWSPI == 1
	
	// M00TODO: implement this
	
#elif OPT_HWSPI == 2

	#define		trisSpiSsSet	TRISGSET
	#define		trisSpiSsClr	TRISGCLR
	#define		prtSpiSsSet		PORTGSET
	#define		prtSpiSsClr		PORTGCLR
	#define 	bnSpiSs			9

#else
	#error "undefined SPI hardware interface"
	
#endif

/* ------------------------------------------------------------ */
/*					Register Field Definitions					*/
/* ------------------------------------------------------------ */

/*	SPI2CON
*/
#define		bnSpi2On		15
#define		bnSpi2Smp		9
#define		bnSpi2Cke		8
#define		bnSpi2Msten		5

/*	SPI2STAT
*/
#define		bnSpi2Spibusy	11
#define		bnSpi2Spitbe	3
#define		bnSpi2Spirbf	0

/* ------------------------------------------------------------ */
/*					General Type Declarations					*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

void    SpiInit();
void    SpiEnable();
void    SpiDisable();
BYTE    BSpiPutByte( BYTE bSnd );
void    SpiPutBuff( BYTE* pbBuff, WORD cbBuff );
void    SpiGetBuff( BYTE bFill, BYTE* pbBuff, WORD cbBuff );
void    SpiPutGetBuff( BYTE* pbSnd, BYTE* pbRcv, WORD cbSnd );

/* ------------------------------------------------------------ */

#endif

/************************************************************************/
