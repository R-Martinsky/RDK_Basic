/************************************************************************/
/*																		*/
/*	spi.c	--  SPI Interface Definitions                               */
/*																		*/
/************************************************************************/
/*	Author: Michael T. Alexander										*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
/*																		*/
/*  This module contains interface declarations for SPI communication.  */
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*  05/21/2009 (MichaelA): created                                      */
/*																		*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include "config.h"
#include "spi.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	SpiInit
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initialize the SPI interface for use.
**
*/

void SpiInit()
{
	// Configure SS pin as digital output.
	trisSpiSsClr = ( 1 << bnSpiSs );
	
	// Default SS pin to disabled.
	prtSpiSsSet = ( 1 << bnSpiSs );
	
	/* Implementation using SPI2 hardware interface.
	*/
	
	//SPI2BRG = 1;	// Fsck = 2 MHz @ Fpb = 8 Mhz
	//SPI2BRG = 31;	// Fsck = 125 kHz @ Fpb = 8 Mhz
	SPI2BRG = 7;	// Fsck = 500 KHz assuming Fpb = 8 MHz
	SPI2CON = ( 1 << bnSpi2On ) | ( 1 << bnSpi2Smp ) | ( 1 << bnSpi2Cke ) | ( 1 << bnSpi2Msten );
}

/* ------------------------------------------------------------ */
/***	SpiEnable
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Enable SPI slave device.
*/

void SpiEnable()
{
	prtSpiSsClr = ( 1 << bnSpiSs );
}

/* ------------------------------------------------------------ */
/***	SpiDisable
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Disable SPI slave device.
*/

void SpiDisable()
{
	prtSpiSsSet = ( 1 << bnSpiSs );
}

/* ------------------------------------------------------------ */
/***	BSpiPutByte
**
**	Parameters:
**		bSnd - byte to send
**
**	Return Value:
**		byte received from slave
**
**	Errors:
**		none
**
**	Description:
**		Send a byte to a SPI slave and receive a byte from the
**      slave.
*/

BYTE BSpiPutByte( BYTE bSnd )
{


	SPI2BUF = bSnd;	// write transmit data to buffer
	
	// Wait for receive buffer to become filled.
	while ( ! (SPI2STAT & ( 1 << bnSpi2Spirbf )) ) {
		asm volatile("nop");
	}
	
	return SPI2BUF;


}

/* ------------------------------------------------------------ */
/***	SpiPutBuff
**
**	Parameters:
**		pbBuff - pointer to a buffer of bytes to send
**      cbBuff - number of bytes in buffer
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Send a buffer of bytes to an SPI slave. Discard any 
**      data received from the slave.
*/

void SpiPutBuff( BYTE* pbBuff, WORD cbBuff )
{
    while ( 0 < cbBuff ) {
        BSpiPutByte(*pbBuff);
        pbBuff++;
        cbBuff--;
    }
}

/* ------------------------------------------------------------ */
/***	SpiGetBuff
**
**	Parameters:
**      bFill  - a filler byte sent to the slave
**		pbBuff - pointer to a buffer of bytes to receive
**      cbBuff - number of bytes to receive
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Receive a specified number of bytes from SPI slave.
**      The bFill byte is shifted to the slave on every transmission.
*/

void SpiGetBuff( BYTE bFill, BYTE* pbBuff, WORD cbBuff )
{
    while ( 0 < cbBuff ) {
        *pbBuff = BSpiPutByte(bFill);
        pbBuff++;
        cbBuff--;
    }
}

/* ------------------------------------------------------------ */
/***	SpiPutGetBuff
**
**	Parameters:
**      pbSnd  - a buffer of bytes to send
**		pbRcv  - a buffer of bytes to receive slave data
**      cbSnd  - number of bytes to be transferred
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Transfer a specified number of bytes between master and
**      slave.
*/

void SpiPutGetBuff( BYTE* pbSnd, BYTE* pbRcv, WORD cbSnd )
{
    while ( 0 < cbSnd ) {
        *pbRcv = BSpiPutByte(*pbSnd);
        pbRcv++;
        pbSnd++;
        cbSnd--;
    }
}

/************************************************************************/
