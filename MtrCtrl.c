/************************************************************************/
/*																		*/
/*	MtrCtrl.c	--  Motor Control Routines				                */
/*																		*/
/************************************************************************/
/*	Author: Michael T. Alexander										*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
/*																		*/
/*  This module contains declarations and definitions for controlling   */
/*  the robot motors using a combination of pulse width modulation and  */
/*  pulse width timing of the feedback signals provided by the DC 		*/
/*	motors.																*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*  06/03/2009 (MichaelA): created                                      */
/*																		*/
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include "config.h"
#include "stdtypes.h"
#include "MtrCtrl.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

const		HWORD	rgtusMtr[] = { 1, 10, 50, 100, 500, 1000, 2000 };
const		HWORD	rgdtcMtr[] = { 0,  1,  5,  10,  20,   30,   40 };

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void    UpdateMotors();  //loads new values into OCRs

/* ------------------------------------------------------------ */
/*				Interrupt Service Routines	            	    */
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/***	UpdateMotors
**
**	Synopsis:
**		UpdateMotors()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Updates Output Compare Registers with values to drive motors
**		as well as updating motor direction bits.
*/

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

void    UpdateMotors(){  //loads new values into OCRs
	
	OC3R 	= dtcMtrStopped;   // this block 
	OC3RS	= dtcMtrStopped;
	OC2R	= dtcMtrStopped;
	OC2RS	= dtcMtrStopped;

	if(dirMtrLeft){
		trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
		prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward
	}else if(!dirMtrLeft){
		trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
		prtMtrLeftDirSet	= ( 1 << bnMtrLeftDir );	// backward
	}

	if(!dirMtrRight){
		trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
		prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward
	}else if(dirMtrRight){
		trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
		prtMtrRightDirClr	= ( 1 << bnMtrRightDir );	// backward
	}		

	OC3R 	= dtcMtrLeft;   
	OC3RS	= dtcMtrLeft;
	OC2R	= dtcMtrRight;
	OC2RS	= dtcMtrRight;
}


/*************************************************************************************/
