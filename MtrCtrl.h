/************************************************************************/
/*																		*/
/*	MtrCtrl.h	--  Motor Control Routines				                */
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
/*	12/29/2009 (LeviB):	   altered to perform preprogrammed movements	*/
/*																		*/
/************************************************************************/

#if !defined(_MTRCTRL_INC)
#define _MTRCTRL_INC

#include "config.h"
#include "stdtypes.h"

/* ------------------------------------------------------------ */
/*					General Type Declarations					*/
/* ------------------------------------------------------------ */

/*	Motor duty cycles.
*/
#define dtcMtrMin		50
#define dtcMtrMax		1950

/*
#define	dtcMtrSlow		8000
#define	dtcMtrMedium	4000
#define	dtcMtrFast		2500
#define	dtcMtrStopped	0
*/

#define dtcMtr15        1500
#define	dtcMtrSlow		2500
#define	dtcMtrMedium	4000
#define	dtcMtrFast		8000
#define dtcMtr100       10000
#define	dtcMtrStopped	0

/*	Motor feedback timing.
*/
#define	tusMtrFbSlow	8000
#define	tusMtrFbMedium	4000
#define	tusMtrFbFast	2500   

/*	Motor direction.
*/
#define	dirMtrLeftFwd	0
#define	dirMtrLeftBwd	1
#define	dirMtrRightFwd	1
#define	dirMtrRightBwd	0

/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */

/*	Motor control variables.
*/
volatile	BOOL	fMtrLeft;
volatile	BOOL	fMtrRight;
			HWORD	dtcMtrLeft;
			HWORD	dtcMtrRight;
			BYTE	dirMtrLeft;
			BYTE	dirMtrRight;

/* ------------------------------------------------------------ */
/*					        Macros		        				*/
/* ------------------------------------------------------------ */

#define MtrCtrlFwd()		dirMtrLeft			= dirMtrLeftFwd;\
							dirMtrRight			= dirMtrRightFwd;\
							dtcMtrLeft			= dtcMtrFast;\
							dtcMtrRight			= dtcMtrFast
							
#define MtrCtrlFwdLeft()	dirMtrLeft			= dirMtrLeftFwd;\
							dirMtrRight			= dirMtrRightFwd;\
							dtcMtrLeft			= dtcMtrMedium;\
							dtcMtrRight			= dtcMtrFast
						
#define MtrCtrlFwdRight()	dirMtrLeft			= dirMtrLeftFwd;\
							dirMtrRight			= dirMtrRightFwd;\
							dtcMtrLeft			= dtcMtrFast;\
							dtcMtrRight			= dtcMtrMedium
							
#define MtrCtrlBwd()		dirMtrLeft			= dirMtrLeftBwd;\
							dirMtrRight			= dirMtrRightBwd;\
							dtcMtrLeft			= dtcMtrFast;\
							dtcMtrRight			= dtcMtrFast
							
#define MtrCtrlBwdLeft()	dirMtrLeft			= dirMtrLeftBwd;\
							dirMtrRight			= dirMtrRightBwd;\
							dtcMtrLeft			= dtcMtrMedium;\
							dtcMtrRight			= dtcMtrFast
						
#define MtrCtrlBwdRight()	dirMtrLeft			= dirMtrLeftBwd;\
							dirMtrRight			= dirMtrRightBwd;\
							dtcMtrLeft			= dtcMtrFast;\
							dtcMtrRight			= dtcMtrMedium
							
#define MtrCtrlLeft()		dirMtrLeft			= dirMtrLeftBwd;\
							dirMtrRight			= dirMtrRightFwd;\
							dtcMtrLeft			= dtcMtrFast;\
							dtcMtrRight			= dtcMtrFast

#define MtrCtrlRight()		dirMtrLeft			= dirMtrLeftFwd;\
							dirMtrRight			= dirMtrRightBwd;\
							dtcMtrLeft			= dtcMtrFast;\
							dtcMtrRight			= dtcMtrFast
							
#define	MtrCtrlStop()		dtcMtrLeft			= dtcMtrStopped;\
							dtcMtrRight			= dtcMtrStopped

/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

void UpdateMotors();

/* ------------------------------------------------------------ */

#endif

/************************************************************************/
