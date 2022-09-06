/************************************************************************/
/*																		*/
/*	config.h -- Project Configuration Declarations                      */
/*																		*/
/************************************************************************/
/*	Author: Michael T. Alexander										*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This header contains declarations used to configure the project     */
/*  based on the target platform.                                       */
/*																		*/
/************************************************************************/
/*  Revision History:						                			*/
/*											                        	*/
/*	05/21/2009 (MichaelA): created			                			*/
/*											                        	*/
/************************************************************************/

#if !defined(_CONFIG_INC)
#define _CONFIG_INC


/* ------------------------------------------------------------ */
/*					     Pin Definitions   		    			*/
/* ------------------------------------------------------------ */

/*	Onboard LEDs
*/
#define	trisLed1			TRISB
#define	trisLed1Set			TRISBSET
#define	trisLed1Clr			TRISBCLR
#define	prtLed1				PORTB
#define	prtLed1Set			PORTBSET
#define	prtLed1Clr			PORTBCLR
#define	prtLed1Inv			PORTBINV
#define	bnLed1				10

#define	trisLed2			TRISB
#define	trisLed2Set			TRISBSET
#define	trisLed2Clr			TRISBCLR
#define	prtLed2				PORTB
#define	prtLed2Set			PORTBSET
#define	prtLed2Clr			PORTBCLR
#define	prtLed2Inv			PORTBINV
#define	bnLed2				11

#define	trisLed3			TRISB
#define	trisLed3Set			TRISBSET
#define	trisLed3Clr			TRISBCLR
#define	prtLed3				PORTB
#define	prtLed3Set			PORTBSET
#define	prtLed3Clr			PORTBCLR
#define	prtLed3Inv			PORTBINV
#define	bnLed3				12

#define	trisLed4			TRISB
#define	trisLed4Set			TRISBSET
#define	trisLed4Clr			TRISBCLR
#define	prtLed4				PORTB
#define	prtLed4Set			PORTBSET
#define	prtLed4Clr			PORTBCLR
#define	prtLed4Inv			PORTBINV
#define	bnLed4				13

/*	Onboard Buttons
*/

#define	trisBtn1			TRISA
#define	trisBtn1Set			TRISASET
#define	trisBtn1Clr			TRISACLR
#define	prtBtn1				PORTA
#define	prtBtn1Set			PORTASET
#define	prtBtn1Clr			PORTACLR
#define	bnBtn1				6

#define	trisBtn2			TRISA
#define	trisBtn2Set			TRISASET
#define	trisBtn2Clr			TRISACLR
#define	prtBtn2				PORTA
#define	prtBtn2Set			PORTASET
#define	prtBtn2Clr			PORTACLR
#define	bnBtn2				7


/*JE 1-4 pins (PmodBTN)
*/

#define	trisJE1			TRISD
#define	trisJE1Set		TRISDSET
#define	trisJE1Clr		TRISDCLR
#define	prtJE1			PORTD
#define	prtJE1Set		PORTDSET
#define	prtJE1Clr		PORTDCLR
#define	bnJE1			14

#define	trisJE2			TRISD
#define	trisJE2Set		TRISDSET
#define	trisJE2Clr		TRISDCLR
#define	prtJE2			PORTD
#define	prtJE2Set		PORTDSET
#define	prtJE2Clr		PORTDCLR
#define	bnJE2			15

#define	trisJE3			TRISF
#define	trisJE3Set		TRISFSET
#define	trisJE3Clr		TRISFCLR
#define	prtJE3			PORTF
#define	prtJE3Set		PORTFSET
#define	prtJE3Clr		PORTFCLR
#define	bnJE3			2

#define	trisJE4			TRISF
#define	trisJE4Set		TRISFSET
#define	trisJE4Clr		TRISFCLR
#define	prtJE4			PORTF
#define	prtJE4Set		PORTFSET
#define	prtJE4Clr		PORTFCLR
#define	bnJE4			8

/*JA 1-4 pins (PmodSWT)
*/
#define	trisJA1			TRISE
#define	trisJA1Set		TRISESET
#define	trisJA1Clr		TRISECLR
#define	prtJA1			PORTE
#define	prtJA1Set		PORTESET
#define	prtJA1Clr		PORTECLR
#define	swtJA1			0

#define	trisJA2			TRISE
#define	trisJA2Set		TRISESET
#define	trisJA2Clr		TRISECLR
#define	prtJA2			PORTE
#define	prtJA2Set		PORTESET
#define	prtJA2Clr		PORTECLR
#define	swtJA2			1

#define	trisJA3			TRISE
#define	trisJA3Set		TRISESET
#define	trisJA3Clr		TRISECLR
#define	prtJA3			PORTE
#define	prtJA3Set		PORTESET
#define	prtJA3Clr		PORTECLR
#define	swtJA3			2

#define	trisJA4			TRISE
#define	trisJA4Set		TRISESET
#define	trisJA4Clr		TRISECLR
#define	prtJA4			PORTE
#define	prtJA4Set		PORTESET
#define	prtJA4Clr		PORTECLR
#define	swtJA4			3


/*	Left motor
*/
#define	trisMtrLeftEnSet	TRISDSET
#define	trisMtrLeftEnClr	TRISDCLR
#define	prtMtrLeftEnSet		PORTDSET
#define	prtMtrLeftEnClr		PORTDCLR
#define	bnMtrLeftEn			1

#define	trisMtrLeftDirSet	TRISDSET
#define	trisMtrLeftDirClr	TRISDCLR
#define	prtMtrLeftDirSet	PORTDSET
#define	prtMtrLeftDirClr	PORTDCLR
#define	bnMtrLeftDir		7

/*	Right motor
*/
#define trisMtrRightEnSet	TRISDSET
#define	trisMtrRightEnClr	TRISDCLR
#define prtMtrRightEnSet	PORTDSET
#define prtMtrRightEnClr	PORTDCLR
#define	bnMtrRightEn		2

#define trisMtrRightDirSet	TRISDSET
#define	trisMtrRightDirClr	TRISDCLR
#define prtMtrRightDirSet	PORTDSET
#define prtMtrRightDirClr	PORTDCLR
#define	bnMtrRightDir		6


/* ------------------------------------------------------------ */
/*					Miscellaneous Declarations					*/
/* ------------------------------------------------------------ */

#define	OPT_HWSPI	2		//use SPI2 controller for SPI interface

/* ------------------------------------------------------------ */
/*					General Type Declarations					*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*					Object Class Declarations					*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */

#endif
