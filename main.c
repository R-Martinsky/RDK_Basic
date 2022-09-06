/************************************************************************/
/*																		*/
/*	main.c	--	Main program module for project							*/
/*																		*/
/************************************************************************/
/*	Author: 	Dion Moses												*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This program is a reference design for the Digilent	Basic			*/
/*	Robotic Development Kit (RDK-Basic) with the Cerebot 32MX4 			*/
/*	Microcontroller board.  It uses two timers to drive two motors 		*/
/*	with output compare modules.										*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 12/09/09(DionM): created											*/
/*   12/29/09(LeviB): altered to add movement functions and PmodBtn and */
/*					  PmodSwt functionality								*/
/*	 12/08/10(AaronO): renamed to RDK_Basic								*/	
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include "stdtypes.h"
#include "config.h"
#include "MtrCtrl.h"
#include "spi.h"
#include "util.h"
#include "math.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

#define		TCKPS22 			6
#define 	TCKPS21				5
#define 	TCKPS20				4

#define		TCKPS32 			6
#define 	TCKPS31				5
#define 	TCKPS30				4
/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */
#ifndef OVERRIDE_CONFIG_BITS

#pragma config ICESEL   = ICS_PGx2		// ICE/ICD Comm Channel Select
#pragma config BWP      = OFF			// Boot Flash Write Protect
#pragma config CP       = OFF			// Code Protect
#pragma config FNOSC    = PRIPLL		// Oscillator Selection
#pragma config FSOSCEN  = OFF			// Secondary Oscillator Enable
#pragma config IESO     = OFF			// Internal/External Switch-over
#pragma config POSCMOD  = HS			// Primary Oscillator
#pragma config OSCIOFNC = OFF			// CLKO Enable
#pragma config FPBDIV   = DIV_8			// Peripheral Clock divisor
#pragma config FCKSM    = CSDCMD		// Clock Switching & Fail Safe Clock Monitor
#pragma config WDTPS    = PS1			// Watchdog Timer Postscale
#pragma config FWDTEN   = OFF			// Watchdog Timer 
#pragma config FPLLIDIV = DIV_2			// PLL Input Divider
#pragma config FPLLMUL  = MUL_16		// PLL Multiplier
#pragma config UPLLIDIV = DIV_2			// USB PLL Input Divider
#pragma config UPLLEN   = OFF			// USB PLL Enabled
#pragma config FPLLODIV = DIV_1			// PLL Output Divider
#pragma config PWP      = OFF			// Program Flash Write Protect
#pragma config DEBUG    = OFF			// Debugger Enable/Disable
    
#endif

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

#define	stPressed	1
#define	stReleased	0

#define	cstMaxCnt	10 // number of consecutive reads required for
					  

#define MAX        100      
#define PULSE_PER_REV       158      
#define Wheel_DC            4000  
#define int_error_Max_Min          10000      
#define MAX_DC              10000     
#define MIN_DC              0        

struct btn {
	BYTE	stBtn;	// status of the button (pressed or released)
	BYTE	stCur;  // current read state of the button
	BYTE	stPrev; // previous read state of the button
	BYTE	cst;	// number of consecutive reads of the same button 
					// state
};

//PmodCLS instructions
static	char szClearScreen[] = { 0x1B, '[', 'j', 0};

static	char szCursorOff[] = { 0x1B, '[', '0', 'c', 0 };
static	char szBacklightOn[]     = { 0x1B, '[', '3', 'e', 0 };

static	char szScrollLeft[] = {0x1B, '[', '1', '@', 0}; 
static	char szScrollRight[] = {0x1B, '[', '1', 'A', 0}; 
static	char szWrapMode[] = {0x1B, '[', '0', 'h', 0}; 

static	char szCursorPos[] = {0x1B, '[', '1', ';', '0', 'H', 0}; 
static	char szNextLineCursorPos[] = {0x1B, '[', '8', ';', '0', 'H', 0}; 
/* ------------------------------------------------------------ */
/*				Global Variables				                */
/* ------------------------------------------------------------ */

volatile	struct btn	btnBtn1;
volatile	struct btn	btnBtn2;

volatile	struct btn	PmodBtn1;
volatile	struct btn	PmodBtn2;
volatile	struct btn	PmodBtn3;
volatile	struct btn	PmodBtn4;

volatile	struct btn	PmodSwt1;
volatile	struct btn	PmodSwt2;
volatile	struct btn	PmodSwt3;
volatile	struct btn	PmodSwt4;

uint16_t ArrIC2[MAX]; 
int IC2Counter = 0;

uint16_t ArrIC3[MAX]; 
int IC3Counter = 0;
//EDIT: need global index 

uint32_t ic2Counter = 0;
uint32_t ic2CounterExtra = 0;           
uint32_t ic3Counter = 0;

float targetSpeed = 5000; // Max is 1.82 Min is 0
float out;
float speed;
uint16_t ArrADC1[100];
uint16_t ArrADC0[100];

float Distance2;
float Distance1;
float Distance0;

float ADCValue0;
float ADCValue1;
float ADCValue2;

int initSpeed = 3000;
int count = 0;
int count2 = 0;

int mode = 0;

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void	AppInit(void);
void	Wait_ms(WORD ms);

/* ------------------------------------------------------------ */
/*				Interrupt Service Routines						*/
/* ------------------------------------------------------------ */
/***	Timer5Handler
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
**		Interrupt service routine for Timer 5 interrupt. Timer 5
**		is used to perform software debouncing of the on-board
**		buttons. It is also used as a time base for updating
**		the on-board LEDs and the Pmod8LD LEDs at a regular interval.
*/
void __ISR(_ADC_VECTOR, ipl3) _ADC_IntHandler(void) 
{

//   ADC CONVERSION
//   ISR is written assuming that ADC channels are scanned and the interrupt is thrown after all the conversions are complete
//

	prtLed3Set = (1 << bnLed3);   		// turn LED3 on in the beginning of interrupt
	IFS1CLR = ( 1 << 1 );  			// clear interrupt flag for ADC1 Convert Done

//  Read the a/d buffers and convert to voltages
	ADCValue0 = (float)ADC1BUF0*3.3/1023.0;	// Reading AN0(zero), pin 1 of connector JJ -- servo sensor (center)
	ADCValue1 = (float)ADC1BUF1*3.3/1023.0;
    ADCValue2 = (float)ADC1BUF2*3.3/1023.0;
    
    Distance0 = 25.868*pow(ADCValue0, -1.059); // Converts from voltage to distance with equation found in excel
    Distance1 = 25.868*pow(ADCValue1, -1.059);
    Distance2 = 25.868*pow(ADCValue2, -1.059);
    
    
   // if (mode == 0){
        if (Distance0 > 15 && Distance1 > 15 && Distance0 < 25 && Distance1 < 25){ //GOOOOOOOOOOO
             OC3R = 3000;
             OC3RS = 3000;
             OC2R = 3000;
             OC2RS = 3000; 
        }

        if (Distance0 < 10 && Distance1 < 10){ //Go away from wall 
             OC3R =  1500;
             OC3RS = 1500;
             OC2R = 2500;
             OC2RS = 2500; 
        }

        if (Distance0 > 30 && Distance1 > 30){ //Go towards wall 
             OC3R = 2500;
             OC3RS = 2500;
             OC2R = 1500;
             OC2RS = 1500; 
        }

        else if (Distance0 > 25 && Distance1 > 25){
             OC3R = 2500;
             OC3RS = 2500;
             OC2R = 1500;
             OC2RS = 1500;
        }

        if (Distance0 < 10 && Distance1 > 15){ // Go Right
             OC3R = 3500;
             OC3RS = 3500;
             OC2R = 1500; 
             OC2RS = 1500;
        }  
  //  }
    
//    if (Distance0 < 10 && Distance1 > 15){ 
  //     OC3R = 5500;
  //     OC3RS = 5500;
  //     OC2R = 1000; 
  //     OC2RS = 1000;
  //  }  
    
  //  if (Distance0 < 10 && Distance1 < 10){ // 
   //     OC3R = 000;
  //      OC3RS = 000;
   //     OC2R = 000; 
    //    OC2RS = 000;
 //   }  
  /*            ///NEW
 
    if (Distance0 > 30 && Distance1 > 30){
        OC2R = initSpeed + 300;
        OC2RS = initSpeed + 300;
        OC3R = initSpeed;
        OC3RS = initSpeed;
    }
    else if (Distance1 < 10){ // get away from wall
        OC2R = initSpeed + 300;
        OC2RS = initSpeed + 300;
        OC3R = initSpeed;
        OC3RS = initSpeed;
    }
    else if (Distance0 >= 11 && Distance0 < 30){ // sweet spot in-between
        OC2R = initSpeed;
        OC2RS = initSpeed;
        OC3R = initSpeed;
        OC3RS = initSpeed;
    }
    else if (Distance0 > 30){ // far from wall go closer
        OC2R = initSpeed;
        OC2RS = initSpeed;
        OC3R = initSpeed + 300;
        OC3RS = initSpeed + 300;
    }
    else if (Distance0 < 10){ // get away from wall
        OC2R = initSpeed + 300;
        OC2RS = initSpeed + 300;
        OC3R = initSpeed;
        OC3RS = initSpeed;
    }
*/
   
//        if (Distance0 > 10 && Distance1 > 10 && Distance0 < 18 && Distance1 < 18 ){
//        OC3R = 4000;
//        OC3RS = 4000;
//         OC2R = 4000;
//         OC2RS = 4000; 
//    }
//        else if (Distance0 < 12 && Distance1 > 18){ // Go Right
//        OC3R = 4000;
//        OC3RS = 4000;
//        OC2R = 2000;
//        OC2RS = 2000;
//        }
//       
//    else if (Distance0 > 12 && Distance1 < 18){ // Go Left
//        OC3R = 4000;
//        OC3RS = 4000;
//        OC2R = 2000;
//        OC2RS = 2000;
//      }
//    else if (Distance0 > 12 && Distance1 > 18){ // soft right
//        OC3R = 4000;
//        OC3RS = 4000;
//        OC2R = 3000;
//        OC2RS = 3000;
//        }
//    else if (Distance0 < 18 && Distance1 < 12){ // soft left
//        OC3R = 3000;
//        OC3RS = 3000;
//        OC2R = 4000;
//        OC2RS = 4000;
//      }
    //              NEW
  //  if (Distance0 < 14){ // soft left  

	prtLed3Clr = (1 << bnLed3);   // turn LED3 off at the end of interrupt
}

void __ISR(_TIMER_5_VECTOR, ipl7) Timer5Handler(void)
{    
	static	WORD tusLeds = 0;
	
	mT5ClearIntFlag();
	prtLed1Set = (1<<bnLed1); 
    

    static float prev_err = 0;
    static float ei = 0;
    
    float kp = 1;//1;
    float ki = 5;//0.5;
    float kd = 1;//0.01;
    
    if (IC2Counter > 1)
    {
        float curr = ArrIC2[IC2Counter-1];
        float prev = ArrIC3[IC2Counter-2];
        
        float time = curr - prev;
        
        if (time < 0)
            time += 65000;
       
        // Grab most recent value
        // and convert to secs
        // The value is orig mirco seconds
        float secs = time * 0.000001;
        // using time find speed
        // X pulse * 1 rev / 137 pulse * 0.75 ft / 1 rev = 0.00547445255*X ft
        // speed = dist/time => ft/s
        speed = 0.00632911/secs;
        
        // find err between curr and target
        float err = targetSpeed - speed; 
        
        // (dx) find diff between curr err and prev err
        float ed = err - prev_err;
        
        // (intx) sum err over time
        ei += ki * err;
        
        // bound ei
        if (ei > int_error_Max_Min)
            ei = int_error_Max_Min;
        else if (ei < -int_error_Max_Min)
            ei = -int_error_Max_Min ;
        
        // calculate pid 
        out = err * kp + ei + ed * kd;
         
        // store curr err in prev err
        prev_err = err;
        
        
        if (out > MAX_DC)
            out = MAX_DC;
        else if (out < MIN_DC)
            out = MIN_DC;
        
        
     //  OC2R = (uint32_t) out;
       // OC2RS = (uint32_t) out;
        
       //OC3R = (uint32_t) out;
       //OC3RS = (uint32_t) out;
        
    }
    
    
    
    
    
        
    
    
	// Read the raw state of the button pins.
	btnBtn1.stCur = ( prtBtn1 & ( 1 << bnBtn1 ) ) ? stPressed : stReleased;
	btnBtn2.stCur = ( prtBtn2 & ( 1 << bnBtn2 ) ) ? stPressed : stReleased;
	
	//Read the raw state of the PmodBTN pins
	PmodBtn1.stCur = ( prtJE1 & ( 1 << bnJE1 ) ) ? stPressed : stReleased;
	PmodBtn2.stCur = ( prtJE2 & ( 1 << bnJE2 ) ) ? stPressed : stReleased;
	PmodBtn3.stCur = ( prtJE3 & ( 1 << bnJE3 ) ) ? stPressed : stReleased;
	PmodBtn4.stCur = ( prtJE4 & ( 1 << bnJE4 ) ) ? stPressed : stReleased;

	//Read the raw state of the PmodSWT pins
	PmodSwt1.stCur = ( prtJA1 & ( 1 << swtJA1 ) ) ? stPressed : stReleased;
	PmodSwt2.stCur = ( prtJA2 & ( 1 << swtJA2 ) ) ? stPressed : stReleased;
	PmodSwt3.stCur = ( prtJA3 & ( 1 << swtJA3 ) ) ? stPressed : stReleased;
	PmodSwt4.stCur = ( prtJA4 & ( 1 << swtJA4 ) ) ? stPressed : stReleased;

	// Update state counts.
	btnBtn1.cst = ( btnBtn1.stCur == btnBtn1.stPrev ) ? btnBtn1.cst + 1 : 0;
	btnBtn2.cst = ( btnBtn2.stCur == btnBtn2.stPrev ) ? btnBtn2.cst + 1 : 0;

	//Update state counts for PmodBTN
	PmodBtn1.cst = (PmodBtn1.stCur == PmodBtn1.stPrev) ? PmodBtn1.cst +1 : 0;
	PmodBtn2.cst = (PmodBtn2.stCur == PmodBtn2.stPrev) ? PmodBtn2.cst +1 : 0;
	PmodBtn3.cst = (PmodBtn3.stCur == PmodBtn3.stPrev) ? PmodBtn3.cst +1 : 0;
	PmodBtn4.cst = (PmodBtn4.stCur == PmodBtn4.stPrev) ? PmodBtn4.cst +1 : 0;

	//Update state counts for PmodSWT
	PmodSwt1.cst = (PmodSwt1.stCur == PmodSwt1.stPrev) ? PmodSwt1.cst +1 : 0;
	PmodSwt2.cst = (PmodSwt2.stCur == PmodSwt2.stPrev) ? PmodSwt2.cst +1 : 0;
	PmodSwt3.cst = (PmodSwt3.stCur == PmodSwt3.stPrev) ? PmodSwt3.cst +1 : 0;
	PmodSwt4.cst = (PmodSwt4.stCur == PmodSwt4.stPrev) ? PmodSwt4.cst +1 : 0;
	
	// Save the current state.
	btnBtn1.stPrev = btnBtn1.stCur;
	btnBtn2.stPrev = btnBtn2.stCur;

	// Save the current state for PmodBTN
	PmodBtn1.stPrev = PmodBtn1.stCur;
	PmodBtn2.stPrev = PmodBtn2.stCur;
	PmodBtn3.stPrev = PmodBtn3.stCur;
	PmodBtn4.stPrev = PmodBtn4.stCur;

	// Save the current state for PmodSWT
	PmodSwt1.stPrev = PmodSwt1.stCur;
	PmodSwt2.stPrev = PmodSwt2.stCur;
	PmodSwt3.stPrev = PmodSwt3.stCur;
	PmodSwt4.stPrev = PmodSwt4.stCur;
	
	// Update the state of button 1 if necessary.
	if ( cstMaxCnt == btnBtn1.cst ) {
		btnBtn1.stBtn = btnBtn1.stCur;
		btnBtn1.cst = 0;
	}
	
	// Update the state of button 2 if necessary.
	if ( cstMaxCnt == btnBtn2.cst ) {
		btnBtn2.stBtn = btnBtn2.stCur;
		btnBtn2.cst = 0;
	}

	//if statements for buttons

	// Update the state of PmodBTN1 if necessary.
	if ( cstMaxCnt == PmodBtn1.cst ) {
		PmodBtn1.stBtn = PmodBtn1.stCur;
		PmodBtn1.cst = 0;
	}
	
	// Update the state of PmodBTN2 if necessary.
	if ( cstMaxCnt == PmodBtn2.cst ) {
		PmodBtn2.stBtn = PmodBtn2.stCur;
		PmodBtn2.cst = 0;
        
	}

	// Update the state of PmodBTN3 if necessary.
	if ( cstMaxCnt == PmodBtn3.cst ) {
		PmodBtn3.stBtn = PmodBtn3.stCur;
		PmodBtn3.cst = 0;
        
	}

	// Update the state of PmodBTN4 if necessary.
	if ( cstMaxCnt == PmodBtn4.cst ) {
		PmodBtn4.stBtn = PmodBtn4.stCur;
		PmodBtn4.cst = 0;
	}

	//if statements for switches

	// Update the state of PmodSWT1 if necessary.
	if ( cstMaxCnt == PmodSwt1.cst ) {
		PmodSwt1.stBtn = PmodSwt1.stCur;
		PmodSwt1.cst = 0;
	}
	
	// Update the state of PmodSWT2 if necessary.
	if ( cstMaxCnt == PmodSwt2.cst ) {
		PmodSwt2.stBtn = PmodSwt2.stCur;
		PmodSwt2.cst = 0;
	}

	// Update the state of PmodSWT3 if necessary.
	if ( cstMaxCnt == PmodSwt3.cst ) {
		PmodSwt3.stBtn = PmodSwt3.stCur;
		PmodSwt3.cst = 0;
	}

	// Update the state of PmodSWT4 if necessary.
	if ( cstMaxCnt == PmodSwt4.cst ) {
		PmodSwt4.stBtn = PmodSwt4.stCur;
		PmodSwt4.cst = 0;
	}
       
    prtLed1Clr	= ( 1 << bnLed1 );
}



/***	Input Capture Interupts
**      
**	Parameters:
**		none
**
*/
void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl5) _IC2_IntHandler(void) 
{
    
    mIC2ClearIntFlag();
    ic2Counter++;
    
    prtLed2Set	= ( 1 << bnLed2 );
    uint16_t buffer_data;
    while(IC2CON & (0x0008)){
        //ic2CounterExtra++;        compare to ic2Counter outside of while loop. Checks if 
        buffer_data = (uint16_t) IC2BUF;
    }

    ArrIC2[IC2Counter] = buffer_data;

    IC2Counter++;
    IC2Counter %= MAX;

    prtLed2Clr	= ( 1 << bnLed2 );
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl5) _IC3_IntHandler(void) 
{
    mIC3ClearIntFlag();  
    ic3Counter++;
    
    prtLed2Set	= ( 1 << bnLed2 );
    uint16_t buffer_data;
    while(IC3CON & (0x0008)){
	 buffer_data = (uint16_t) IC3BUF;
	 
    }

    ArrIC3[IC3Counter] = buffer_data;

    IC3Counter++;
    IC3Counter %= MAX;
    
    // clear interrupt flag for Input Capture 3
    // increment counter
    
    prtLed2Clr	= ( 1 << bnLed2 );
}

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	main
**
**	Synopsis:
**		st = main()
**
**	Parameters:
**		none
**
**	Return Values:
**		does not return
**
**	Errors:
**		none
**
**	Description:
**		Main program module. Performs basic board initialization
**		and then enters the main program loop.
*/


int main(void) {

	BYTE	stBtn1;
	BYTE	stBtn2;

	BYTE	stPmodBtn1;
	BYTE	stPmodBtn2;
	BYTE	stPmodBtn3;
	BYTE	stPmodBtn4;

	BYTE	stPmodSwt1;
	BYTE	stPmodSwt2;
	BYTE	stPmodSwt3;
	BYTE	stPmodSwt4;

    OC2R = initSpeed;
    OC2RS = initSpeed;
    OC3R = initSpeed;
    OC3RS = initSpeed;
	
    
    
    DeviceInit();
	AppInit();
    InitLeds(); 

	//INTDisableInterrupts();
	DelayMs(500);

	//write to PmodCLS
	//SpiEnable();
//	SpiPutBuff(szClearScreen, 3);
//	DelayMs(4);
//	SpiPutBuff(szBacklightOn, 4);
//	DelayMs(4);
//	SpiPutBuff(szCursorOff, 4);
//	DelayMs(4);
//	SpiPutBuff("Hello from", 10);
//	DelayMs(4);
//	SpiPutBuff(szCursorPos, 6);
//	DelayMs(4);
//	SpiPutBuff("Digilent!", 9);
//	DelayMs(2000);
    
    char strout[100];
	//SpiDisable();

	INTEnableInterrupts();
	while (fTrue)
	{	
        
		//INTDisableInterrupts();
        
        SpiEnable();
        SpiPutBuff(szClearScreen, 3);
        DelayMs(4);
        SpiPutBuff(szBacklightOn, 4);
        DelayMs(4);
        SpiPutBuff(szCursorOff, 4);
        DelayMs(4);
        //sprintf(strout, "PID=%.2f", out);
       // sprintf(strout, "ADC0=%.2f", ADCValue0);
        sprintf(strout, "D0=%.2f D2=%.2f", Distance0, Distance2);
        SpiPutBuff(strout, strlen(strout));
        DelayMs(4);
        SpiPutBuff(szCursorPos, 6);
        DelayMs(4);
        //sprintf(strout, "TS=%.2f CS=%.2f", targetSpeed, speed);
        //sprintf(strout, "ADC1=%.2f", ADCValue1);
        sprintf(strout, "D1=%.2f", Distance1);
        SpiPutBuff(strout, strlen(strout));
        DelayMs(250);
        SpiDisable();        
		//get data here
		stBtn1 = btnBtn1.stBtn;
		stBtn2 = btnBtn2.stBtn;

		stPmodBtn1 = PmodBtn1.stBtn;
		stPmodBtn2 = PmodBtn2.stBtn;
		stPmodBtn3 = PmodBtn3.stBtn;
		stPmodBtn4 = PmodBtn4.stBtn;

		stPmodSwt1 = PmodSwt1.stBtn;
		stPmodSwt2 = PmodSwt2.stBtn;
		stPmodSwt3 = PmodSwt3.stBtn;
		stPmodSwt4 = PmodSwt4.stBtn;

        
        
        
		//INTEnableInterrupts();
		//configure OCR to go forward

//		if(stPressed == stPmodBtn1){
//			//start motor if button 2 pressed
//
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00);
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			
//		}else if(stPressed == stPmodBtn2){
//			//start left turn
//
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00);
//			MtrCtrlBwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			
//		}else if(stPressed == stPmodBtn3){
//			//start right turn
//
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00);
//			MtrCtrlRight();
//			UpdateMotors();
//			Wait_ms(0x0200);
//			MtrCtrlStop();
//			UpdateMotors();
//
//		} else if(stPressed == stPmodBtn4){
//			//start move backward
//
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00);
//			MtrCtrlLeft();
//			UpdateMotors();
//			Wait_ms(0x0200);
//			MtrCtrlStop();
//			UpdateMotors();
//
//		} else if(stPressed == stPmodSwt1){
//			//make square to right
//
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00);  //gives delay to toggle switch
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlRight();
//			UpdateMotors();		// first turn
//			Wait_ms(0x0180);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlRight();     // second turn
//			UpdateMotors();
//			Wait_ms(0x0180);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlRight();		// third turn
//			UpdateMotors();
//			Wait_ms(0x0180);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlRight();
//			UpdateMotors();
//			Wait_ms(0x0180);
//			MtrCtrlStop();
//			UpdateMotors();
//
//		} else if(stPressed == stPmodSwt2){
//			//make triangle to left
//
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00); //gives delay to toggle switch
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlLeft();  	//first turn
//			UpdateMotors();
//			Wait_ms(0x0280);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlLeft();		//second turn
//			UpdateMotors();
//			Wait_ms(0x0280);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlLeft();		//third turn
//			UpdateMotors();
//			Wait_ms(0x0280);
//			MtrCtrlStop();
//			UpdateMotors();
//		
//		}else if(stPressed == stPmodSwt3){
//			// Three point turn around
//
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00);  //gives delay to toggle switch
//			MtrCtrlFwdRight();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlBwdLeft();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0500);
//			MtrCtrlFwd();
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//
//		}else if(stPressed == stPmodSwt4){
//			// dance
//			
//			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
//			UpdateMotors(); 
//			Wait_ms(0x0A00);  //gives delay to toggle switch
//			MtrCtrlFwdLeft(); // step left
//			UpdateMotors();
//			Wait_ms(0x0300);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0200);
//			MtrCtrlFwdRight(); // step right
//			UpdateMotors();		
//			Wait_ms(0x0300);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0200);
//			MtrCtrlFwdLeft();  // step left
//			UpdateMotors();
//			Wait_ms(0x0300);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0200);
//			MtrCtrlFwdRight(); // step right
//			UpdateMotors();
//			Wait_ms(0x0200);
//			MtrCtrlStop();
//			UpdateMotors();
//			Wait_ms(0x0200);
//			MtrCtrlLeft();     // spin
//			UpdateMotors();
//			Wait_ms(0x0800);
//			MtrCtrlStop();
//			UpdateMotors();
//		}  //end if

	}  //end while
}  //end main

/* ------------------------------------------------------------ */
/***	DeviceInit
**
**	Synopsis:
**		DeviceInit()
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
**		Initializes on chip peripheral devices to the default
**		state.
*/

void DeviceInit() {
	// Configure left motor direction pin and set default direction.
	trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
	prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward
	
	// Configure right motor direction pin and set default direction.
	trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
	prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward

	// Configure Output Compare 2 to drive the left motor.
	OC2CON	= ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 );	// pwm set up
	//OC2R	= dtcMtrStopped; //original
	OC2R = Wheel_DC; // Wheel DC = 2000
    //OC2RS	= dtcMtrStopped;  //original
    OC2RS = Wheel_DC;// Wheel DC = 2000
    
	// Configure Output Compare 3.
	OC3CON = ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 );	// pwm
	//OC3R	= dtcMtrStopped;
	OC3R = Wheel_DC;// Wheel DC = 2000
    //OC3RS	= dtcMtrStopped;
    OC3RS = Wheel_DC;// Wheel DC = 2000
    
    //Input capture 2 for timer 2
    IPC2SET	= ( 1 << 12 ) | ( 0 << 11 ) | ( 1 << 10 ) | ( 1 << 9 ) | ( 1 << 8 ); // interrupt priority level 5, sub 3
	IFS0CLR = ( 1 << 9 );
	IEC0SET	= ( 1 << 9 );
    
    IC2CON = (1<<7) | (1<<1) | (1<<0);  //trigger from every edge to only rising edge

    
    //Input capture 3 for timer 2
    IPC3SET	= ( 1 << 12 ) | ( 0 << 11 ) | ( 1 << 10 ) | ( 1 << 9 ) | ( 1 << 8 ); // interrupt priority level 5, sub 3
	IFS0CLR = ( 1 << 13 );
	IEC0SET	= ( 1 << 13 );
    
    IC3CON = (1<<7) | (1<<1) | (1<<0);  //trigger from every edge to only rising edge
    
    // turn on Input Capture 2 & 3
    IC2CONSET = (1<<15);
    IC3CONSET = (1<<15);
    
    
    // Configure Timer 2.
	TMR2	= 0;									// clear timer 2 count
	PR2		= 64999;
    
	// Configure Timer 3.
	TMR3	= 0;
	PR3		= 9999;

	// Start timers and output compare units.
	T2CON		= ( 1 << 15 ) | ( 1 << TCKPS20 )|(1 << TCKPS21);		// timer 2 prescale = 8
	OC2CONSET	= ( 1 << 15 );	// enable output compare module 2
	OC3CONSET	= ( 1 << 15 );	// enable output compare module 3
	T3CON		= ( 1 << 15 ) | ( 1 << TCKPS31 ) | ( 1 << TCKPS30); 	// timer 3 prescale = 8

	// Configure Timer 5.
	TMR5	= 0;
	PR5		= 999; // period match every 1000 us
	IPC5SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 7, sub 3
	IFS0CLR = ( 1 << 20);
	IEC0SET	= ( 1 << 20);
    
   
	ADC_Init();
	// Start timers.
	T5CON = ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 ); // fTimer5 = fPb / 8
    
	//enable SPI
	SpiInit();
    

	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();
}

void ADC_Init(){
//	CONFIGURE ADC
	AD1PCFG	= 0XFFF8;
	// CONFIGURE AN0, AN1, and AN2 AS ANALOG INPUTS
	AD1CON1	= 0X00E4;   
	// BIT 7-5 SSRC 111 = INTERNAL COUNTER ENDS SAMPLING AND STARTS CONVERSION
	// BIT 4 CLRASM 0 = Normal Operation, buffer contents will be overwritten by the next conversion sequence
	// BIT 2 ASAM 1 = SAMPLING BEGINS immediately after conversion completes; SAMP bit is automatically set
	// BIT 1 SAMP 0 = ADC IS NOT SAMPLING
	// BUT 0 DONE 0 = STATUS BIT
	AD1CON2	= 0X0408; // 0000 0100 0000 1000     				
	// BIT 10 CSCNA 1 = SCAN INPUTS
	// BIT 2-3 SMPL 1-1 = ONE INTERRUPT AFTER EVERY THIRD CONVERSION
	AD1CON3	= 0X1FFF;
	// BIT 15 ADRC 0 = CLOCK DERIVED FROM PERIPHERAL BUS CLOCK
	// SAMC AND ADCS - I NEED TO READ MORE ABOUT TIMING TO UNDERSTAND THE FUNCTION OF THESE TWO VARIABLES
	AD1CHS	= 0X00000000;   // 32 bit SFR  
	AD1CSSL	= 0X0007;   // select channel 0 and 1 and 2 
	// CSSL = SCAN CHANNELS 1 and 0
	IPC6SET	= ( 1 << 27 ) | ( 1 << 26 ); // ADC interrupt priority level 3, sub 0
	IFS1CLR	= 2;    // CLEAR ADC INTERRUPT FLAG
	IEC1SET = 2;	// ENABLE ADC INTERRUPT
	AD1CON1SET = 0X8000;	// 	TURN ADC ON

}

/* ------------------------------------------------------------ */
/***	AppInit
**
**	Synopsis:
**		AppInit()
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
**		Performs application specific initialization. Sets devices
**		and global variables for application.
*/

void AppInit() {
        int a = 0;
  
    for (a = 0 ; a<100 ; a++)
   {
        ArrIC2[a]=0;
        ArrIC3[a]=0;
        //diffTime[a]=0;

    }
}


/* ------------------------------------------------------------ */
/***	Wait_ms
**
**	Synopsis:
**		Wait_ms(WORD)
**
**	Parameters:
**		WORD (range from 0 to 65535)
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Will wait for specified number of milliseconds.  Using a 
**		word variable allows for delays up to 65.535 seconds.  The value
**		in the for statement may need to be altered depending on how your
**		compiler translates this code into AVR assembly.  In assembly, it is
**		possible to generate exact delay values by counting the number of clock
**		cycles your loop takes to execute.  When writing code in C, the compiler
**		interprets the code, and translates it into assembly.  This process is 
**		notoriously inefficient and may vary between different versions of AVR Studio
**		and WinAVR GCC.  A handy method of calibrating the delay loop is to write a 
**		short program that toggles an LED on and off once per second using this 
**		function and using a watch to time how long it is actually taking to
**		complete. 
**
*/

void Wait_ms(WORD delay) {

	WORD i;

	while(delay > 0){

		for( i = 0; i < 375; i ++){
			;;
		}
		delay -= 1;
	}
}

/************************************************************************/
void InitLeds( void ) 
{
	// Configure LEDs as digital outputs.
	trisLed1Clr = ( 1 << bnLed1 );
	trisLed2Clr = ( 1 << bnLed2 );
	trisLed3Clr = ( 1 << bnLed3 );
	trisLed4Clr = ( 1 << bnLed4 );
	
	// Turn off the LEDs.
	prtLed1Clr	= ( 1 << bnLed1 );
	prtLed2Clr	= ( 1 << bnLed2 );
	prtLed3Clr	= ( 1 << bnLed3 );
	prtLed4Clr	= ( 1 << bnLed4 );
}	