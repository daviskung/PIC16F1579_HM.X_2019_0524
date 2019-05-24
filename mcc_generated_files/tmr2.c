/**
  TMR2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr2.c

  @Summary
    This is the generated driver implementation file for the TMR2 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides APIs for TMR2.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65.2
        Device            :  PIC16F1579
        Driver Version    :  2.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.45
        MPLAB 	          :  MPLAB X 4.15
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "tmr2.h"
#include "HM_HeartRate.h"
#include "pin_manager.h"



uint8_t NSTROBE_PWM_cnt;
uint8_t NSTROBE_LOW_EndSet; 
uint8_t NSTROBE_Rset;

bit bNDISCH_GoLow;


/**
  Section: Global Variables Definitions
*/

void (*TMR2_InterruptHandler)(void);

/**
  Section: TMR2 APIs
*/

void TMR2_Initialize(void)
{
    // Set TMR2 to the options selected in the User Interface

    // PR2 82; 
    PR2 = 0x52;

    // TMR2 0; 
    TMR2 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR2IF = 0;

    // Enabling TMR2 interrupt.
    PIE1bits.TMR2IE = 1;

    // Set Default Interrupt Handler
    TMR2_SetInterruptHandler(TMR2_InterruptHandler_davis);

    // T2CKPS 1:4; T2OUTPS 1:3; TMR2ON on; 
    T2CON = 0x15;
}

void TMR2_StartTimer(void)
{
    // Start the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 1;
}

void TMR2_StopTimer(void)
{
    // Stop the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 0;
}

uint8_t TMR2_ReadTimer(void)
{
    uint8_t readVal;

    readVal = TMR2;

    return readVal;
}

void TMR2_WriteTimer(uint8_t timerVal)
{
    // Write to the Timer2 register
    TMR2 = timerVal;
}

void TMR2_LoadPeriodRegister(uint8_t periodVal)
{
   PR2 = periodVal;
}

void TMR2_ISR(void)
{

    // clear the TMR2 interrupt flag
    PIR1bits.TMR2IF = 0;

    if(TMR2_InterruptHandler)
    {
        TMR2_InterruptHandler();
    }
}


void TMR2_SetInterruptHandler(void (* InterruptHandler)(void)){
    TMR2_InterruptHandler = InterruptHandler;
}

void TMR2_DefaultInterruptHandler(void){
    // add your TMR2 interrupt custom code
    // or set custom function using TMR2_SetInterruptHandler()
}

void TMR2_InterruptHandler_davis(void)
{
	uint8_t i;
	
	if( NSTROBE_PWM_cnt == 0 ){ 
		
		/*Configure NDISCH_PIN GPIO input mode */
		NDISCH_portSet = INPUT;
		
		NS1_PIN = LOW;	
		if(NSTROBE_Rset == 1)	NS2_PIN = LOW;	
		else if(NSTROBE_Rset == 2){
			NS2_PIN = LOW; NS3_PIN = LOW;
		}
		else if(NSTROBE_Rset == 3){
			NS2_PIN = LOW; NS3_PIN = LOW; NS4_PIN = LOW;
		}
				
	}
	else if( NSTROBE_PWM_cnt == NSTROBE_LOW_EndSet ){ 
		All_NS_pin = 0xff;

		SAMP_PIN = HIGH;
		for (i = 0; i < 6; ++i);	//delay 18 us
		SAMP_PIN = LOW;

		/*Configure NDISCH_PIN GPIO output mode */
		NDISCH_portSet = OUTPUT;
				
		NDISCH_PIN = LOW;
		
	}

	NSTROBE_PWM_cnt = NSTROBE_PWM_cnt + 1;
	if(NSTROBE_PWM_cnt > 9) NSTROBE_PWM_cnt=0;
	
	
}

/**
  End of File
*/
