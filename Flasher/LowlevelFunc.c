/*
 * LowlevelFunc.c
 *
 *  Created on: Jul 10, 2018
 *      Author: CGV
 */
//----------------------------------------------------------------------------
//! \brief Initialization of the Controller Board
#include<LowLevelFunc.h>
#include<stdio.h>

void InitController(void)
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

    // Disable the GPIO power-on default high-impedance mode to activate
   // previously configured port settings
          PM5CTL0 &= ~LOCKLPM5;

          JTAGOUT = 0;
          JTAGDIR = TCK + TDIO;


}

//----------------------------------------------------------------------------
//! \brief Delay function (resolution is 1 ms)
//! \param milliseconds (number of ms, max number is 0xFFFF)
void MsDelay(word milliseconds)
{
   word i;
   for(i = milliseconds; i > 0; i--)
   {
        TA0CCTL0 &= ~CCIFG;             // Clear the interrupt flag
        TA0CTL |= TACLR+MC_1;           // Clear & start timer
        while ((TA0CCTL0 & CCIFG)==0);  // Wait until the Timer elapses
        TA0CTL &= ~MC_1;    // Stop Timer
   }
}

//----------------------------------------------------------------------------
//! \brief Delay function (resolution is ~1 us)
//! \param microseconds (number of ms, max number is 0xFFFF)
//Clock freq for source code processor
//NOP takes one cycle
void usDelay(word microseconds)
{
    do
    {
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
    }
    while (--microseconds > 0);
}

//----------------------------------------------------------------------------
//! \brief Set the direction for the TDIO pin
//! \param dir (0 = IN, !0 = OUT)
void TDOI_dir(word dir)
{
    // Always set to input in the F5437
    if( dir == 0 )
        JTAGDIR |= TDIO;    //Input to target From host (Host Output)
    else
        JTAGDIR &= ~TDIO;   //Input from target to host (Host Input)
}

//----------------------------------------------------------------------------
//! \brief Set up I/O pins for JTAG communication
void DrvSignals(void)
{
    JTAGSEL  = 0x00; //Select GPIO pins
    TDOI_dir( 0 );   //Host Output
    JTAGOUT = TDIO + TCK;

}

void RlsSignals(void)
{
    JTAGOUT = 0;
}

void InitTarget(void)
{
    DrvSignals();
   //RlsSignals();
}
