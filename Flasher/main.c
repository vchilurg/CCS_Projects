/*
 *   Created on: Jul 10, 2018
 *      Author: CGV
 */
#include <msp430.h> 
#include<LowLevelFunc.h>
#include<SBW430FR.h>
#include<stdio.h>


void main(void)
{
	runProgramm();
}

void runProgramm(void)
{

    //printf("test");

/*------------------------------------------------------------------------------------------------------*/
/*  1. | Initialize host MSP430 (on Replicator board) & target board                                    */
/*------------------------------------------------------------------------------------------------------*/

    InitController();                     // Initialize the host MSP430FR6989

    //ShowStatus(STATUS_ACTIVE, 0);         // Switch both LEDs on to indicate operation.

    InitTarget();                         // Initialize target board

}
