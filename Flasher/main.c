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

    /*------------------------------------------------------------------------------------------------------*/
    /*  2. | Connect to the target device (provide JTAG password if needed)                                 */
    /*------------------------------------------------------------------------------------------------------*/

        // Uncomment the following section if the JTAG password is set and should be removed - FR5xx/FR6xx
        {
            // enter password & length here:
            uint16_t Password[] = { 0x1111, 0x2222 };
            uint32_t  PasswordLength = 0x02; // password length in words

            //Unlock device with user password
            if(UnlockDevice_430Xv2(Password, PasswordLength) != SC_ERR_NONE)
            {
                printf("Wrong Password\n");  // stop here password was wrong
            }
        }

        if (GetCoreID() != SC_ERR_NONE)         // Set DeviceId
            {
                printf("Invalid ID\n");      // Stop here if invalid JTAG ID or
            }

}
