/*
 *   Created on: Jul 10, 2018
 *      Author: CGV
 */
/****************************************************************************/
/* INCLUDES                                                                 */
/****************************************************************************/

#include "JTAGfunc430FR.h"       // JTAG functions
#include "Config430FR.h"         // High level user configuration
#include "LowLevelFunc430Xv2.h"  // Low level functions

/****************************************************************************/
/* VARIABLES                                                                */
/****************************************************************************/

//! \brief This variable holds the start address of the main memory
unsigned long mainStartAdress = MAIN_START_ADDRESS;
//! \brief This variable holds the length of the main memory (in words)
unsigned long mainLength = MAIN_LENGTH;

/****************************************************************************/
/* FUNCTIONS                                                                */
/****************************************************************************/

//! Main function
void main(void)
{
    runProgramm();
}

//! \brief The basic Replicator routine
//! \details This function is executed once at startup and can be restarted by pressing button S1 on the REP430F board.
void runProgramm(void)
{

    //! \brief Array to store data for a memory write
    word WriteData[WordBufferSize];
    //! \brief Array to store data for a memory read
    word ReadData[WordBufferSize];

/*------------------------------------------------------------------------------------------------------*/
/*  1. | Initialize host MSP430 (on Replicator board) & target board                                    */
/*------------------------------------------------------------------------------------------------------*/

    InitController();                     // Initialize the host MSP430F5437

    ShowStatus(STATUS_ACTIVE, 0);         // Switch both LEDs on to indicate operation.

    InitTarget();                         // Initialize target board

/*------------------------------------------------------------------------------------------------------*/
/*  2. | Connect to the target device (provide JTAG password if needed)                                 */
/*------------------------------------------------------------------------------------------------------*/

    // Uncomment the following section if the JTAG password is set and should be removed - FR5xx/FR6xx
    {
        // enter password & length here:
        unsigned short Password[] = { 0x1111, 0x2222 };
        unsigned long  PasswordLength = 0x02; // password length in words

        //Unlock device with user password
        if(UnlockDevice_430Xv2(Password, PasswordLength) != STATUS_OK)
        {
            ShowStatus(STATUS_ERROR, 1);  // stop here password was wrong
        }
    }

    // Uncomment the following section if the JTAG password is set and should be removed - FR4xx/FR2xx
    /*if(GetDevice_430Xv2() == STATUS_FUSEBLOWN)
    {
        if (!EraseFRAMViaBootCode_430Xv2(MAIL_BOX_32BIT, STOP_DEVICE, USER_CODE_ERASE))
        {
            ShowStatus(STATUS_ERROR, 2);
        }
        // Check if main memory is completely erased.
        if (!EraseCheck_430Xv2(mainStartAdress, mainLength/2))
        {
            ShowStatus(STATUS_ERROR, 2);
        }
    }*/


    if (GetDevice_430Xv2() != STATUS_OK)         // Set DeviceId
    {
        ShowStatus(STATUS_ERROR, 1);      // Stop here if invalid JTAG ID or
    }
    // time-out. (error: red LED is ON)



/*------------------------------------------------------------------------------------------------------*/
/*  3. | Perform a erase + write + read + verify in the target RAM (optional)                           */
/*------------------------------------------------------------------------------------------------------*/

    // The following section is not required and included only as a reference on
    // how to access the target's RAM and alter its content
    // DisableMpu_430Xv2() must be called prior to this, if the device's MPU is enabled


    word i,j;
    // write dummy data to target RAM
    for(i = 0, j = 0; j < WordBufferSize; j++, i+=2)
    {
        ReadData[j]  = 0;
        WriteData[j] = j;
        WriteMem_430Xv2(F_WORD, MAIN_START_ADDRESS + i, j);
    }
    // read data from target RAM and verify
    for(i = 0, j = 0; j < WordBufferSize; j++, i+=2)
    {
        ReadData[j] = ReadMem_430Xv2(F_WORD, MAIN_START_ADDRESS + i);
        MsDelay(1);
        if(ReadData[j] != WriteData[j])
        {
            ShowStatus(STATUS_ERROR, 2);
        }
    }
    // verify content with PSA
    if(VerifyMem_430Xv2(MAIN_START_ADDRESS, WordBufferSize, WriteData) != STATUS_OK)
    {
        ShowStatus(STATUS_ERROR, 3);
    }


/*------------------------------------------------------------------------------------------------------*/
/*  4. | Operations in the device's main memory (disable MPU if necessary)                              */
/*------------------------------------------------------------------------------------------------------*/

    // The Memory Protection Unit (MPU) allows the user to set up individual access rights for up to
    // three user defined memory segments. For detailed information see the Memory Protection Unit
    // description in the device family user's guide.

    // Disable Memory Protection Unit
    if (DisableMpu_430Xv2() != STATUS_OK)
    {
        ShowStatus(STATUS_ERROR, 1);
    }

    // Not possible on FR5739 - use EraseFRAM_430Xv2 instead
    // Erase main FRAM memory using the JTAG mailbox and Bootcode
    if (!EraseFRAMViaBootCode_430Xv2(MAIL_BOX_32BIT, STOP_DEVICE, USER_CODE_ERASE))
    {
        ShowStatus(STATUS_ERROR, 2);
    }
    // Check if main memory is completely erased.
    if (!EraseCheck_430Xv2(mainStartAdress, mainLength/2))
    {
        ShowStatus(STATUS_ERROR, 2);
    }
    // Disable FRAM write protection - only FR5994 and FR6047 device family. Disabled by default
    /*if (DisableFramWprod_430Xv2() != STATUS_OK)
    {
        ShowStatus(STATUS_ERROR, 2);
    }*/
    // Since EraseCheck executes a BOR - The MPU gets enabled again - Disable it before code download
    if (DisableMpu_430Xv2() != STATUS_OK)
    {
        ShowStatus(STATUS_ERROR, 2);
    }
    //Program blinking LED target code
    if(DownloadMsp430Code() != STATUS_OK)
    {
        ShowStatus(STATUS_ERROR, 2);
    }

/*------------------------------------------------------------------------------------------------------*/
/*  5. | Program the JTAG lock key (optional)                                                           */
/*------------------------------------------------------------------------------------------------------*/

    // Remove following comments to enable Lock Key programming routine.
    // This makes the MSP430 device permanently inaccessible via JTAG

    /*if (DisableMpu_430Xv2() != STATUS_OK)
    {
        ShowStatus(STATUS_ERROR, 2);
    }
    if (!ProgramLockKey())        // ***Action is permanent***
    {
        ShowStatus(STATUS_ERROR, 15);
    }*/

}



/****************************************************************************/
/*                         END OF SOURCE FILE                               */
/****************************************************************************/
