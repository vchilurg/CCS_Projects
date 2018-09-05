/*
 * SBW430FR.c
 *
 *  Created on: Jul 27, 2018
 *      Author: CGV
 */
#include<LowLevelFunc.h>
#include<SBW430FR.h>
#include<stdio.h>

//******************************************************************************
// Global Variables SWB Options ************************************************
//******************************************************************************
#define FR4xx_LOCKREGISTER 0x160
#define SAFE_FRAM_PC 0x0004
uint16_t SegmentInfoAKey5xx = 0xA548;
uint8_t tdo_bit;               //holds the value of TDO-bit
uint8_t TCLK_saved;  // holds the last value of TCLK before entering a JTAG sequence
uint16_t JtagId = 0;
uint16_t CoreId = 0;
uint16_t DeviceId = 0;
uint32_t DeviceIdPointer = 0;

/****************************************************************************/
/* FUNCTIONS                                                                */
/****************************************************************************/

//  combinations of sbw-cycles (TMS, TDI, TDO)
void TMSH_TDIH(void)
{
    TMSH
    TDIH
    TDOsbw
}

void TMSL_TDIH(void)
{
    TMSL
    TDIH
    TDOsbw
}

void TMSL_TDIH_TDOrd(void)
{
    TMSL  TDIH  TDO_RD
}

void TMSL_TDIL_TDOrd(void)
{
    TMSL  TDIL  TDO_RD
}

void TMSH_TDIH_TDOrd(void)
{
    TMSH  TDIH  TDO_RD
}

void TMSH_TDIL_TDOrd(void)
{
    TMSH  TDIL  TDO_RD
}

void TMSL_TDIL(void)
{
    TMSL  TDIL  TDOsbw
}

void TMSH_TDIL(void)
{
    TMSH  TDIL  TDOsbw
}

void ClrTCLK(void)
{
    if (TCLK_saved)
    {
        TMSLDH
    }
    else
    {
        TMSL
    }

    ClrSBWTDIO();

    TDIL TDOsbw    //ExitTCLK
    TCLK_saved = 0;
}

void SetTCLK(void)
{
   if (TCLK_saved)
   {
        TMSLDH
   }
   else
   {
        TMSL
   }

   SetSBWTDIO();

   TDIH TDOsbw    //ExitTCLK
   TCLK_saved = 1;
}

//*****************************************************************************
//
// Reset SBW TAP controller
//
//*****************************************************************************
void ResetTAP(void)
{
    uint32_t i;
    // Now fuse is checked, Reset JTAG FSM
    for (i = 6; i > 0; i--)      // 6 is nominal
    {
        TMSH_TDIH();
    }
    // JTAG FSM is now in Test-Logic-Reset
    TMSL_TDIH();                 // now in Run/Test Idle
}

//*****************************************************************************
//
// Shift bits
//
//*****************************************************************************
uint32_t AllShifts(uint16_t Format, uint32_t Data)
{
   uint32_t TDOword = 0x00000000;
   uint32_t MSB = 0x00000000;
   uint32_t i;

   switch(Format)
   {
   case F_BYTE: MSB = 0x00000080;
     break;
   case F_WORD: MSB = 0x00008000;
     break;
   case F_ADDR: MSB = 0x00080000;
     break;
   case F_LONG: MSB = 0x80000000;
     break;
   default: // this is an unsupported format, function will just return 0
     return TDOword;
   }
   // shift in bits
   for (i = Format; i > 0; i--)
   {
        if (i == 1)                     // last bit requires TMS=1; TDO one bit before TDI
        {
          ((Data & MSB) == 0) ? TMSH_TDIL_TDOrd() : TMSH_TDIH_TDOrd();
        }
        else
        {
          ((Data & MSB) == 0) ? TMSL_TDIL_TDOrd() : TMSL_TDIH_TDOrd();
        }
        Data <<= 1;
        if (tdo_bit)
            TDOword++;
        if (i > 1)
            TDOword <<= 1;               // TDO could be any port pin
   }
   TMSH_TDIH();                         // update IR
   if (TCLK_saved)
   {
        TMSL_TDIH();
   }
   else
   {
        TMSL_TDIL();
   }

   // de-scramble bits on a 20bit shift
   if(Format == F_ADDR)
   {
     TDOword = ((TDOword << 16) + (TDOword >> 4)) & 0x000FFFFF;
   }

   return(TDOword);
}

//*****************************************************************************
//
// IR scan
//
//*****************************************************************************
uint32_t IR_Shift(uint8_t instruction)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSH_TDIH();

    // JTAG FSM state = Select IR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-IR
    TMSL_TDIH();
    // JTAG FSM state = Shift-IR, Shift in TDI (8-bit)
    return(AllShifts(F_BYTE, instruction));
    // JTAG FSM state = Run-Test/Idle
}

//*****************************************************************************
//
// 16 bit DR scan
//
//*****************************************************************************
uint16_t DR_Shift16(uint16_t data)
{

    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-DR
    TMSL_TDIH();

    // JTAG FSM state = Shift-DR, Shift in TDI (16-bit)
    return(AllShifts(F_WORD, data));
    // JTAG FSM state = Run-Test/Idle
}

//*****************************************************************************
//
// 20 bit DR scan
//
//*****************************************************************************
uint32_t DR_Shift20(uint32_t address)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-DR
    TMSL_TDIH();

    // JTAG FSM state = Shift-DR, Shift in TDI (16-bit)
    return(AllShifts(F_ADDR, address));
    // JTAG FSM state = Run-Test/Idle
}

//*****************************************************************************
//
//! \brief Function to execute a Power-On Reset (POR) using JTAG CNTRL SIG
//! register
//! \return uint16_t (SC_ERR_NONE if target is in Full-Emulation-State afterwards,
//! SC_ERR_GENERIC otherwise)
//*****************************************************************************

uint16_t ExecutePOR_430Xv2(void)
{
    // provide one clock cycle to empty the pipe
    ClrTCLK();
    SetTCLK();

    // prepare access to the JTAG CNTRL SIG register
    IR_Shift(IR_CNTRL_SIG_16BIT);
    // release CPUSUSP signal and apply POR signal
    DR_Shift16(0x0C01);
    // release POR signal again
    DR_Shift16(0x0401);


    // Set PC to 'safe' memory location
    IR_Shift(IR_DATA_16BIT);
    ClrTCLK();
    SetTCLK();
    ClrTCLK();
    SetTCLK();
    DR_Shift16(SAFE_FRAM_PC);
    // PC is set to 0x4 - MAB value can be 0x6 or 0x8

    // drive safe address into PC
    ClrTCLK();
    SetTCLK();

    IR_Shift(IR_DATA_CAPTURE);

    // two more to release CPU internal POR delay signals
    ClrTCLK();
    SetTCLK();
    ClrTCLK();
    SetTCLK();

    // now set CPUSUSP signal again
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x0501);
    // and provide one more clock
    ClrTCLK();
    SetTCLK();
    // the CPU is now in 'Full-Emulation-State'

    // disable Watchdog Timer on target device now by setting the HOLD signal
    // in the WDT_CNTRL register
    uint16_t id =  IR_Shift(IR_CNTRL_SIG_CAPTURE);
    if(id == JTAG_ID98)
    {
        WriteMem_430Xv2(F_WORD, 0x01CC, 0x5A80);
    }
    else
    {
        WriteMem_430Xv2(F_WORD, 0x015C, 0x5A80);
    }

    // Initialize Test Memory with default values to ensure consistency
    // between PC value and MAB (MAB is +2 after sync)
    if(id == JTAG_ID91 || id == JTAG_ID99)
    {
        WriteMem_430Xv2(F_WORD, 0x06, 0x3FFF);
        WriteMem_430Xv2(F_WORD, 0x08, 0x3FFF);
    }

    // Check if device is in Full-Emulation-State again and return status
    IR_Shift(IR_CNTRL_SIG_CAPTURE);
    if(DR_Shift16(0) & 0x0301)
    {
        return(SC_ERR_NONE);
    }

    return(SC_ERR_GENERIC);
}

//*****************************************************************************
//
//! \brief Load a given address into the target CPU's program counter (PC).
//! \param[in] uint32_t Addr (destination address)
//
//*****************************************************************************
void SetPC_430Xv2(uint32_t Addr)
{
    uint16_t Mova;
    uint16_t Pc_l;

    Mova  = 0x0080;
    Mova += (uint16_t)((Addr>>8) & 0x00000F00);
    Pc_l  = (uint16_t)((Addr & 0xFFFF));

    // Check Full-Emulation-State at the beginning
    IR_Shift(IR_CNTRL_SIG_CAPTURE);
    if(DR_Shift16(0) & 0x0301)
    {
        // MOVA #imm20, PC
        ClrTCLK();
        // take over bus control during clock LOW phase
        IR_Shift(IR_DATA_16BIT);
        SetTCLK();
        DR_Shift16(Mova);
        // insert on 24.03.2010 Florian
        ClrTCLK();
        // above is just for delay
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x1400);
        IR_Shift(IR_DATA_16BIT);
        ClrTCLK();
        SetTCLK();
        DR_Shift16(Pc_l);
        ClrTCLK();
        SetTCLK();
        DR_Shift16(0x4303);
        ClrTCLK();
        IR_Shift(IR_ADDR_CAPTURE);
        DR_Shift20(0x00000);
    }
}

//*****************************************************************************
//
//! \brief Read a 32bit value from the JTAG mailbox.
//! \return uint32_t (32bit value from JTAG mailbox)
//
//*****************************************************************************
uint32_t i_ReadJmbOut(void)
{
    uint16_t sJMBINCTL;
    uint32_t  lJMBOUT = 0;
    uint16_t sJMBOUT0, sJMBOUT1;

    sJMBINCTL = 0;

    IR_Shift(IR_JMB_EXCHANGE);// start exchange
    lJMBOUT = DR_Shift16(sJMBINCTL);

    if(lJMBOUT & OUT1RDY)// check if new data available
    {
        sJMBINCTL |= JMB32B + OUTREQ;
        lJMBOUT  = DR_Shift16(sJMBINCTL);
        sJMBOUT0 = (uint16_t)DR_Shift16(0);
        sJMBOUT1 = (uint16_t)DR_Shift16(0);
        lJMBOUT = ((uint32_t)sJMBOUT1<<16) + sJMBOUT0;
    }
    return lJMBOUT;
}

//*****************************************************************************
//
//! \brief Write a 16bit value into the JTAG mailbox system.
//! The function timeouts if the mailbox is not empty after a certain number
//! of retries.
//! \param[in] uint16_t dataX (data to be shifted into mailbox)
//
//*****************************************************************************
int16_t i_WriteJmbIn16(uint16_t dataX)
{
    uint16_t sJMBINCTL;
    uint16_t sJMBIN0;
    uint32_t Timeout = 0;
    sJMBIN0 = (uint16_t)(dataX & 0x0000FFFF);
    sJMBINCTL = INREQ;

    IR_Shift(IR_JMB_EXCHANGE);
    do
    {
        Timeout++;
        if(Timeout >= 3000)
        {
            return SC_ERR_GENERIC;
        }
    }
    while(!(DR_Shift16(0x0000) & IN0RDY) && Timeout < 3000);
    if(Timeout < 3000)
    {
        DR_Shift16(sJMBINCTL);
        DR_Shift16(sJMBIN0);
    }
    return SC_ERR_NONE;
}

//*****************************************************************************
//
//! \brief Write a 32bit value into the JTAG mailbox system.
//! The function timeouts if the mailbox is not empty after a certain number
//! of retries.
//! \param[in] uint16_t dataX (data to be shifted into mailbox)
//! \param[in] uint16_t dataY (data to be shifted into mailbox)
//
//*****************************************************************************
int16_t i_WriteJmbIn32(uint16_t dataX,uint16_t dataY)
{
    uint16_t sJMBINCTL;
    uint16_t sJMBIN0,sJMBIN1;
    uint32_t Timeout = 0;

    sJMBIN0 = (uint16_t)(dataX & 0x0000FFFF);
    sJMBIN1 = (uint16_t)(dataY & 0x0000FFFF);
    sJMBINCTL =  JMB32B | INREQ;

    IR_Shift(IR_JMB_EXCHANGE);
    do
    {
        Timeout++;
        if(Timeout >= 3000)
        {
            return SC_ERR_GENERIC;
        }
    }
    while(!(DR_Shift16(0x0000) & IN0RDY) && Timeout < 3000);

    if(Timeout < 3000)
    {
        sJMBINCTL = 0x11;
        DR_Shift16(sJMBINCTL) ;
        DR_Shift16(sJMBIN0);
        DR_Shift16(sJMBIN1);
    }
    return SC_ERR_NONE;
}


//*****************************************************************************
//
// Connect the JTAG/SBW Signals and execute delay
//
//*****************************************************************************
void ConnectJTAG()
{
    // drive JTAG/TEST signals
    DrvSignals();
    MsDelay(15);             // delay 15ms
}
//*****************************************************************************
//
//! \brief Function to start the JTAG communication - RST line high - device starts
//! code execution
//
//*****************************************************************************
void EntrySequences_RstHigh_SBW()
{
    ClrTST();
    usDelay(800);              // delay min 800us - clr SBW controller
    SetTST();
    usDelay(50);

    // SpyBiWire entry sequence
    // Reset Test logic
    ClrSBWTDIO();                   // put device in normal operation: Reset = 0
    ClrSBWTCK();                    // TEST pin = 0
    MsDelay(1);                     // wait 1ms (minimum: 100us)

    // SpyBiWire entry sequence
    SetSBWTDIO();                   // Reset = 1
    SetSBWTCK();                    // TEST pin = 1
                                    // initial 1 SBWCLKs to enter sbw-mode
    ClrSBWTCK();
    SetSBWTCK();
}

//*****************************************************************************
//
//! \brief Function to start the SBW communication - RST line low - device do not
//! start code execution
//
//*****************************************************************************
void EntrySequences_RstLow_SBW()
{
    ClrTST();
    ClrRST();                  //Added for Low RST
    usDelay(800);              // delay min 800us - clr SBW controller
    SetTST();
    usDelay(50);

    // SpyBiWire entry sequence
    // Reset Test logic
    ClrSBWTDIO();                   // put device in normal operation: Reset = 0
    ClrSBWTCK();                    // TEST pin = 0
    MsDelay(1);                     // wait 1ms (minimum: 100us)

    // SpyBiWire entry sequence
    SetSBWTDIO();                   // Reset = 1
    SetSBWTCK();                    // TEST pin = 1
                                    // initial 1 SBWCLKs to enter sbw-mode
    ClrSBWTCK();
    SetSBWTCK();
}

//*****************************************************************************
//
// Stop JTAG/SBW by disabling the pins and executing delay
//
//*****************************************************************************
void StopJtag (void)
{
    // release JTAG/TEST signals
    RlsSignals();
    MsDelay(15);             // delay 15ms
}

//*****************************************************************************
//
//! \brief Function to enable JTAG communication with a target. Use JSBW mode
//!  if device is in LPM5 mode.
//! \return word (JTAG_ID91(0x91) if connection was established successfully,
//! invalid JTAG ID (0x1) otherwise)
//
//*****************************************************************************
uint16_t magicPattern(void)
{
    uint16_t deviceJtagID = 0;

    // Enable the JTAG interface to the device.
    ConnectJTAG();
    // Apply again 4wire/SBW entry Sequence.
    // set ResetPin = 0
    EntrySequences_RstLow_SBW();
    // reset TAP state machine -> Run-Test/Idle
    ResetTAP();
    // feed JTAG mailbox with magic pattern
    if(i_WriteJmbIn16(STOP_DEVICE) == SC_ERR_NONE)
    {
        // Apply again 4wire/SBW entry Sequence.

        EntrySequences_RstHigh_SBW();

        ResetTAP();  // reset TAP state machine -> Run-Test/Idle

        deviceJtagID = (uint16_t)IR_Shift(IR_CNTRL_SIG_CAPTURE);

        if(deviceJtagID == JTAG_ID91)
        {
            // if Device is in LPM.x5 -> reset IO lock of JTAG pins and Configure it for debug
            IR_Shift(IR_TEST_3V_REG);
            DR_Shift16(0x4020);
        }
        else if(deviceJtagID == JTAG_ID99)
        {
            IR_Shift(IR_TEST_3V_REG);
            DR_Shift16(0x40A0);
        }
        return deviceJtagID;
    }
    return 1;  // return 1 as an invalid JTAG ID
}

//*****************************************************************************
//
//! \brief This function unlocks the Fram memory when a JTAG password is set.
//! \param[in] unsigned short* password (Pointer to array containing the JTAG
//! Password)
//! \param[in] unsigned long passwordLength (length of the password in words)
//! \return word (STATUS_OK if memory unlock was successful, STATUS_ERROR
//! otherwise)
//
//*****************************************************************************
uint16_t UnlockDevice_430Xv2(uint16_t* password, uint32_t passwordLength)
{
  uint16_t i = 0;
    /*----------------------------------------------------------------------- */
    /*            phase 1 of device entry using a user password               */
    /*------------------------------------------------------------------------*/

    // Enable the JTAG interface to the device.
    ConnectJTAG();
    // Apply again 4wire/SBW entry Sequence.
    // set ResetPin =0

    EntrySequences_RstLow_SBW();
    // reset TAP state machine -> Run-Test/Idle
    ResetTAP();
    // shift in JTAG mailbox exchange request
    if(i_WriteJmbIn32(STOP_DEVICE, 0x1E1E) == SC_ERR_GENERIC)
    {
        return SC_ERR_GENERIC;
    }
    StopJtag();
    /*----------------------------------------------------------------------- */
    /*            phase 2 of device entry using a user password               */
    /*------------------------------------------------------------------------*/
    // Enable the JTAG interface to the device.
    ConnectJTAG();
    // Apply again 4wire/SBW entry Sequence.

    EntrySequences_RstHigh_SBW();

    // reset TAP state machine -> Run-Test/Idle
    ResetTAP();
    // shift in JTAG mailbox exchange request
    while(i < passwordLength)
    {
        if(i_WriteJmbIn16(password[i]) == SC_ERR_GENERIC)
        {
            return SC_ERR_GENERIC;
        }
    }
    return(SC_ERR_NONE);
}

//*****************************************************************************
//
//! \brief Function to determine & compare core identification info
//! \return uint16_t (SC_ERR_NONE if correct JTAG ID was returned, SC_ERR_GENERIC
//! otherwise)
//
//*****************************************************************************
uint16_t GetCoreID (void)
{
    uint16_t i;
    //uint16_t JtagId = 0;  //initialize JtagId with an invalid value
    for (i = 0; i < MAX_ENTRY_TRY; i++)
    {
        // release JTAG/TEST signals to safely reset the test logic
        StopJtag();
        // establish the physical connection to the JTAG interface
        ConnectJTAG();
        // Apply again 4wire/SBW entry Sequence.
        // set ResetPin =1

        EntrySequences_RstHigh_SBW();
        // reset TAP state machine -> Run-Test/Idle
        ResetTAP();
        // shift out JTAG ID
        JtagId = (uint16_t)IR_Shift(IR_CNTRL_SIG_CAPTURE);
        usDelay(500);
        // break if a valid JTAG ID is being returned
        if((JtagId == JTAG_ID91) || (JtagId == JTAG_ID99) || (JtagId == JTAG_ID98))                     //****************************
        {
            break;
        }
    }
    if(i >= MAX_ENTRY_TRY)
    {
    // if connected device is MSP4305438 JTAG Mailbox is not usable
#ifdef ACTIVATE_MAGIC_PATTERN
        for(i = 0; i < MAX_ENTRY_TRY; i++)
        {
            // if no JTAG ID is returns -> apply magic pattern to stop user cd excecution
            JtagId = magicPattern();
            if((JtagId == 1) || (i >= MAX_ENTRY_TRY))
            {
                // if magic pattern failed and 4 tries passed -> return status error
                return(SC_ERR_GENERIC);
            }
            else
            {
                break;
            }
        }
        // For MSP430F5438 family mailbox is not functional in reset state.
        // Because of this issue the magicPattern is not usable on MSP430F5438 family devices
#else
    return(SC_ERR_ET_DCDC_DEVID);
#endif
    }
    if((JtagId == JTAG_ID91) || (JtagId == JTAG_ID99) || (JtagId == JTAG_ID98))
    {
        return(SC_ERR_NONE);
    }
    else
    {
        return(SC_ERR_ET_DCDC_DEVID);
    }
}

//*****************************************************************************
//
//! \brief Function to determine & compare core identification info (Xv2)
//! \return word (STATUS_OK if correct JTAG ID was returned, STATUS_ERROR
//! otherwise)
//
//*****************************************************************************
uint16_t GetCoreipIdXv2()
{
    IR_Shift(IR_COREIP_ID);
    CoreId = DR_Shift16(0);
    if(CoreId == 0)
    {
        return(SC_ERR_GENERIC);
    }
    IR_Shift(IR_DEVICE_ID);
    DeviceIdPointer = DR_Shift20(0);
    // The ID pointer is an un-scrambled 20bit value
    return(SC_ERR_NONE);
}

//*****************************************************************************
//
//! \brief Function to resync the JTAG connection and execute a Power-On-Reset
//! \return uint16_t (SC_ERR_NONE if operation was successful, SC_ERR_GENERIC
//! otherwise)
//
//*****************************************************************************
uint16_t SyncJtag_AssertPor (void)
{
    uint16_t i = 0;

    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x1501);                  // Set device into JTAG mode + read

    if ((IR_Shift(IR_CNTRL_SIG_CAPTURE) != JTAG_ID91) &&
        (IR_Shift(IR_CNTRL_SIG_CAPTURE) != JTAG_ID99) &&
        (IR_Shift(IR_CNTRL_SIG_CAPTURE) != JTAG_ID98))
    {
      return(SC_ERR_GENERIC);
    }
    // wait for sync
    while(!(DR_Shift16(0) & 0x0200) && i < 50)
    {
        i++;
    };
    // continues if sync was successful
    if(i >= 50)
    {
        return(SC_ERR_GENERIC);
    }
    // execute a Power-On-Reset
    if(ExecutePOR_430Xv2() != SC_ERR_NONE)
    {
        return(SC_ERR_GENERIC);
    }

    return(SC_ERR_NONE);
}

//*****************************************************************************
//
//! \brief Function to take target device under JTAG control. Disables the
//! target watchdog. Sets the global DEVICE variable as read from the target
//! device.
//! \return uint16_t (SC_ERR_GENERIC if fuse is blown, incorrect JTAG ID or
//! synchronizing time-out; SC_ERR_NONE otherwise)
//
//*****************************************************************************
uint16_t GetDevice_430Xv2(void)
{
    if(GetCoreID () != SC_ERR_NONE)
    {
        return(SC_ERR_GENERIC);
    }
    if (IsLockKeyProgrammed() != SC_ERR_NONE)                 // Stop here if fuse is already blown
    {
        printf("Fuse Blown Error\r\n");
        return(STATUS_FUSEBLOWN);
    }
    if (GetCoreipIdXv2()!= SC_ERR_NONE)
    {
        printf("GetCoreipIdXv2 Error\r\n");
        return(SC_ERR_GENERIC);
    }
    if(SyncJtag_AssertPor() != SC_ERR_NONE)
    {
        printf("Sync JTAG Error\r\n");
        return(SC_ERR_GENERIC);
    }
    // CPU is now in Full-Emulation-State
    // read DeviceId from memory
    ReadMemQuick_430Xv2(DeviceIdPointer + 4, 1, (uint16_t*)&DeviceId);

    return(SC_ERR_NONE);
}

//*****************************************************************************
//
//! \brief This function checks if the JTAG lock key is programmed.
//! \return word (STATUS_OK if fuse is blown, STATUS_ERROR otherwise)
//
//*****************************************************************************
uint16_t IsLockKeyProgrammed(void)
{
    uint16_t i;

    for (i = 3; i > 0; i--)     //  First trial could be wrong
    {
        IR_Shift(IR_CNTRL_SIG_CAPTURE);
        if (DR_Shift16(0xAAAA) == 0x5555)
        {
            return(SC_ERR_GENERIC);  // Fuse is blown
        }
    }
    return(SC_ERR_NONE);       // Fuse is not blown
}

//*****************************************************************************
//
//! \brief This function writes one byte/uint16_t at a given address ( <0xA00)
//! \param[in] uint16_t Format (F_BYTE or F_WORD)
//! \param[in] uint16_t Addr (Address of data to be written)
//! \param[in] uint16_t Data (shifted data)
//
//*****************************************************************************
void WriteMem_430Xv2(uint16_t Format, uint32_t Addr, uint16_t Data)
{
    // Check Init State at the beginning
    IR_Shift(IR_CNTRL_SIG_CAPTURE);
    if(DR_Shift16(0) & 0x0301)
    {
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        if  (Format == F_WORD)
        {
            DR_Shift16(0x0500);
        }
        else
        {
            DR_Shift16(0x0510);
        }
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift20(Addr);

        SetTCLK();
        // New style: Only apply data during clock high phase
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(Data);           // Shift in 16 bits
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x0501);
        SetTCLK();
        // one or more cycle, so CPU is driving correct MAB
        ClrTCLK();
        SetTCLK();
        // Processor is now again in Init State
    }
}

//*****************************************************************************
//
//! \brief This function writes an array of words into the target memory.
//! \param[in] uint16_t StartAddr (Start address of target memory)
//! \param[in] uint16_t Length (Number of words to be programmed)
//! \param[in] uint16_t *DataArray (Pointer to array with the data)
//
//*****************************************************************************
void WriteMemQuick_430Xv2(uint32_t StartAddr, uint32_t Length, uint16_t const *DataArray)
{
    uint32_t i;

    for (i = 0; i < Length; i++)
    {
        WriteMem_430Xv2(F_WORD, StartAddr, DataArray[i]);
        StartAddr += 2;
    }
}

//*****************************************************************************
//
//! \brief This function reads one byte/word from a given address in memory
//! \param[in] word Format (F_BYTE or F_WORD)
//! \param[in] word Addr (address of memory)
//! \return word (content of the addressed memory location)
//
//*****************************************************************************
uint16_t ReadMem_430Xv2(uint16_t Format, uint32_t Addr)
{
    uint16_t TDOword = 0;
    MsDelay(1);
    // Check Init State at the beginning
    IR_Shift(IR_CNTRL_SIG_CAPTURE);
    if(DR_Shift16(0) & 0x0301)
    {
        // Read Memory
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        if  (Format == F_WORD)
        {
            DR_Shift16(0x0501);             // Set uint16_t read
        }
        else
        {
            DR_Shift16(0x0511);             // Set byte read
        }
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift20(Addr);                   // Set address
        IR_Shift(IR_DATA_TO_ADDR);
        SetTCLK();
        ClrTCLK();
        TDOword = DR_Shift16(0x0000);       // Shift out 16 bits

        SetTCLK();
        // one or more cycle, so CPU is driving correct MAB
        ClrTCLK();
        SetTCLK();
        // Processor is now again in Init State
    }
    return TDOword;
}

//*****************************************************************************
//
//! \brief This function reads an array of words from the memory.
//! \param[in] uint16_t StartAddr (Start address of memory to be read)
//! \param[in] uint16_t Length (Number of words to be read)
//! \param[out] uint16_t *DataArray (Pointer to array for the data)
//
//*****************************************************************************
void ReadMemQuick_430Xv2(uint32_t StartAddr, uint32_t Length, uint16_t *DataArray)
{
    uint32_t i, lPc = 0;

    // Set PC to 'safe' address
    if((IR_Shift(IR_CNTRL_SIG_CAPTURE) == JTAG_ID99) || (IR_Shift(IR_CNTRL_SIG_CAPTURE) == JTAG_ID98))
    {
        lPc = 0x00000004;
    }

    SetPC_430Xv2(StartAddr);
    SetTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x0501);
    IR_Shift(IR_ADDR_CAPTURE);

    IR_Shift(IR_DATA_QUICK);

    for (i = 0; i < Length; i++)
    {
        SetTCLK();
        ClrTCLK();
        *DataArray++   = DR_Shift16(0);  // Read data from memory.
    }

    if(lPc)
    {
        SetPC_430Xv2(lPc);
    }
    SetTCLK();
}




