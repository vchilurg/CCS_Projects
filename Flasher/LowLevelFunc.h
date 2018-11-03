/*
 * LowLevelFunc.h
 *
 *  Created on: Jul 10, 2018
 *      Author: CGV
 */

#ifndef LOWLEVELFUNC_H_
#define LOWLEVELFUNC_H_



#endif /* LOWLEVELFUNC_H_ */
#include <stdint.h>
#include <msp430.h>

//******************************************************************************
// Global Variables SWB Options ************************************************
//******************************************************************************

uint8_t tdo_bit;               //holds the value of TDO-bit
uint8_t TCLK_saved;  // holds the last value of TCLK before entering a JTAG sequence
//uint16_t JtagId = 0;

/****************************************************************************/
/* Pin-to-Signal Assignments                                                */
/****************************************************************************/

// JTAG ports are P2.x
//! \brief JTAG output register
#define JTAGOUT          P2OUT
//! \brief JTAG input register
#define JTAGIN          P2IN
//! \brief JTAG direction register
#define JTAGDIR         P2DIR
//! \brief JTAG select register
#define JTAGSEL         P2SEL0
//! \brief P2.7 JTAG TDI/TDO input-output pin
#define TDIO             0x80
//! \brief P2.4 JTAG TCK input pin
#define TCK             0x10



//! \brief P2.2 JTAG Test input pin
/*#define TEST            0x04
//! \brief P2.3 Hardware RESET input pin
#define RST             0x08
//! \brief P2.7 TDI (former XOUT) receives TCLK
#define TCLK            TDI
//! \brief P2.5 JTAG TMS input pin
#define TMS             0x20
//! \brief P2.6 JTAG TDO output pin
#define TDO             0x40*/
/****************************************************************************/
/* Macros to control Spy-Bi-Wire-IF                                         */
/****************************************************************************/

#define     SBW_DELAY { _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP();}

#define     TMSH    JTAGOUT |= TDIO; SBW_DELAY; JTAGOUT &= ~TCK; SBW_DELAY; JTAGOUT |= TCK;
#define     TMSL    JTAGOUT &= ~TDIO; SBW_DELAY; JTAGOUT &= ~TCK; SBW_DELAY; JTAGOUT |= TCK;
#define     TMSLDH    JTAGOUT &= ~TDIO; SBW_DELAY; JTAGOUT &= ~TCK; SBW_DELAY; JTAGOUT |= TDIO; JTAGOUT |= TCK; // TMS = 0, then TCLK(TDI) immediately = 1

#define     TDIH    JTAGOUT |= TDIO; SBW_DELAY; JTAGOUT &= ~TCK; SBW_DELAY; JTAGOUT |= TCK;
#define     TDIL    JTAGOUT &= ~TDIO; SBW_DELAY; JTAGOUT &= ~TCK; SBW_DELAY; JTAGOUT |= TCK;

#define     TDOsbw  TDOI_dir(1); SBW_DELAY; JTAGOUT &= ~TCK; SBW_DELAY; JTAGOUT |= TCK; TDOI_dir(0);
#define     TDO_RD  TDOI_dir(1); SBW_DELAY; JTAGOUT &= ~TCK; SBW_DELAY; tdo_bit = JTAGIN; SBW_DELAY; JTAGOUT |= TCK; TDOI_dir(0);

#define     SetRST()   (JTAGOUT |= TDIO)
#define     ClrRST()   (JTAGOUT &= ~TDIO)
#define     SetTST()   (JTAGOUT |= TCK)
#define     ClrTST()   (JTAGOUT &= ~TCK)

#define   SetSBWTCK()     (JTAGOUT |=   TCK)
#define   ClrSBWTCK()     (JTAGOUT &=  ~TCK)
#define   SetSBWTDIO()    (JTAGOUT |=   TDIO)
#define   ClrSBWTDIO()    (JTAGOUT &=  ~TDIO)

/****************************************************************************/
/* TYPEDEFS                                                                 */
/****************************************************************************/

typedef unsigned short word;

/****************************************************************************/
/* FUNCTION PROTOTYPES                                                      */
/****************************************************************************/

void runProgramm(void);
void main(void);

void    InitController(void);
void    InitTarget(void);
void    DrvSignals( void );
void    RlsSignals(void);

void    MsDelay(word milliseconds);      // millisecond delay loop, uses Timer_A
void    usDelay(word microeconds);       // microsecond delay loop, uses nops

void    TDOI_dir( word dir );

void TMSH_TDIH(void);
void TMSL_TDIH(void);
void TMSL_TDIH_TDOrd(void);
void TMSL_TDIL_TDOrd(void);
void TMSH_TDIH_TDOrd(void);
void TMSH_TDIL_TDOrd(void);
void TMSL_TDIL(void);
void TMSH_TDIL(void);
void ClrTCLK(void);
void SetTCLK(void);

void ResetTAP(void);
uint32_t AllShifts(uint16_t Format, uint32_t Data);
uint32_t IR_Shift(uint8_t instruction);
uint16_t DR_Shift16(uint16_t data);
uint32_t DR_Shift20(uint32_t address);
uint16_t ExecutePOR_430Xv2(void);
void SetPC_430Xv2(uint32_t Addr);
uint32_t i_ReadJmbOut(void);
int16_t i_WriteJmbIn16(uint16_t dataX);
int16_t i_WriteJmbIn32(uint16_t dataX,uint16_t dataY);
void ConnectJTAG();
void EntrySequences_RstHigh_SBW();
void EntrySequences_RstLow_SBW();
void StopJtag (void);
uint16_t magicPattern(void);
uint16_t UnlockDevice_430Xv2(uint16_t* password, uint32_t passwordLength);
uint16_t GetCoreID (void);
uint16_t GetCoreipIdXv2();
uint16_t SyncJtag_AssertPor (void);
uint16_t GetDevice_430Xv2(void);
uint16_t ReleaseDevice_430Xv2(uint32_t Addr);
void WriteMem_430Xv2(uint16_t Format, uint32_t Addr, uint16_t Data);
void WriteMemQuick_430Xv2(uint32_t StartAddr, uint32_t Length, uint16_t const *DataArray);
uint16_t ReadMem_430Xv2(uint16_t Format, uint32_t Addr);
void ReadMemQuick_430Xv2(uint32_t StartAddr, uint32_t Length, uint16_t *DataArray);
uint16_t EraseCheck_430Xv2(uint32_t StartAddr, uint32_t Length);
uint16_t VerifyMem_430Xv2(uint32_t StartAddr, uint32_t Length, uint16_t const *DataArray);
uint16_t IsLockKeyProgrammed(void);
