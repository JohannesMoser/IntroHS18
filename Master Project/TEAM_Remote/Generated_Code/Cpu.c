/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : Cpu.c
**     Project     : Andy_Johannes_Remote
**     Processor   : MK20DX128VFT5
**     Component   : MK20DX128FT5
**     Version     : Component 01.001, Driver 01.04, CPU db: 3.00.000
**     Repository  : Kinetis
**     Datasheet   : K20P48M50SF0RM Rev. 1, Oct 2011
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-10-04, 15:00, # CodeGen: 0
**     Abstract    :
**
**     Settings    :
**          Component name                                 : Cpu
**          CPU type                                       : MK20DX128VFT5
**          CPU                                            : CPU
**          MemModelDev                                    : MemModel_FlexMem
**          Clock settings                                 : 
**            Internal oscillator                          : 
**              Slow internal reference clock [kHz]        : 32.768
**              Initialize slow trim value                 : no
**              Fast internal reference clock [MHz]        : 4
**              Initialize fast trim value                 : no
**            RTC oscillator                               : Disabled
**            System oscillator 0                          : Enabled
**              Clock source                               : External crystal
**                Clock input pin                          : 
**                  Pin name                               : EXTAL0/PTA18/FTM0_FLT2/FTM_CLKIN0
**                  Pin signal                             : EXTAL
**                Clock output pin                         : 
**                  Pin name                               : XTAL0/PTA19/FTM1_FLT0/FTM_CLKIN1/LPTMR0_ALT1
**                  Pin signal                             : XTAL
**                Clock frequency [MHz]                    : 8
**                Capacitor load                           : 0pF
**                Oscillator operating mode                : Low power
**            Clock source settings                        : 1
**              Clock source setting 0                     : 
**                Internal reference clock                 : 
**                  MCGIRCLK clock                         : Enabled
**                  MCGIRCLK in stop                       : Disabled
**                  MCGIRCLK source                        : Slow
**                  MCGIRCLK clock [MHz]                   : 0.032768
**                External reference clock                 : 
**                  OSC0ERCLK clock                        : Enabled
**                  OSC0ERCLK in stop                      : Disabled
**                  OSC0ERCLK clock [MHz]                  : 8
**                  ERCLK32K clock source                  : Auto select
**                  ERCLK32K. clock [kHz]                  : 0.001
**                MCG settings                             : 
**                  MCG mode                               : PEE
**                  MCG output clock                       : PLL clock
**                  MCG output [MHz]                       : 96
**                  MCG external ref. clock source         : System oscillator 0
**                  MCG external ref. clock [MHz]          : 8
**                  Clock monitor                          : Disabled
**                  FLL settings                           : 
**                    FLL module                           : Disabled
**                    FLL output [MHz]                     : 0
**                    MCGFFCLK clock [kHz]                 : 31.25
**                    Reference clock source               : External clock
**                      Reference clock divider            : Auto select
**                    FLL reference clock [kHz]            : 31.25
**                    Multiplication factor                : Auto select
**                  PLL 0 settings                         : 
**                    PLL module                           : Enabled
**                    PLL module in Stop                   : Disabled
**                    PLL output [MHz]                     : 96
**                    Reference clock divider              : Auto select
**                    PLL reference clock [MHz]            : 4
**                    Multiplication factor                : Auto select
**                    Loss of lock interrupt               : Disabled
**          Initialization priority                        : minimal priority
**          Watchdog disable                               : yes
**          Internal peripherals                           : 
**            NMI pin                                      : Disabled
**            Reset control                                : Enabled
**              Reset pin                                  : RESET_b
**              Reset pin signal                           : 
**              Filter in STOP                             : Disabled
**              Filter in RUN/WAIT                         : Disabled
**              Filter width                               : 1
**            Debug interface (JTAG)                       : 
**              JTAG Mode                                  : cJTAG/SWD
**                TDO                                      : Disabled
**                TCK                                      : Enabled
**                  TCK Pin                                : TSI0_CH1/PTA0/UART0_CTS_b/UART0_COL_b/FTM0_CH5/JTAG_TCLK/SWD_CLK/EZP_CLK
**                  TCK Pin signal                         : 
**                TMS                                      : Enabled
**                  TMS Pin                                : TSI0_CH4/PTA3/UART0_RTS_b/FTM0_CH0/JTAG_TMS/SWD_DIO
**                  TMS Pin signal                         : 
**            Flash memory organization                    : 
**              FlexNVM settings                           : Partition code: 0xFFFF
**                FlexNVM size                             : 32 KB
**                DFlash size                              : 32 KB
**                EEPROM size                              : 0 bytes
**                  Start                                  : 0x10000000
**                  Size                                   : 0x0
**                  Start                                  : 0x14000000
**                  Size                                   : 0x0
**                  FlexRAM                                : Disabled
**              Flash blocks                               : 2
**                Flash block 0                            : PFlash
**                  Address                                : 0x0
**                  Size                                   : 131072
**                  Write unit size                        : 4
**                  Erase unit size                        : 1024
**                  Protection unit size                   : 4096
**                Flash block 1                            : DFlash
**                  Address                                : 0x10000000
**                  Size                                   : 32768
**                  Write unit size                        : 4
**                  Erase unit size                        : 1024
**                  Protection unit size                   : 4096
**            Flexible memory controller                   : Disabled
**            Flash configuration field                    : Enabled
**              Security settings                          : 
**                Flash security                           : Disabled
**                Freescale failure analysis access        : Enabled
**                Mass erase                               : Enabled
**                Backdoor key security                    : Disabled
**                Backdoor key 0                           : 255
**                Backdoor key 1                           : 255
**                Backdoor key 2                           : 255
**                Backdoor key 3                           : 255
**                Backdoor key 4                           : 255
**                Backdoor key 5                           : 255
**                Backdoor key 6                           : 255
**                Backdoor key 7                           : 255
**              Protection regions                         : 
**                P-Flash protection settings              : 
**                  Protection region size                 : 4096
**                  P-Flash protection                     : 0xFFFFFFFF
**                  Protection regions                     : 
**                    Protection region 0                  : Unprotected
**                    Protection region 1                  : Unprotected
**                    Protection region 2                  : Unprotected
**                    Protection region 3                  : Unprotected
**                    Protection region 4                  : Unprotected
**                    Protection region 5                  : Unprotected
**                    Protection region 6                  : Unprotected
**                    Protection region 7                  : Unprotected
**                    Protection region 8                  : Unprotected
**                    Protection region 9                  : Unprotected
**                    Protection region 10                 : Unprotected
**                    Protection region 11                 : Unprotected
**                    Protection region 12                 : Unprotected
**                    Protection region 13                 : Unprotected
**                    Protection region 14                 : Unprotected
**                    Protection region 15                 : Unprotected
**                    Protection region 16                 : Unprotected
**                    Protection region 17                 : Unprotected
**                    Protection region 18                 : Unprotected
**                    Protection region 19                 : Unprotected
**                    Protection region 20                 : Unprotected
**                    Protection region 21                 : Unprotected
**                    Protection region 22                 : Unprotected
**                    Protection region 23                 : Unprotected
**                    Protection region 24                 : Unprotected
**                    Protection region 25                 : Unprotected
**                    Protection region 26                 : Unprotected
**                    Protection region 27                 : Unprotected
**                    Protection region 28                 : Unprotected
**                    Protection region 29                 : Unprotected
**                    Protection region 30                 : Unprotected
**                    Protection region 31                 : Unprotected
**                D-Flash protection settings              : 
**                  Protection region size                 : 4096
**                  D-Flash protection                     : 0xFF
**                  Protection regions                     : 
**                    Protection region 0                  : Unprotected
**                    Protection region 1                  : Unprotected
**                    Protection region 2                  : Unprotected
**                    Protection region 3                  : Unprotected
**                    Protection region 4                  : Unprotected
**                    Protection region 5                  : Unprotected
**                    Protection region 6                  : Unprotected
**                    Protection region 7                  : Unprotected
**                Eeprom protection settings               : 
**                  Protection region size                 : 0
**                  Eeprom protection                      : 0xFF
**                  Protection regions                     : 
**                    Protection region 0                  : Unprotected
**                    Protection region 1                  : Unprotected
**                    Protection region 2                  : Unprotected
**                    Protection region 3                  : Unprotected
**                    Protection region 4                  : Unprotected
**                    Protection region 5                  : Unprotected
**                    Protection region 6                  : Unprotected
**                    Protection region 7                  : Unprotected
**              Peripheral settings                        : 
**                NMI function                             : Disabled
**                EzPort operation at boot                 : Enabled
**                Low power boot                           : Disabled
**            System control block settings                : Disabled
**            Power management controller                  : 
**              LVD reset                                  : Enabled
**              LVD voltage treshold                       : Low
**              LVW voltage treshold                       : Low
**              Bandgap buffer                             : Disabled
**              LVD interrupt                              : 
**                Interrupt                                : INT_LVD_LVW
**                Interrupt request                        : Disabled
**                Interrupt priority                       : 0 (Highest)
**                LVD interrupt                            : Disabled
**                LVW interrupt                            : Disabled
**            System Integration Module                    : 
**              CLKOUT pin control                         : Disabled
**              Clock gating control                       : Disabled
**          CPU interrupts/resets                          : 
**            NMI interrupt                                : Enabled
**              Interrupt                                  : INT_NMI
**            Hard Fault                                   : Disabled
**            Bus Fault                                    : Disabled
**            Usage Fault                                  : Disabled
**            Supervisor Call                              : Disabled
**            Pendable Service                             : Disabled
**            MCG                                          : Disabled
**          Low power mode settings                        : 
**            Allowed power modes                          : 
**              Very low power modes                       : Not allowed
**              Low leakage stop mode                      : Not allowed
**              Very low leakage stop mode                 : Not allowed
**            LLWU settings                                : Disabled
**            Operation mode settings                      : 
**              WAIT operation mode                        : 
**                Return to wait after ISR                 : no
**              SLEEP operation mode                       : 
**                Return to stop after ISR                 : no
**              STOP operation mode                        : Disabled
**          Clock configurations                           : 1
**            Clock configuration 0                        : 
**              __IRC_32kHz                                : 0.032768
**              __IRC_4MHz                                 : 2
**              __SYSTEM_OSC                               : 8
**              __RTC_OSC                                  : 0
**              Very low power mode                        : Disabled
**              Clock source setting                       : configuration 0
**                MCG mode                                 : PEE
**                MCG output [MHz]                         : 96
**                MCGIRCLK clock [MHz]                     : 0.032768
**                OSCERCLK clock [MHz]                     : 8
**                ERCLK32K. clock [kHz]                    : 0.001
**                MCGFFCLK [kHz]                           : 31.25
**              System clocks                              : 
**                Core clock prescaler                     : Auto select
**                Core clock                               : 48
**                Bus clock prescaler                      : Auto select
**                Bus clock                                : 24
**                Flash clock prescaler                    : Auto select
**                Flash clock                              : 24
**                PLL/FLL clock selection                  : PLL clock
**                  Clock frequency [MHz]                  : 96
**     Contents    :
**         No public methods
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file Cpu.c
** @version 01.04
** @brief
**
*/         
/*!
**  @addtogroup Cpu_module Cpu module documentation
**  @{
*/         

/* MODULE Cpu. */

/* {Default RTOS Adapter} No RTOS includes */
#include "MCUC1.h"
#include "LEDPin1.h"
#include "BitIoLdd17.h"
#include "TI1.h"
#include "TimerIntLdd1.h"
#include "TU1.h"
#include "CLS1.h"
#include "AS1.h"
#include "ASerialLdd1.h"
#include "RTT1.h"
#include "SW1.h"
#include "BitIoLdd10.h"
#include "SW2.h"
#include "BitIoLdd11.h"
#include "SW3.h"
#include "BitIoLdd12.h"
#include "SW4.h"
#include "BitIoLdd13.h"
#include "SW5.h"
#include "BitIoLdd14.h"
#include "SW6.h"
#include "BitIoLdd15.h"
#include "SW7.h"
#include "BitIoLdd16.h"
#include "PTA.h"
#include "PTB.h"
#include "PTD.h"
#include "USB0.h"
#include "USB1.h"
#include "CDC1.h"
#include "Tx1.h"
#include "Rx1.h"
#include "TMOUT1.h"
#include "WAIT1.h"
#include "UTIL1.h"
#include "HF1.h"
#include "CS1.h"
#include "KIN1.h"
#include "XF1.h"
#include "IFsh1.h"
#include "IntFlashLdd1.h"
#include "TmDt1.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Events.h"
#include "Cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Global variables */
volatile uint8_t SR_reg;               /* Current value of the FAULTMASK register */
volatile uint8_t SR_lock = 0x00U;      /* Lock */

/*
** ===================================================================
**     Method      :  Cpu_SetBASEPRI (component MK20DX128FT5)
**
**     Description :
**         This method sets the BASEPRI core register.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void Cpu_SetBASEPRI(uint32_t Level);

/*
** ===================================================================
**     Method      :  Cpu_INT_NMIInterrupt (component MK20DX128FT5)
**
**     Description :
**         This ISR services the Non Maskable Interrupt interrupt.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
PE_ISR(Cpu_INT_NMIInterrupt)
{
  Cpu_OnNMIINT();
}

/*
** ===================================================================
**     Method      :  Cpu_Cpu_Interrupt (component MK20DX128FT5)
**
**     Description :
**         This ISR services an unused interrupt/exception vector.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
PE_ISR(Cpu_Interrupt)
{
  /* This code can be changed using the CPU component property "Build Options / Unhandled int code" */
  PE_DEBUGHALT();
}


/*** !!! Here you can place your own code using property "User data declarations" on the build options tab. !!! ***/

/*lint -esym(765,__init_hardware) Disable MISRA rule (8.10) checking for symbols (__init_hardware). The function is linked to the EWL library */
/*lint -esym(765,Cpu_Interrupt) Disable MISRA rule (8.10) checking for symbols (Cpu_Interrupt). */
void __init_hardware(void)
{

  /*** !!! Here you can place your own code before PE initialization using property "User code before PE initialization" on the build options tab. !!! ***/

  /*** ### MK20DX128VFT5 "Cpu" init code ... ***/
  /*** PE initialization code after reset ***/
  SCB_VTOR = (uint32_t)(&__vect_table); /* Set the interrupt vector table position */
  /* Disable the WDOG module */
  /* WDOG_UNLOCK: WDOGUNLOCK=0xC520 */
  WDOG_UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xC520); /* Key 1 */
  /* WDOG_UNLOCK: WDOGUNLOCK=0xD928 */
  WDOG_UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xD928); /* Key 2 */
  /* WDOG_STCTRLH: ??=0,DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,??=0,??=1,WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,WDOGEN=0 */
  WDOG_STCTRLH = WDOG_STCTRLH_BYTESEL(0x00) |
                 WDOG_STCTRLH_WAITEN_MASK |
                 WDOG_STCTRLH_STOPEN_MASK |
                 WDOG_STCTRLH_ALLOWUPDATE_MASK |
                 WDOG_STCTRLH_CLKSRC_MASK |
                 0x0100U;

  /* System clock initialization */
  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=1,??=0,??=0,??=0,??=0,OUTDIV4=3,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x01) |
                SIM_CLKDIV1_OUTDIV4(0x03); /* Set the system prescalers to safe value */
  /* SIM_SCGC5: PORTD=1,PORTC=1,PORTB=1,PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK |
               SIM_SCGC5_PORTC_MASK |
               SIM_SCGC5_PORTB_MASK |
               SIM_SCGC5_PORTA_MASK;   /* Enable clock gate for ports to enable pin routing */
  if ((PMC_REGSC & PMC_REGSC_ACKISO_MASK) != 0x0U) {
    /* PMC_REGSC: ACKISO=1 */
    PMC_REGSC |= PMC_REGSC_ACKISO_MASK; /* Release IO pads after wakeup from VLLS mode. */
  }
  /* SIM_CLKDIV1: OUTDIV1=1,OUTDIV2=3,??=0,??=0,??=0,??=0,OUTDIV4=3,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x01) |
                SIM_CLKDIV1_OUTDIV2(0x03) |
                SIM_CLKDIV1_OUTDIV4(0x03); /* Update system prescalers */
  /* SIM_SOPT2: PLLFLLSEL=1 */
  SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK; /* Select PLL as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=3 */
  SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0x03); /* LPO 1kHz oscillator drives 32 kHz clock for various peripherals */
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* Switch to FBE Mode */
  /* MCG_C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
  MCG_C2 = (MCG_C2_RANGE0(0x02) | MCG_C2_EREFS0_MASK);
  /* OSC0_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0_CR = OSC_CR_ERCLKEN_MASK;
  /* MCG_C7: OSCSEL=0 */
  MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL_MASK);
  /* MCG_C1: CLKS=2,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG_C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
  /* MCG_C4: DMX32=0,DRST_DRS=0 */
  MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
  /* MCG_C5: ??=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
  MCG_C5 = MCG_C5_PRDIV0(0x01);
  /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG_C6 = MCG_C6_VDIV0(0x00);
  while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
  }
  while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG_S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to PBE Mode */
  /* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0 */
  MCG_C6 = (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x00));
  while((MCG_S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  while((MCG_S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
  }
  /* Switch to PEE Mode */
  /* MCG_C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG_C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK);
  while((MCG_S & 0x0CU) != 0x0CU) {    /* Wait until output of the PLL is selected */
  }
  /*** End of PE initialization code after reset ***/

  /*** !!! Here you can place your own code after PE initialization using property "User code after PE initialization" on the build options tab. !!! ***/

}

/*
** ===================================================================
**     Method      :  Cpu_SetBASEPRI (component MK20DX128FT5)
**
**     Description :
**         This method sets the BASEPRI core register.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
/*lint -save  -e586 -e950 Disable MISRA rule (2.1,1.1) checking. */
#ifdef _lint
  #define Cpu_SetBASEPRI(Level)  /* empty */
#else
void Cpu_SetBASEPRI(uint32_t Level) {
  __asm ("msr basepri, %[input]"::[input] "r" (Level):);
}
#endif
/*lint -restore Enable MISRA rule (2.1,1.1) checking. */


/*
** ===================================================================
**     Method      :  PE_low_level_init (component MK20DX128FT5)
**
**     Description :
**         Initializes beans and provides common register initialization. 
**         The method is called automatically as a part of the 
**         application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void PE_low_level_init(void)
{
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
      /* Initialization of the SIM module */
        /* Initialization of the RCM module */
  /* RCM_RPFW: RSTFLTSEL=0 */
  RCM_RPFW &= (uint8_t)~(uint8_t)(RCM_RPFW_RSTFLTSEL(0x1F));
  /* RCM_RPFC: RSTFLTSS=0,RSTFLTSRW=0 */
  RCM_RPFC &= (uint8_t)~(uint8_t)(
               RCM_RPFC_RSTFLTSS_MASK |
               RCM_RPFC_RSTFLTSRW(0x03)
              );
        /* Initialization of the FTFL_FlashConfig module */
      /* Initialization of the PMC module */
  /* PMC_REGSC: ACKISO=0,BGBE=0 */
  PMC_REGSC &= (uint8_t)~(uint8_t)(
                PMC_REGSC_ACKISO_MASK |
                PMC_REGSC_BGBE_MASK
               );
  /* PMC_LVDSC1: LVDACK=1,LVDIE=0,LVDRE=1,LVDV=0 */
  PMC_LVDSC1 = (uint8_t)((PMC_LVDSC1 & (uint8_t)~(uint8_t)(
                PMC_LVDSC1_LVDIE_MASK |
                PMC_LVDSC1_LVDV(0x03)
               )) | (uint8_t)(
                PMC_LVDSC1_LVDACK_MASK |
                PMC_LVDSC1_LVDRE_MASK
               ));
  /* PMC_LVDSC2: LVWACK=1,LVWIE=0,LVWV=0 */
  PMC_LVDSC2 = (uint8_t)((PMC_LVDSC2 & (uint8_t)~(uint8_t)(
                PMC_LVDSC2_LVWIE_MASK |
                PMC_LVDSC2_LVWV(0x03)
               )) | (uint8_t)(
                PMC_LVDSC2_LVWACK_MASK
               ));
  /* SMC_PMPROT: ??=0,??=0,AVLP=0,??=0,ALLS=0,??=0,AVLLS=0,??=0 */
  SMC_PMPROT = 0x00U;                  /* Setup Power mode protection register */
  /* Common initialization of the CPU registers */
  /* PORTA_ISFR: ISF=0x10 */
  PORTA_ISFR = PORT_ISFR_ISF(0x10);
  /* Common initialization of the CPU registers */
  /* NVICIP40: PRI40=0 */
  NVICIP40 = NVIC_IP_PRI40(0x00);
  /* NVICIP41: PRI41=0 */
  NVICIP41 = NVIC_IP_PRI41(0x00);
  /* NVICIP43: PRI43=0 */
  NVICIP43 = NVIC_IP_PRI43(0x00);
  /* NVICIP35: PRI35=0 */
  NVICIP35 = NVIC_IP_PRI35(0x00);
  /* NVICIP8: PRI8=0 */
  NVICIP8 = NVIC_IP_PRI8(0x00);
  /* PORTA_PCR4: ISF=0,IRQC=0,LK=0,DSE=0,ODE=0,PFE=0,SRE=0,PE=1,PS=1 */
  PORTA_PCR4 = (uint32_t)((PORTA_PCR4 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_IRQC(0x0F) |
                PORT_PCR_LK_MASK |
                PORT_PCR_DSE_MASK |
                PORT_PCR_ODE_MASK |
                PORT_PCR_PFE_MASK |
                PORT_PCR_SRE_MASK
               )) | (uint32_t)(
                PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK
               ));
  /* PORTA_PCR1: ISF=0,PE=1,PS=1 */
  PORTA_PCR1 = (uint32_t)((PORTA_PCR1 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK
               )) | (uint32_t)(
                PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK
               ));
  /* PORTA_PCR2: ISF=0,PE=1,PS=1 */
  PORTA_PCR2 = (uint32_t)((PORTA_PCR2 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK
               )) | (uint32_t)(
                PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK
               ));
  /* PORTB_PCR0: ISF=0,PE=1,PS=1 */
  PORTB_PCR0 = (uint32_t)((PORTB_PCR0 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK
               )) | (uint32_t)(
                PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK
               ));
  /* PORTB_PCR2: ISF=0,PE=1,PS=1 */
  PORTB_PCR2 = (uint32_t)((PORTB_PCR2 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK
               )) | (uint32_t)(
                PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK
               ));
  /* PORTD_PCR0: ISF=0,PE=1,PS=1 */
  PORTD_PCR0 = (uint32_t)((PORTD_PCR0 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK
               )) | (uint32_t)(
                PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK
               ));
  /* PORTD_PCR5: ISF=0,PE=1,PS=1 */
  PORTD_PCR5 = (uint32_t)((PORTD_PCR5 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK
               )) | (uint32_t)(
                PORT_PCR_PE_MASK |
                PORT_PCR_PS_MASK
               ));
  /* PORTD_DFCR: CS=0 */
  PORTD_DFCR &= (uint32_t)~(uint32_t)(PORT_DFCR_CS_MASK);
  /* PORTD_DFWR: FILT=0 */
  PORTD_DFWR &= (uint32_t)~(uint32_t)(PORT_DFWR_FILT(0x1F));
  /* ### McuLibConfig "MCUC1" init code ... */
  MCUC1_Init();
  /* ### BitIO_LDD "BitIoLdd17" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd17_Init(NULL);
  /* ### TimerInt_LDD "TimerIntLdd1" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)TimerIntLdd1_Init(NULL);
  /* ### TimerInt "TI1" init code ... */
  WAIT1_Init(); /* ### Wait "WAIT1" init code ... */
  CS1_Init(); /* ### CriticalSection "CS1" init code ... */
  XF1_Init(); /* ### XFormat "XF1" init code ... */
  CLS1_Init(); /* ### Shell "CLS1" init code ... */
  /* ### Asynchro serial "AS1" init code ... */
  AS1_Init();
  RTT1_Init(); /* ### SeggerRTT "RTT1" init code ... */
  /* ### BitIO_LDD "BitIoLdd10" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd10_Init(NULL);
  /* ### BitIO_LDD "BitIoLdd11" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd11_Init(NULL);
  /* ### BitIO_LDD "BitIoLdd12" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd12_Init(NULL);
  /* ### BitIO_LDD "BitIoLdd13" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd13_Init(NULL);
  /* ### BitIO_LDD "BitIoLdd14" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd14_Init(NULL);
  /* ### BitIO_LDD "BitIoLdd15" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd15_Init(NULL);
  /* ### BitIO_LDD "BitIoLdd16" component auto initialization. Auto initialization feature can be disabled by component property "Auto initialization". */
  (void)BitIoLdd16_Init(NULL);
  /* ### Init_GPIO "PTA" init code ... */
  PTA_Init();


  /* ### Init_GPIO "PTB" init code ... */
  PTB_Init();


  /* ### Init_GPIO "PTD" init code ... */
  PTD_Init();


  /* ### Init_USB_OTG "USB0" init code ... */
  /* ### Call "USB0_Init();" init method in a user code, i.e. in the main code */

  /* ### Note:   To enable automatic calling of the "USB0" init code here,
                 the 'Call Init method' property must be set to 'yes'.
   */


  /* ### Timeout "TMOUT1" init code ... */
  TMOUT1_Init();
  Tx1_Init(); /* ### RingBuffer "Tx1" init code ... */
  Rx1_Init(); /* ### RingBuffer "Rx1" init code ... */
  (void)USB1_Init();
  /* ### HardFault "HF1" init code ... */
  HF1_Init();
  /* ### KinetisTools "KIN1" init code ... */
  /* ### IntFLASH "IFsh1" init code ... */
  IFsh1_Init();
  /* ### GenericTimeDate "TmDt1" init code ... */
#if TmDt1_INIT_IN_STARTUP
  (void)TmDt1_Init();
#endif
  /* Enable interrupts of the given priority level */
  Cpu_SetBASEPRI(0U);
}
  /* Flash configuration field */
  __attribute__ ((section (".cfmconfig"))) const uint8_t _cfm[0x10] = {
   /* NV_BACKKEY3: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY2: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY1: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY0: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY7: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY6: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY5: KEY=0xFF */
    0xFFU,
   /* NV_BACKKEY4: KEY=0xFF */
    0xFFU,
   /* NV_FPROT3: PROT=0xFF */
    0xFFU,
   /* NV_FPROT2: PROT=0xFF */
    0xFFU,
   /* NV_FPROT1: PROT=0xFF */
    0xFFU,
   /* NV_FPROT0: PROT=0xFF */
    0xFFU,
   /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */
    0x7EU,
   /* NV_FOPT: ??=1,??=1,??=1,??=1,??=1,NMI_DIS=0,EZPORT_DIS=1,LPBOOT=1 */
    0xFBU,
   /* NV_FEPROT: EPROT=0xFF */
    0xFFU,
   /* NV_FDPROT: DPROT=0xFF */
    0xFFU
  };

/* END Cpu. */

#ifdef __cplusplus
}  /* extern "C" */
#endif

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
