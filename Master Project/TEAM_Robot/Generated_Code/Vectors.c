/** ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : Vectors.c
**     Project     : Andy_Johannes_Robot
**     Processor   : MK22FX512VLK12
**     Version     : Component 01.014, Driver 01.04, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
<<<<<<< HEAD
**     Date/Time   : 2018-11-08, 15:13, # CodeGen: 15
=======
**     Date/Time   : 2018-10-18, 15:16, # CodeGen: 6
>>>>>>> 0dbae26c797a6984e3df0b54283e7146fc1a1a20
**     Abstract    :
**
**     Settings    :
**
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
** @file Vectors.c                                                  
** @version 01.04
** @brief
**
*/         
/*!
**  @addtogroup Vectors_module Vectors module documentation
**  @{
*/         

  #include "Cpu.h"
  #include "CLS1.h"
  #include "MCUC1.h"
  #include "WAIT1.h"
  #include "CS1.h"
  #include "HF1.h"
  #include "XF1.h"
  #include "KIN1.h"
  #include "UTIL1.h"
  #include "LEDPin1.h"
  #include "BitIoLdd1.h"
  #include "LEDPin2.h"
  #include "BitIoLdd2.h"
  #include "SW1.h"
  #include "ExtIntLdd1.h"
<<<<<<< HEAD
  #include "FRTOS1.h"
  #include "RTOSCNTRLDD1.h"
=======
>>>>>>> 0dbae26c797a6984e3df0b54283e7146fc1a1a20
  #include "TI1.h"
  #include "TimerIntLdd1.h"
  #include "TU1.h"
  #include "BUZ1.h"
  #include "BitIoLdd4.h"
  #include "RTT1.h"
  #include "SYS1.h"
  #include "LED_IR.h"
  #include "LEDpin3.h"
  #include "BitIoLdd5.h"
  #include "RefCnt.h"
  #include "IR1.h"
  #include "BitIoLdd6.h"
  #include "IR2.h"
  #include "BitIoLdd7.h"
  #include "IR3.h"
  #include "BitIoLdd8.h"
  #include "IR4.h"
  #include "BitIoLdd9.h"
  #include "IR5.h"
  #include "BitIoLdd10.h"
  #include "IR6.h"
  #include "BitIoLdd11.h"
  #include "Q4CLeft.h"
  #include "C12.h"
  #include "BitIoLdd16.h"
  #include "C23.h"
  #include "BitIoLdd17.h"
  #include "Q4CRight.h"
  #include "C13.h"
  #include "BitIoLdd18.h"
  #include "C25.h"
  #include "BitIoLdd19.h"
  #include "MOTTU.h"
  #include "DIRL.h"
  #include "BitIoLdd12.h"
  #include "PWMR.h"
  #include "PwmLdd2.h"
  #include "DIRR.h"
  #include "BitIoLdd13.h"
  #include "PWML.h"
  #include "PwmLdd3.h"
  #include "QuadInt.h"
  #include "TimerIntLdd2.h"
  #include "TU_QuadInt.h"
  #include "TMOUT1.h"
  #include "USB1.h"
  #include "CDC1.h"
  #include "Tx1.h"
  #include "Rx1.h"
  #include "IFsh1.h"
  #include "IntFlashLdd1.h"
  #include "USB0.h"
  #include "ADC_Bat.h"
  #include "TmDt1.h"
  #include "Events.h"


  /* ISR prototype */
  extern uint32_t __SP_INIT;
  extern
  #ifdef __cplusplus
  "C"
  #endif
  void __thumb_startup( void );
  
  
  /*lint -esym(765,__vect_table) Disable MISRA rule (8.10) checking for symbols (__vect_table). Definition of the interrupt vector table placed by linker on a predefined location. */
  /*lint -save  -e926 -e927 -e928 -e929 Disable MISRA rule (11.4) checking. Need to explicitly cast pointers to the general ISR for Interrupt vector table */
  
  __attribute__ ((section (".vectortable"))) const tVectorTable __vect_table = { /* Interrupt vector table */
  
    /* ISR name                             No. Address      Pri Name                           Description */
    &__SP_INIT,                        /* 0x00  0x00000000   -   ivINT_Initial_Stack_Pointer    used by PE */
    {
    (tIsrFunc)&__thumb_startup,        /* 0x01  0x00000004   -   ivINT_Initial_Program_Counter  used by PE */
    (tIsrFunc)&Cpu_INT_NMIInterrupt,   /* 0x02  0x00000008   -2   ivINT_NMI                      used by PE */
    (tIsrFunc)&HF1_HardFaultHandler,   /* 0x03  0x0000000C   -1   ivINT_Hard_Fault               used by PE */
    (tIsrFunc)&Cpu_ivINT_Mem_Manage_Fault, /* 0x04  0x00000010   -   ivINT_Mem_Manage_Fault         unused by PE */
    (tIsrFunc)&Cpu_ivINT_Bus_Fault,    /* 0x05  0x00000014   -   ivINT_Bus_Fault                unused by PE */
    (tIsrFunc)&Cpu_ivINT_Usage_Fault,  /* 0x06  0x00000018   -   ivINT_Usage_Fault              unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved7,    /* 0x07  0x0000001C   -   ivINT_Reserved7                unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved8,    /* 0x08  0x00000020   -   ivINT_Reserved8                unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved9,    /* 0x09  0x00000024   -   ivINT_Reserved9                unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved10,   /* 0x0A  0x00000028   -   ivINT_Reserved10               unused by PE */
    (tIsrFunc)&vPortSVCHandler,        /* 0x0B  0x0000002C   -   ivINT_SVCall                   used by PE */
    (tIsrFunc)&Cpu_ivINT_DebugMonitor, /* 0x0C  0x00000030   -   ivINT_DebugMonitor             unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved13,   /* 0x0D  0x00000034   -   ivINT_Reserved13               unused by PE */
    (tIsrFunc)&vPortPendSVHandler,     /* 0x0E  0x00000038   -   ivINT_PendableSrvReq           used by PE */
    (tIsrFunc)&vPortTickHandler,       /* 0x0F  0x0000003C   -   ivINT_SysTick                  used by PE */
    (tIsrFunc)&Cpu_ivINT_DMA0,         /* 0x10  0x00000040   -   ivINT_DMA0                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA1,         /* 0x11  0x00000044   -   ivINT_DMA1                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA2,         /* 0x12  0x00000048   -   ivINT_DMA2                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA3,         /* 0x13  0x0000004C   -   ivINT_DMA3                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA4,         /* 0x14  0x00000050   -   ivINT_DMA4                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA5,         /* 0x15  0x00000054   -   ivINT_DMA5                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA6,         /* 0x16  0x00000058   -   ivINT_DMA6                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA7,         /* 0x17  0x0000005C   -   ivINT_DMA7                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA8,         /* 0x18  0x00000060   -   ivINT_DMA8                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA9,         /* 0x19  0x00000064   -   ivINT_DMA9                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA10,        /* 0x1A  0x00000068   -   ivINT_DMA10                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA11,        /* 0x1B  0x0000006C   -   ivINT_DMA11                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA12,        /* 0x1C  0x00000070   -   ivINT_DMA12                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA13,        /* 0x1D  0x00000074   -   ivINT_DMA13                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA14,        /* 0x1E  0x00000078   -   ivINT_DMA14                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA15,        /* 0x1F  0x0000007C   -   ivINT_DMA15                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_DMA_Error,    /* 0x20  0x00000080   -   ivINT_DMA_Error                unused by PE */
    (tIsrFunc)&Cpu_ivINT_MCM,          /* 0x21  0x00000084   -   ivINT_MCM                      unused by PE */
    (tIsrFunc)&Cpu_ivINT_FTFE,         /* 0x22  0x00000088   -   ivINT_FTFE                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_Read_Collision, /* 0x23  0x0000008C   -   ivINT_Read_Collision           unused by PE */
    (tIsrFunc)&Cpu_ivINT_LVD_LVW,      /* 0x24  0x00000090   -   ivINT_LVD_LVW                  unused by PE */
    (tIsrFunc)&Cpu_ivINT_LLWU,         /* 0x25  0x00000094   -   ivINT_LLWU                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_WDOG_EWM,     /* 0x26  0x00000098   -   ivINT_WDOG_EWM                 unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved39,   /* 0x27  0x0000009C   -   ivINT_Reserved39               unused by PE */
    (tIsrFunc)&Cpu_ivINT_I2C0,         /* 0x28  0x000000A0   -   ivINT_I2C0                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_I2C1,         /* 0x29  0x000000A4   -   ivINT_I2C1                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_SPI0,         /* 0x2A  0x000000A8   -   ivINT_SPI0                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_SPI1,         /* 0x2B  0x000000AC   -   ivINT_SPI1                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_I2S0_Tx,      /* 0x2C  0x000000B0   -   ivINT_I2S0_Tx                  unused by PE */
    (tIsrFunc)&Cpu_ivINT_I2S0_Rx,      /* 0x2D  0x000000B4   -   ivINT_I2S0_Rx                  unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved46,   /* 0x2E  0x000000B8   -   ivINT_Reserved46               unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART0_RX_TX,  /* 0x2F  0x000000BC   -   ivINT_UART0_RX_TX              unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART0_ERR,    /* 0x30  0x000000C0   -   ivINT_UART0_ERR                unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART1_RX_TX,  /* 0x31  0x000000C4   -   ivINT_UART1_RX_TX              unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART1_ERR,    /* 0x32  0x000000C8   -   ivINT_UART1_ERR                unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART2_RX_TX,  /* 0x33  0x000000CC   -   ivINT_UART2_RX_TX              unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART2_ERR,    /* 0x34  0x000000D0   -   ivINT_UART2_ERR                unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART3_RX_TX,  /* 0x35  0x000000D4   -   ivINT_UART3_RX_TX              unused by PE */
    (tIsrFunc)&Cpu_ivINT_UART3_ERR,    /* 0x36  0x000000D8   -   ivINT_UART3_ERR                unused by PE */
    (tIsrFunc)&Cpu_ivINT_ADC0,         /* 0x37  0x000000DC   -   ivINT_ADC0                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_CMP0,         /* 0x38  0x000000E0   -   ivINT_CMP0                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_CMP1,         /* 0x39  0x000000E4   -   ivINT_CMP1                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_FTM0,         /* 0x3A  0x000000E8   -   ivINT_FTM0                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_FTM1,         /* 0x3B  0x000000EC   -   ivINT_FTM1                     unused by PE */
    (tIsrFunc)&TU1_Interrupt,          /* 0x3C  0x000000F0   0   ivINT_FTM2                     used by PE */
    (tIsrFunc)&Cpu_ivINT_CMT,          /* 0x3D  0x000000F4   -   ivINT_CMT                      unused by PE */
    (tIsrFunc)&Cpu_ivINT_RTC,          /* 0x3E  0x000000F8   -   ivINT_RTC                      unused by PE */
    (tIsrFunc)&Cpu_ivINT_RTC_Seconds,  /* 0x3F  0x000000FC   -   ivINT_RTC_Seconds              unused by PE */
    (tIsrFunc)&RTOSCNTRLDD1_Interrupt, /* 0x40  0x00000100   8   ivINT_PIT0                     used by PE */
    (tIsrFunc)&TU_QuadInt_Interrupt,   /* 0x41  0x00000104   15   ivINT_PIT1                     used by PE */
    (tIsrFunc)&Cpu_ivINT_PIT2,         /* 0x42  0x00000108   -   ivINT_PIT2                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_PIT3,         /* 0x43  0x0000010C   -   ivINT_PIT3                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_PDB0,         /* 0x44  0x00000110   -   ivINT_PDB0                     unused by PE */
    (tIsrFunc)&USB_ISR,                /* 0x45  0x00000114   0   ivINT_USB0                     used by PE */
    (tIsrFunc)&Cpu_ivINT_USBDCD,       /* 0x46  0x00000118   -   ivINT_USBDCD                   unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved71,   /* 0x47  0x0000011C   -   ivINT_Reserved71               unused by PE */
    (tIsrFunc)&Cpu_ivINT_DAC0,         /* 0x48  0x00000120   -   ivINT_DAC0                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_MCG,          /* 0x49  0x00000124   -   ivINT_MCG                      unused by PE */
    (tIsrFunc)&Cpu_ivINT_LPTMR0,       /* 0x4A  0x00000128   -   ivINT_LPTMR0                   unused by PE */
    (tIsrFunc)&ExtIntLdd1_Interrupt,   /* 0x4B  0x0000012C   8   ivINT_PORTA                    used by PE */
    (tIsrFunc)&Cpu_ivINT_PORTB,        /* 0x4C  0x00000130   -   ivINT_PORTB                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_PORTC,        /* 0x4D  0x00000134   -   ivINT_PORTC                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_PORTD,        /* 0x4E  0x00000138   -   ivINT_PORTD                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_PORTE,        /* 0x4F  0x0000013C   -   ivINT_PORTE                    unused by PE */
    (tIsrFunc)&Cpu_ivINT_SWI,          /* 0x50  0x00000140   -   ivINT_SWI                      unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved81,   /* 0x51  0x00000144   -   ivINT_Reserved81               unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved82,   /* 0x52  0x00000148   -   ivINT_Reserved82               unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved83,   /* 0x53  0x0000014C   -   ivINT_Reserved83               unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved84,   /* 0x54  0x00000150   -   ivINT_Reserved84               unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved85,   /* 0x55  0x00000154   -   ivINT_Reserved85               unused by PE */
    (tIsrFunc)&Cpu_ivINT_CMP2,         /* 0x56  0x00000158   -   ivINT_CMP2                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_FTM3,         /* 0x57  0x0000015C   -   ivINT_FTM3                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_Reserved88,   /* 0x58  0x00000160   -   ivINT_Reserved88               unused by PE */
    (tIsrFunc)&Cpu_ivINT_ADC1,         /* 0x59  0x00000164   -   ivINT_ADC1                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_I2C2,         /* 0x5A  0x00000168   -   ivINT_I2C2                     unused by PE */
    (tIsrFunc)&Cpu_ivINT_CAN0_ORed_Message_buffer, /* 0x5B  0x0000016C   -   ivINT_CAN0_ORed_Message_buffer unused by PE */
    (tIsrFunc)&Cpu_ivINT_CAN0_Bus_Off, /* 0x5C  0x00000170   -   ivINT_CAN0_Bus_Off             unused by PE */
    (tIsrFunc)&Cpu_ivINT_CAN0_Error,   /* 0x5D  0x00000174   -   ivINT_CAN0_Error               unused by PE */
    (tIsrFunc)&Cpu_ivINT_CAN0_Tx_Warning, /* 0x5E  0x00000178   -   ivINT_CAN0_Tx_Warning          unused by PE */
    (tIsrFunc)&Cpu_ivINT_CAN0_Rx_Warning, /* 0x5F  0x0000017C   -   ivINT_CAN0_Rx_Warning          unused by PE */
    (tIsrFunc)&Cpu_ivINT_CAN0_Wake_Up, /* 0x60  0x00000180   -   ivINT_CAN0_Wake_Up             unused by PE */
    (tIsrFunc)&Cpu_ivINT_SDHC          /* 0x61  0x00000184   -   ivINT_SDHC                     unused by PE */
    }
  };
  /*lint -restore Enable MISRA rule (11.4) checking. */
  

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
