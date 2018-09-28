/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : Q4CLeft.h
**     Project     : TEAM_Robot
**     Processor   : MK22FX512VLK12
**     Component   : QuadCounter
**     Version     : Component 01.031, Driver 01.00, CPU db: 3.00.000
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-09-28, 13:35, # CodeGen: 0
**     Abstract    :
**
This driver implements a quadrature encoder using two signals (C1 and C2) to generate position information.
**     Settings    :
**          Component name                                 : Q4CLeft
**          C1 and C2 swapped                              : no
**          Counter Type                                   : 32bit
**          Method                                         : 
**            Sampling                                     : Enabled
**              Error Correction                           : no
**              C1                                         : Cx
**              C2                                         : Cx
**            Input Capture                                : Disabled
**          Shell                                          : Enabled
**            Shell                                        : CLS1
**            Utility                                      : UTIL1
**     Contents    :
**         GetPos       - Q4CLeft_QuadCntrType Q4CLeft_GetPos(void);
**         SetPos       - void Q4CLeft_SetPos(Q4CLeft_QuadCntrType pos);
**         GetVal       - uint8_t Q4CLeft_GetVal(void);
**         Sample       - void Q4CLeft_Sample(void);
**         NofErrors    - uint16_t Q4CLeft_NofErrors(void);
**         SwapPins     - uint8_t Q4CLeft_SwapPins(bool swap);
**         Deinit       - void Q4CLeft_Deinit(void);
**         Init         - void Q4CLeft_Init(void);
**         ParseCommand - uint8_t Q4CLeft_ParseCommand(const unsigned char *cmd, bool *handled, const...
**
**Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**All Rights Reserved.
**
**Redistribution and use in source and binary forms, with or without modification,
**are permitted provided that the following conditions are met:
**
**o Redistributions of source code must retain the above copyright notice, this list
**  of conditions and the following disclaimer.
**
**o Redistributions in binary form must reproduce the above copyright notice, this
**  list of conditions and the following disclaimer in the documentation and/or
**  other materials provided with the distribution.
**
**o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**  contributors may be used to endorse or promote products derived from this
**  software without specific prior written permission.
**
**THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**http: www.freescale.com
**mail: support@freescale.com
** ###################################################################*/
/*!
** @file Q4CLeft.h
** @version 01.00
** @brief
**
This driver implements a quadrature encoder using two signals (C1 and C2) to generate position information.
*/         
/*!
**  @addtogroup Q4CLeft_module Q4CLeft module documentation
**  @{
*/         

#ifndef __Q4CLeft_H
#define __Q4CLeft_H

/* MODULE Q4CLeft. */
#include "MCUC1.h" /* SDK and API used */
#include "Q4CLeftconfig.h" /* configuration */

/* Include inherited components */
#include "C12.h"
#include "C23.h"
#include "CLS1.h"
#include "UTIL1.h"
#include "MCUC1.h"


#define Q4CLeft_SWAP_PINS  0 /* 1: C1 and C2 are swapped */
#define Q4CLeft_SWAP_PINS_AT_RUNTIME  1 /* 1: C1 and C2 are swapped at runtime, if SwapPins() method is available */
#define Q4CLeft_GET_C1_PIN()      (C12_GetVal())
#define Q4CLeft_GET_C2_PIN()      (C23_GetVal())
#if Q4CLeft_SWAP_PINS
  #define Q4CLeft_GET_C1_C2_PINS()               ((Q4CLeft_GET_C2_PIN()!=0?2:0)|(Q4CLeft_GET_C1_PIN()!=0?1:0))
  #define Q4CLeft_GET_C1_C2_PINS_SWAPPED()       ((Q4CLeft_GET_C1_PIN()!=0?2:0)|(Q4CLeft_GET_C2_PIN()!=0?1:0))
#else
  #define Q4CLeft_GET_C1_C2_PINS()               ((Q4CLeft_GET_C1_PIN()!=0?2:0)|(Q4CLeft_GET_C2_PIN()!=0?1:0))
  #define Q4CLeft_GET_C1_C2_PINS_SWAPPED()       ((Q4CLeft_GET_C2_PIN()!=0?2:0)|(Q4CLeft_GET_C1_PIN()!=0?1:0))
#endif

typedef uint32_t Q4CLeft_QuadCntrType;
#define Q4CLeft_CNTR_BITS  32
  /*!< Number of bits in counter */

#define Q4CLeft_PARSE_COMMAND_ENABLED  1  /* set to 1 if method ParseCommand() is present, 0 otherwise */


void Q4CLeft_SetPos(Q4CLeft_QuadCntrType pos);
/*
** ===================================================================
**     Method      :  Q4CLeft_SetPos (component QuadCounter)
**     Description :
**         Sets the position information. Can be used as well to reset
**         the position information.
**     Parameters  :
**         NAME            - DESCRIPTION
**         pos             - Position value to be set.
**     Returns     : Nothing
** ===================================================================
*/

Q4CLeft_QuadCntrType Q4CLeft_GetPos(void);
/*
** ===================================================================
**     Method      :  Q4CLeft_GetPos (component QuadCounter)
**     Description :
**         Returns the current position based on the encoder tracking.
**     Parameters  : None
**     Returns     :
**         ---             - position
** ===================================================================
*/

void Q4CLeft_Sample(void);
/*
** ===================================================================
**     Method      :  Q4CLeft_Sample (component QuadCounter)
**     Description :
**         Call this method to periodically sample the signals.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

uint16_t Q4CLeft_NofErrors(void);
/*
** ===================================================================
**     Method      :  Q4CLeft_NofErrors (component QuadCounter)
**     Description :
**         Returns the number of decoding errors
**     Parameters  : None
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

void Q4CLeft_Deinit(void);
/*
** ===================================================================
**     Method      :  Q4CLeft_Deinit (component QuadCounter)
**     Description :
**         Module de-initialization method
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void Q4CLeft_Init(void);
/*
** ===================================================================
**     Method      :  Q4CLeft_Init (component QuadCounter)
**     Description :
**         Module initialization method
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

uint8_t Q4CLeft_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
/*
** ===================================================================
**     Method      :  Q4CLeft_ParseCommand (component QuadCounter)
**     Description :
**         Handler to process shell commands
**     Parameters  :
**         NAME            - DESCRIPTION
**         cmd             - Command string to be parsed
**       * handled         - Pointer to boolean. The handler
**                           sets this variable to TRUE if command was
**                           handled, otherwise let it untouched.
**         io              - Pointer to I/O structure
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint8_t Q4CLeft_GetVal(void);
/*
** ===================================================================
**     Method      :  Q4CLeft_GetVal (component QuadCounter)
**     Description :
**         Returns the quadrature value (0, 1, 2 or 3)
**     Parameters  : None
**     Returns     :
**         ---             - Quadrature value (0-3)
** ===================================================================
*/

uint8_t Q4CLeft_SwapPins(bool swap);
/*
** ===================================================================
**     Method      :  Q4CLeft_SwapPins (component QuadCounter)
**     Description :
**         Swap the two pins
**     Parameters  :
**         NAME            - DESCRIPTION
**         swap            - if C1 and C2 pins shall be swapped.
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

/* END Q4CLeft. */

#endif
/* ifndef __Q4CLeft_H */
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
