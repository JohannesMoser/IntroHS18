/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : RNET1.h
**     Project     : Andy_Johannes_Robot
**     Processor   : MK22FX512VLK12
**     Component   : RNet
**     Version     : Component 01.095, Driver 01.00, CPU db: 3.00.000
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-12-13, 17:05, # CodeGen: 43
**     Abstract    :
**          This components implements the RNet Stack.
**     Settings    :
**          Component name                                 : RNET1
**          Transceiver                                    : 
**            Transceiver Type                             : nRF24L01+
**            nRF24L01+                                    : Enabled
**              nRF24L01+                                  : RF1
**              Radio Channel                              : 89
**              Data Rate                                  : 2000 kBit
**              Payload Size                               : 32
**              Address                                    : 0x11, 0x22, 0x33, 0x44, 0x55
**            SMAC                                         : Disabled
**          Network                                        : 
**            Address Size                                 : 8 Bits
**          Queues                                         : 
**            Rx Message Queue Size                        : 12
**            Tx Message Queue Size                        : 12
**            Message Queue Blocking Time (ms)             : 400
**          Send Retry Count                               : 0
**          Send Timeout (ms)                              : 200
**          System                                         : 
**            Utility                                      : UTIL1
**            RTOS                                         : FRTOS1
**            Shell                                        : Enabled
**              Remote StdIO                               : Enabled
**                Queue length                             : 48
**                Queue Timeout (ms)                       : 500
**              Shell                                      : CLS1
**     Contents    :
**         SetChannel   - uint8_t RNET1_SetChannel(uint8_t channel);
**         Process      - uint8_t RNET1_Process(void);
**         PowerUp      - uint8_t RNET1_PowerUp(void);
**         ParseCommand - uint8_t RNET1_ParseCommand(const unsigned char *cmd, bool *handled, const...
**         Init         - void RNET1_Init(void);
**         Deinit       - void RNET1_Deinit(void);
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
** @file RNET1.h
** @version 01.00
** @brief
**          This components implements the RNet Stack.
*/         
/*!
**  @addtogroup RNET1_module RNET1 module documentation
**  @{
*/         

#ifndef __RNET1_H
#define __RNET1_H

/* MODULE RNET1. */
#include "MCUC1.h" /* SDK and API used */
#include "RNET1config.h" /* configuration */

/* Include inherited components */
#include "RF1.h"
#include "MCUC1.h"
#include "UTIL1.h"
#include "FRTOS1.h"
#include "CLS1.h"


#define RNET1_PARSE_COMMAND_ENABLED  1  /* set to 1 if method ParseCommand() is present, 0 otherwise */

typedef enum {
  RNET1_RADIO_MSG_RECEIVED,        /* message has been received */
  RNET1_RADIO_MSG_SENT,            /* message has been sent */
  RNET1_RADIO_TIMEOUT,             /* timeout, no response received */
  RNET1_RADIO_RETRY,               /* retry, sending message again */
  RNET1_RADIO_RETRY_MSG_FAILED,    /* creating retry message failed */
  RNET1_RADIO_ACK_RECEIVED         /* acknowledge message received */
} RNET1_RadioEvent;

#define RNET1_CREATE_EVENTS   1  /* call user event handler */

void RF1_OnInterrupt(void);

void RNET1_Init(void);
/*
** ===================================================================
**     Method      :  RNET1_Init (component RNet)
**     Description :
**         Initializes the RNet Stack
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void RNET1_Deinit(void);
/*
** ===================================================================
**     Method      :  RNET1_Deinit (component RNet)
**     Description :
**         Deinitializes the RNet Stack
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

uint8_t RNET1_Process(void);
/*
** ===================================================================
**     Method      :  RNET1_Process (component RNet)
**     Description :
**         Processes the Radio Rx and Tx messages
**     Parameters  : None
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint8_t RNET1_PowerUp(void);
/*
** ===================================================================
**     Method      :  RNET1_PowerUp (component RNet)
**     Description :
**         Initializes and powers the radio up.
**     Parameters  : None
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint8_t RNET1_SetChannel(uint8_t channel);
/*
** ===================================================================
**     Method      :  RNET1_SetChannel (component RNet)
**     Description :
**         Sets the radio channel
**     Parameters  :
**         NAME            - DESCRIPTION
**         channel         - Channel number
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

uint8_t RNET1_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io);
/*
** ===================================================================
**     Method      :  RNET1_ParseCommand (component RNet)
**     Description :
**         Shell Command Line parser. This method is enabled/disabled
**         depending on if you have the Shell enabled/disabled in the
**         properties.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * cmd             - Pointer to command string
**       * handled         - Pointer to variable which tells if
**                           the command has been handled or not
**       * io              - Pointer to I/O structure
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

/* END RNET1. */

#endif
/* ifndef __RNET1_H */
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