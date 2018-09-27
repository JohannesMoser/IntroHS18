/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : USB1.h
**     Project     : TEAM_Robot
**     Processor   : MK22FX512VLL12
**     Component   : FSL_USB_Stack
**     Version     : Component 01.051, Driver 01.00, CPU db: 3.00.000
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-09-27, 15:18, # CodeGen: 0
**     Abstract    :
**         This component implements a wrapper to the FSL USB Stack.
**     Settings    :
**          Component name                                 : USB1
**          Freescale USB Stack Version                    : v4.1.1
**          USB Init                                       : Init_USB_OTG_VAR0
**          Device Class                                   : CDC Device
**          CDC Device                                     : Enabled
**            CDCDevice                                    : FSL_USB_CDC_Device
**          CDC Host                                       : Disabled
**          HID Keyboard Device                            : Disabled
**          HID Mouse Device                               : Disabled
**          MSD Host                                       : Disabled
**          DATA_BUFF_SIZE                                 : 64
**          Initialization                                 : 
**            Use USB Stack Inititalization                : yes
**            Call Init Method                             : yes
**     Contents    :
**         Deinit - uint8_t USB1_Deinit(void);
**         Init   - uint8_t USB1_Init(void);
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
** @file USB1.h
** @version 01.00
** @brief
**         This component implements a wrapper to the FSL USB Stack.
*/         
/*!
**  @addtogroup USB1_module USB1 module documentation
**  @{
*/         

#ifndef __USB1_H
#define __USB1_H

/* MODULE USB1. */
#include "MCUC1.h" /* SDK and API used */
#include "USB1config.h" /* configuration */

/* Include inherited components */
#include "MCUC1.h"
#include "CDC1.h"

#include <stddef.h> /* for size_t */

/* Interfaces/wrappers to the CDC device class, needed for Shell if using default serial/connection to USB (CDC): */
#define USB1_SendString      CDC1_SendString
#define USB1_RecvChar        CDC1_GetChar
#define USB1_SendChar        CDC1_SendChar
#define USB1_GetCharsInRxBuf CDC1_GetCharsInRxBuf

#include "Cpu.h"


#ifndef __BWUserType_USB1_TComData
#define __BWUserType_USB1_TComData
  typedef uint8_t USB1_TComData ;      /* User type for communication data type. */
#endif

/*
   DATA_BUFF_SIZE should be greater than or equal to the endpoint buffer size,
   otherwise there will be data loss. For MC9S08JS16, maximum DATA_BUFF_SIZE
   supported is 16 Bytes
*/
#define USB1_DATA_BUFF_SIZE    64 /* data buffer size as specified in the properties */

#define USB1_USB_ERR_SEND            1  /* Error while sending */
#define USB1_USB_ERR_BUSOFF          2  /* Bus not ready */
#define USB1_USB_ERR_INIT            3  /* USB initialization error */
#define USB1_USB_ERR_TX_CHAR         4  /* Error sending character */
#define USB1_USB_ERR_TX_STRING       5  /* Error sending string */
#define USB1_USB_ERR_CHECKED_TXFULL  6  /* Error during sending a checked block */
#define USB1_USB_ERR_RECEIVE         7  /* Error while starting a receive transaction */
#define USB1_USB_ERR_DEINIT          8  /* USB deinitialization error */

uint8_t USB1_Init(void);
/*
** ===================================================================
**     Method      :  USB1_Init (component FSL_USB_Stack)
**     Description :
**         Initializes the driver
**     Parameters  : None
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

void USB1_usb_int_dis(void);
/*
** ===================================================================
**     Method      :  USB1_usb_int_dis (component FSL_USB_Stack)
**
**     Description :
**         Disables USB interrupts (if supported)
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void USB1_usb_int_en(void);
/*
** ===================================================================
**     Method      :  USB1_usb_int_en (component FSL_USB_Stack)
**
**     Description :
**         Enables USB interrupts (if supported).
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

uint8_t USB1_Deinit(void);
/*
** ===================================================================
**     Method      :  USB1_Deinit (component FSL_USB_Stack)
**     Description :
**         Deinitializes the driver
**     Parameters  : None
**     Returns     :
**         ---             - Error code
** ===================================================================
*/

/* END USB1. */

#endif
/* ifndef __USB1_H */
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
