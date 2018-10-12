/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : IR4.c
**     Project     : Andy_Johannes_Robot
**     Processor   : MK22FX512VLK12
**     Component   : BitIO
**     Version     : Component 02.086, Driver 01.00, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-10-12, 16:00, # CodeGen: 3
**     Abstract    :
**         This component "BitIO" implements an one-bit input/output.
**         It uses one bit/pin of a port.
**         Methods of this component are mostly implemented as a macros
**         (if supported by target language and compiler).
**     Settings    :
**          Component name                                 : IR4
**          Pin for I/O                                    : ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/UART0_COL_b/FTM0_CH5/FB_AD1/EWM_OUT_b
**          Pin signal                                     : IR_4
**          BitIO_LDD                                      : BitIO_LDD
**          Direction                                      : Input/Output
**          Initialization                                 : 
**            Init. direction                              : Input
**            Init. value                                  : 0
**          Safe mode                                      : no
**          Optimization for                               : speed
**     Contents    :
**         SetDir    - void IR4_SetDir(bool Dir);
**         SetInput  - void IR4_SetInput(void);
**         SetOutput - void IR4_SetOutput(void);
**         GetVal    - bool IR4_GetVal(void);
**         PutVal    - void IR4_PutVal(bool Val);
**         ClrVal    - void IR4_ClrVal(void);
**         SetVal    - void IR4_SetVal(void);
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
** @file IR4.c
** @version 01.00
** @brief
**         This component "BitIO" implements an one-bit input/output.
**         It uses one bit/pin of a port.
**         Methods of this component are mostly implemented as a macros
**         (if supported by target language and compiler).
*/         
/*!
**  @addtogroup IR4_module IR4 module documentation
**  @{
*/         

/* MODULE IR4. */

#include "IR4.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
** ===================================================================
**     Method      :  IR4_SetDir (component BitIO)
**     Description :
**         This method sets direction of the component.
**     Parameters  :
**         NAME       - DESCRIPTION
**         Dir        - Direction to set (FALSE or TRUE)
**                      FALSE = Input, TRUE = Output
**     Returns     : Nothing
** ===================================================================
*/
/*
void IR4_SetDir(bool Dir)

**  This method is implemented as a macro. See IR4.h file.  **
*/

/*
** ===================================================================
**     Method      :  IR4_SetInput (component BitIO)
**     Description :
**         This method sets direction of the component to input.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void IR4_SetInput(void)

**  This method is implemented as a macro. See IR4.h file.  **
*/

/*
** ===================================================================
**     Method      :  IR4_SetOutput (component BitIO)
**     Description :
**         This method sets direction of the component to output.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void IR4_SetOutput(void)

**  This method is implemented as a macro. See IR4.h file.  **
*/

/*
** ===================================================================
**     Method      :  IR4_GetVal (component BitIO)
**     Description :
**         This method returns an input value.
**           a) direction = Input  : reads the input value from the
**                                   pin and returns it
**           b) direction = Output : returns the last written value
**     Parameters  : None
**     Returns     :
**         ---             - Input value. Possible values:
**                           FALSE - logical "0" (Low level)
**                           TRUE - logical "1" (High level)

** ===================================================================
*/
/*
bool IR4_GetVal(void)

**  This method is implemented as a macro. See IR4.h file.  **
*/

/*
** ===================================================================
**     Method      :  IR4_PutVal (component BitIO)
**     Description :
**         This method writes the new output value.
**         Note: If direction is set to input the method code may not
**               work properly.
**     Parameters  :
**         NAME       - DESCRIPTION
**         Val             - Output value. Possible values:
**                           FALSE - logical "0" (Low level)
**                           TRUE - logical "1" (High level)
**     Returns     : Nothing
** ===================================================================
*/
/*
void IR4_PutVal(bool Val)

**  This method is implemented as a macro. See IR4.h file.  **
*/

/*
** ===================================================================
**     Method      :  IR4_ClrVal (component BitIO)
**     Description :
**         This method clears (sets to zero) the output value.
**         Note: If direction is set to input the method code may not
**               work properly.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void IR4_ClrVal(void)

**  This method is implemented as a macro. See IR4.h file.  **
*/

/*
** ===================================================================
**     Method      :  IR4_SetVal (component BitIO)
**     Description :
**         This method sets (sets to one) the output value.
**         Note: If direction is set to input the method code may not
**               work properly.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void IR4_SetVal(void)

**  This method is implemented as a macro. See IR4.h file.  **
*/

/* END IR4. */

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
