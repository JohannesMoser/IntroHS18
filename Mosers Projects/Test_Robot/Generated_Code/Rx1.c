/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : Rx1.c
**     Project     : TEAM_Robot
**     Processor   : MK22FX512VLL12
**     Component   : RingBuffer
**     Version     : Component 01.053, Driver 01.00, CPU db: 3.00.000
**     Repository  : My Components
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-09-27, 15:18, # CodeGen: 0
**     Abstract    :
**         This component implements a ring buffer for different integer data type.
**     Settings    :
**          Component name                                 : Rx1
**          SDK                                            : MCUC1
**          Reentrant                                      : yes
**          Critical Section                               : CS1
**          Buffer Elements                                : 128
**          Element Size                                   : 1 Byte
**     Contents    :
**         Clear           - void Rx1_Clear(void);
**         Put             - uint8_t Rx1_Put(Rx1_ElementType elem);
**         Get             - uint8_t Rx1_Get(Rx1_ElementType *elemP);
**         Peek            - uint8_t Rx1_Peek(Rx1_BufSizeType index, Rx1_ElementType *elemP);
**         Update          - uint8_t Rx1_Update(Rx1_BufSizeType index, Rx1_ElementType *elemP);
**         Putn            - uint8_t Rx1_Putn(Rx1_ElementType *elem, Rx1_BufSizeType nof);
**         Getn            - uint8_t Rx1_Getn(Rx1_ElementType *buf, Rx1_BufSizeType nof);
**         Compare         - uint8_t Rx1_Compare(Rx1_BufSizeType index, Rx1_ElementType *elemP,...
**         Delete          - uint8_t Rx1_Delete(void);
**         NofElements     - Rx1_BufSizeType Rx1_NofElements(void);
**         NofFreeElements - Rx1_BufSizeType Rx1_NofFreeElements(void);
**         Deinit          - void Rx1_Deinit(void);
**         Init            - void Rx1_Init(void);
**
**     * Copyright (c) 2014-2018, Erich Styger
**      * Web:         https://mcuoneclipse.com
**      * SourceForge: https://sourceforge.net/projects/mcuoneclipse
**      * Git:         https://github.com/ErichStyger/McuOnEclipse_PEx
**      * All rights reserved.
**      *
**      * Redistribution and use in source and binary forms, with or without modification,
**      * are permitted provided that the following conditions are met:
**      *
**      * - Redistributions of source code must retain the above copyright notice, this list
**      *   of conditions and the following disclaimer.
**      *
**      * - Redistributions in binary form must reproduce the above copyright notice, this
**      *   list of conditions and the following disclaimer in the documentation and/or
**      *   other materials provided with the distribution.
**      *
**      * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**      * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**      * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**      * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**      * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**      * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**      * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**      * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**      * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**      * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file Rx1.c
** @version 01.00
** @brief
**         This component implements a ring buffer for different integer data type.
*/         
/*!
**  @addtogroup Rx1_module Rx1 module documentation
**  @{
*/         

/* MODULE Rx1. */

#include "Rx1.h"

#if Rx1_CONFIG_REENTRANT
  #define Rx1_DEFINE_CRITICAL() CS1_CriticalVariable()
  #define Rx1_ENTER_CRITICAL()  CS1_EnterCritical()
  #define Rx1_EXIT_CRITICAL()   CS1_ExitCritical()
#else
  #define Rx1_DEFINE_CRITICAL() /* nothing */
  #define Rx1_ENTER_CRITICAL()  /* nothing */
  #define Rx1_EXIT_CRITICAL()   /* nothing */
#endif
static Rx1_ElementType Rx1_buffer[Rx1_CONFIG_BUF_SIZE]; /* ring buffer */
static Rx1_BufSizeType Rx1_inIdx;  /* input index */
static Rx1_BufSizeType Rx1_outIdx; /* output index */
static Rx1_BufSizeType Rx1_inSize; /* size data in buffer */
/*
** ===================================================================
**     Method      :  Rx1_Put (component RingBuffer)
**     Description :
**         Puts a new element into the buffer
**     Parameters  :
**         NAME            - DESCRIPTION
**         elem            - New element to be put into the buffer
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t Rx1_Put(Rx1_ElementType elem)
{
  uint8_t res = ERR_OK;
  Rx1_DEFINE_CRITICAL();

  Rx1_ENTER_CRITICAL();
  if (Rx1_inSize==Rx1_CONFIG_BUF_SIZE) {
    res = ERR_TXFULL;
  } else {
    Rx1_buffer[Rx1_inIdx] = elem;
    Rx1_inIdx++;
    if (Rx1_inIdx==Rx1_CONFIG_BUF_SIZE) {
      Rx1_inIdx = 0;
    }
    Rx1_inSize++;
  }
  Rx1_EXIT_CRITICAL();
  return res;
}

/*
** ===================================================================
**     Method      :  Rx1_Putn (component RingBuffer)
**     Description :
**         Put a number new element into the buffer.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * elem            - Pointer to new elements to be put into
**                           the buffer
**         nof             - number of elements
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t Rx1_Putn(Rx1_ElementType *elem, Rx1_BufSizeType nof)
{
  uint8_t res = ERR_OK;

  while(nof>0) {
    res = Rx1_Put(*elem);
    if (res!=ERR_OK) {
      break;
    }
    elem++; nof--;
  }
  return res;
}

/*
** ===================================================================
**     Method      :  Rx1_Get (component RingBuffer)
**     Description :
**         Removes an element from the buffer
**     Parameters  :
**         NAME            - DESCRIPTION
**       * elemP           - Pointer to where to store the received
**                           element
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t Rx1_Get(Rx1_ElementType *elemP)
{
  uint8_t res = ERR_OK;
  Rx1_DEFINE_CRITICAL();

  Rx1_ENTER_CRITICAL();
  if (Rx1_inSize==0) {
    res = ERR_RXEMPTY;
  } else {
    *elemP = Rx1_buffer[Rx1_outIdx];
    Rx1_inSize--;
    Rx1_outIdx++;
    if (Rx1_outIdx==Rx1_CONFIG_BUF_SIZE) {
      Rx1_outIdx = 0;
    }
  }
  Rx1_EXIT_CRITICAL();
  return res;
}

/*
** ===================================================================
**     Method      :  Rx1_Getn (component RingBuffer)
**     Description :
**         Get a number elements into a buffer.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * buf             - Pointer to buffer where to store the
**                           elements
**         nof             - number of elements
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t Rx1_Getn(Rx1_ElementType *buf, Rx1_BufSizeType nof)
{
  uint8_t res = ERR_OK;

  while(nof>0) {
    res = Rx1_Get(buf);
    if (res!=ERR_OK) {
      break;
    }
    buf++; nof--;
  }
  return res;
}

/*
** ===================================================================
**     Method      :  Rx1_NofElements (component RingBuffer)
**     Description :
**         Returns the actual number of elements in the buffer.
**     Parameters  : None
**     Returns     :
**         ---             - Number of elements in the buffer.
** ===================================================================
*/
Rx1_BufSizeType Rx1_NofElements(void)
{
  return Rx1_inSize;
}

/*
** ===================================================================
**     Method      :  Rx1_NofFreeElements (component RingBuffer)
**     Description :
**         Returns the actual number of free elements/space in the
**         buffer.
**     Parameters  : None
**     Returns     :
**         ---             - Number of elements in the buffer.
** ===================================================================
*/
Rx1_BufSizeType Rx1_NofFreeElements(void)
{
  return (Rx1_BufSizeType)(Rx1_CONFIG_BUF_SIZE-Rx1_inSize);
}

/*
** ===================================================================
**     Method      :  Rx1_Init (component RingBuffer)
**     Description :
**         Initializes the data structure
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Rx1_Init(void)
{
  Rx1_inIdx = 0;
  Rx1_outIdx = 0;
  Rx1_inSize = 0;
}

/*
** ===================================================================
**     Method      :  Rx1_Clear (component RingBuffer)
**     Description :
**         Clear (empty) the ring buffer.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void Rx1_Clear(void)
{
  Rx1_DEFINE_CRITICAL();

  Rx1_ENTER_CRITICAL();
  Rx1_Init();
  Rx1_EXIT_CRITICAL();
}

/*
** ===================================================================
**     Method      :  Rx1_Peek (component RingBuffer)
**     Description :
**         Returns an element of the buffer without removiing it.
**     Parameters  :
**         NAME            - DESCRIPTION
**         index           - Index of element. 0 peeks the top
**                           element, 1 the next, and so on.
**       * elemP           - Pointer to where to store the received
**                           element
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t Rx1_Peek(Rx1_BufSizeType index, Rx1_ElementType *elemP)
{
  uint8_t res = ERR_OK;
  int idx; /* index inside ring buffer */
  Rx1_DEFINE_CRITICAL();

  Rx1_ENTER_CRITICAL();
  if (index>=Rx1_CONFIG_BUF_SIZE) {
    res = ERR_OVERFLOW; /* asking for an element outside of ring buffer size */
  } else if (index<Rx1_inSize) {
    idx = (Rx1_outIdx+index)%Rx1_CONFIG_BUF_SIZE;
    *elemP = Rx1_buffer[idx];
  } else { /* asking for an element which does not exist */
    res = ERR_RXEMPTY;
  }
  Rx1_EXIT_CRITICAL();
  return res;
}

/*
** ===================================================================
**     Method      :  Rx1_Compare (component RingBuffer)
**     Description :
**         Compares the elements in the buffer.
**     Parameters  :
**         NAME            - DESCRIPTION
**         index           - Index of element. 0 peeks the top
**                           element, 1 the next, and so on.
**       * elemP           - Pointer to elements to compare with
**         nof             - number of elements to compare
**     Returns     :
**         ---             - zero if elements are the same, -1 otherwise
** ===================================================================
*/
uint8_t Rx1_Compare(Rx1_BufSizeType index, Rx1_ElementType *elemP, Rx1_BufSizeType nof)
{
  uint8_t cmpResult = 0;
  uint8_t res;
  Rx1_ElementType val;

  while(nof>0) {
    res = Rx1_Peek(index, &val);
    if (res!=ERR_OK) { /* general failure? */
      cmpResult = (uint8_t)-1; /* no match */
      break;
    }
    if (val!=*elemP) { /* mismatch */
      cmpResult = (uint8_t)-1; /* no match */
      break;
    }
    elemP++; index++; nof--;
  }

  return cmpResult;
}

/*
** ===================================================================
**     Method      :  Rx1_Deinit (component RingBuffer)
**     Description :
**         Driver de-initialization
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/**
void Rx1_Deinit(void)
{
  ** Function is implemented as macro in the header file
}
*/
/*
** ===================================================================
**     Method      :  Rx1_Delete (component RingBuffer)
**     Description :
**         Removes an element from the buffer
**     Parameters  : None
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t Rx1_Delete(void)
{
  uint8_t res = ERR_OK;
  Rx1_DEFINE_CRITICAL();

  Rx1_ENTER_CRITICAL();
  if (Rx1_inSize==0) {
    res = ERR_RXEMPTY;
  } else {
    Rx1_inSize--;
    Rx1_outIdx++;
    if (Rx1_outIdx==Rx1_CONFIG_BUF_SIZE) {
      Rx1_outIdx = 0;
    }
  }
  Rx1_EXIT_CRITICAL();
  return res;
}

/*
** ===================================================================
**     Method      :  Rx1_Update (component RingBuffer)
**     Description :
**         Updates the data of an element.
**     Parameters  :
**         NAME            - DESCRIPTION
**         index           - Index of element. 0 peeks the top
**                           element, 1 the next, and so on.
**       * elemP           - Pointer to where to store the received
**                           element
**     Returns     :
**         ---             - Error code
** ===================================================================
*/
uint8_t Rx1_Update(Rx1_BufSizeType index, Rx1_ElementType *elemP)
{
  uint8_t res = ERR_OK;
  int idx; /* index inside ring buffer */
  Rx1_DEFINE_CRITICAL();

  Rx1_ENTER_CRITICAL();
  if (index>=Rx1_CONFIG_BUF_SIZE) {
    res = ERR_OVERFLOW; /* asking for an element outside of ring buffer size */
  } else if (index<Rx1_inSize) {
    idx = (Rx1_outIdx+index)%Rx1_CONFIG_BUF_SIZE;
    Rx1_buffer[idx] = *elemP; /* replace element */
  } else { /* asking for an element which does not exist */
    res = ERR_RXEMPTY;
  }
  Rx1_EXIT_CRITICAL();
  return res;
}

/* END Rx1. */

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
