/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : TMOUT1.c
**     Project     : Andy_Johannes_Robot
**     Processor   : MK22FX512VLK12
**     Component   : Timeout
**     Version     : Component 01.037, Driver 01.00, CPU db: 3.00.000
**     Repository  : My Components
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-10-12, 16:00, # CodeGen: 3
**     Abstract    :
**
The module implements timeout functionality. With this implementation,
it is possible to wait for a given time, and the time is counted by
a periodic interrupt.
**     Settings    :
**          Component name                                 : TMOUT1
**          SDK                                            : MCUC1
**          Critical Section                               : CS1
**          Maximum counters                               : 3
**          Counter tick period (ms)                       : 10
**          RTOS                                           : Disabled
**     Contents    :
**         GetCounter     - TMOUT1_CounterHandle TMOUT1_GetCounter(TMOUT1_CounterType nofTicks);
**         LeaveCounter   - void TMOUT1_LeaveCounter(TMOUT1_CounterHandle handle);
**         Value          - TMOUT1_CounterType TMOUT1_Value(TMOUT1_CounterHandle handle);
**         SetCounter     - TMOUT1_CounterType TMOUT1_SetCounter(TMOUT1_CounterHandle handle,...
**         CounterExpired - bool TMOUT1_CounterExpired(TMOUT1_CounterHandle handle);
**         AddTick        - void TMOUT1_AddTick(void);
**         Init           - void TMOUT1_Init(void);
**
**     * Copyright (c) 2011-2016, Erich Styger
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
** @file TMOUT1.c
** @version 01.00
** @brief
**
The module implements timeout functionality. With this implementation,
it is possible to wait for a given time, and the time is counted by
a periodic interrupt.
*/         
/*!
**  @addtogroup TMOUT1_module TMOUT1 module documentation
**  @{
*/         

/* MODULE TMOUT1. */

#include "TMOUT1.h"

#define TMOUT1_NOF_COUNTERS  3         /* number of timeout counters available */

static TMOUT1_CounterType TMOUT1_Counters[TMOUT1_NOF_COUNTERS]; /* array of timeout counters */
static bool TMOUT1_FreeCounters[TMOUT1_NOF_COUNTERS]; /* array to indicate which counters are free */

/*
** ===================================================================
**     Method      :  TMOUT1_GetCounter (component Timeout)
**     Description :
**         Initializes a new timeout counter and returns the handle to
**         it. At the end, use LeaveCounter() to free up the resource.
**     Parameters  :
**         NAME            - DESCRIPTION
**         nofTicks        - Number of ticks for the counter
**                           until it expires.
**     Returns     :
**         ---             - Handle to the counter, to be used for
**                           further API calls.
** ===================================================================
*/
TMOUT1_CounterHandle TMOUT1_GetCounter(TMOUT1_CounterType nofTicks)
{
  TMOUT1_CounterHandle handle;
  CS1_CriticalVariable();

  handle = 0;
  if (nofTicks==0) {
    nofTicks = 1; /* wait at least for one tick, otherwise will timeout immediately */
  }
  CS1_EnterCritical();
  while (handle<TMOUT1_NOF_COUNTERS && !TMOUT1_FreeCounters[handle]) {
    handle++;
  }
  if (handle<TMOUT1_NOF_COUNTERS) {
    TMOUT1_FreeCounters[handle]=FALSE;
    TMOUT1_Counters[handle] = nofTicks;
  }
  CS1_ExitCritical();
  if (handle==TMOUT1_NOF_COUNTERS) {
    return TMOUT1_OUT_OF_HANDLE;
  }
  return handle;
}

/*
** ===================================================================
**     Method      :  TMOUT1_LeaveCounter (component Timeout)
**     Description :
**         To be called to return the counter. Note that a counter
**         always should be returned so it can be reused.
**     Parameters  :
**         NAME            - DESCRIPTION
**         handle          - Counter handle
**     Returns     : Nothing
** ===================================================================
*/
void TMOUT1_LeaveCounter(TMOUT1_CounterHandle handle)
{
  CS1_CriticalVariable();

  if (handle==TMOUT1_OUT_OF_HANDLE) {
    return;
  }
  CS1_EnterCritical();
  TMOUT1_Counters[handle] = 0;
  TMOUT1_FreeCounters[handle]=TRUE;
  CS1_ExitCritical();
}

/*
** ===================================================================
**     Method      :  TMOUT1_CounterExpired (component Timeout)
**     Description :
**         Returns true if the timeout counter has been expired
**     Parameters  :
**         NAME            - DESCRIPTION
**         handle          - The timeout handle retrieved using
**                           GetCounter()
**     Returns     :
**         ---             - Returns TRUE if the counter has been
**                           expired, FALSE otherwise
** ===================================================================
*/
bool TMOUT1_CounterExpired(TMOUT1_CounterHandle handle)
{
  bool res;
  CS1_CriticalVariable();

  if (handle==TMOUT1_OUT_OF_HANDLE) {
    return TRUE;
  }
  CS1_EnterCritical();
  res = (bool)(TMOUT1_Counters[handle]==0);
  CS1_ExitCritical();
  return res;
}

/*
** ===================================================================
**     Method      :  TMOUT1_AddTick (component Timeout)
**     Description :
**         Method to be called from a periodic timer or interrupt. It
**         will decrement all current counters by one down to zero.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void TMOUT1_AddTick(void)
{
  uint8_t i;
  CS1_CriticalVariable();

  CS1_EnterCritical();
  for(i=0;i<TMOUT1_NOF_COUNTERS;i++) {
    if (TMOUT1_Counters[i]>0) {
      TMOUT1_Counters[i]--;
    }
  }
  CS1_ExitCritical();
}

/*
** ===================================================================
**     Method      :  TMOUT1_Init (component Timeout)
**     Description :
**         Initialization of the driver
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void TMOUT1_Init(void)
{
  uint8_t i;

  for(i=0;i<TMOUT1_NOF_COUNTERS;i++) {
    TMOUT1_Counters[i] = 0;
    TMOUT1_FreeCounters[i] = TRUE;
  }
}

/*
** ===================================================================
**     Method      :  TMOUT1_Value (component Timeout)
**     Description :
**         Return the current value of the counter (in ticks)
**     Parameters  :
**         NAME            - DESCRIPTION
**         handle          - Handle of the timeout counter
**     Returns     :
**         ---             - Returns the value of the timeout counter.
** ===================================================================
*/
TMOUT1_CounterType TMOUT1_Value(TMOUT1_CounterHandle handle)
{
  TMOUT1_CounterType val;
  CS1_CriticalVariable();

  if (handle==TMOUT1_OUT_OF_HANDLE) {
    return 0; /* return dummy value */
  }
  CS1_EnterCritical();
  val = TMOUT1_Counters[handle];
  CS1_ExitCritical();
  return val;
}

/*
** ===================================================================
**     Method      :  TMOUT1_SetCounter (component Timeout)
**     Description :
**         Sets the counter to a new value and returns the value just
**         prior to the call.
**     Parameters  :
**         NAME            - DESCRIPTION
**         handle          - Counter handle which shall get a new
**                           value.
**         nofTicks        - New value (tick count) of the
**                           timeout counter. Pass zero to have it
**                           expire immediately.
**     Returns     :
**         ---             - Value of counter before reset.
** ===================================================================
*/
TMOUT1_CounterType TMOUT1_SetCounter(TMOUT1_CounterHandle handle, TMOUT1_CounterType nofTicks)
{
  TMOUT1_CounterType res;
  CS1_CriticalVariable();

  if (handle==TMOUT1_OUT_OF_HANDLE) {
    return 0; /* return dummy value */
  }
  CS1_EnterCritical();
  res = TMOUT1_Counters[handle];
  TMOUT1_Counters[handle] = nofTicks;
  CS1_ExitCritical();
  return res;
}

/* END TMOUT1. */

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
