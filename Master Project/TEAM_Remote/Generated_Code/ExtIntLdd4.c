/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : ExtIntLdd4.c
**     Project     : Andy_Johannes_Remote
**     Processor   : MK20DX128VFT5
**     Component   : ExtInt_LDD
**     Version     : Component 02.156, Driver 01.02, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-10-18, 14:57, # CodeGen: 4
**     Abstract    :
**         This component, "ExtInt_LDD", provide a low level API 
**         for unified access of external interrupts handling
**         across various device designs.
**         The component uses one pin which generates interrupt on 
**         selected edge.
**     Settings    :
**          Component name                                 : ExtIntLdd4
**          Pin                                            : ADC0_SE8/TSI0_CH0/PTB0/LLWU_P5/I2C0_SCL/FTM1_CH0/FTM1_QD_PHA
**          Pin signal                                     : Button_Middle
**          Generate interrupt on                          : falling edge
**          Interrupt                                      : INT_PORTB
**          Interrupt priority                             : medium priority
**          Initialization                                 : 
**            Enabled in init. code                        : yes
**            Auto initialization                          : yes
**     Contents    :
**         Init    - LDD_TDeviceData* ExtIntLdd4_Init(LDD_TUserData *UserDataPtr);
**         Enable  - void ExtIntLdd4_Enable(LDD_TDeviceData *DeviceDataPtr);
**         Disable - void ExtIntLdd4_Disable(LDD_TDeviceData *DeviceDataPtr);
**         GetVal  - bool ExtIntLdd4_GetVal(LDD_TDeviceData *DeviceDataPtr);
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
** @file ExtIntLdd4.c
** @version 01.02
** @brief
**         This component, "ExtInt_LDD", provide a low level API 
**         for unified access of external interrupts handling
**         across various device designs.
**         The component uses one pin which generates interrupt on 
**         selected edge.
*/         
/*!
**  @addtogroup ExtIntLdd4_module ExtIntLdd4 module documentation
**  @{
*/         

/* MODULE ExtIntLdd4. */

#include "SW4.h"
#include "ExtIntLdd4.h"
/* {Default RTOS Adapter} No RTOS includes */

#ifdef __cplusplus
extern "C" {
#endif 

typedef struct {
  LDD_TUserData *UserData;             /* RTOS device data structure */
} ExtIntLdd4_TDeviceData, *ExtIntLdd4_TDeviceDataPtr; /* Device data structure type */

/* {Default RTOS Adapter} Static object used for simulation of dynamic driver memory allocation */
static ExtIntLdd4_TDeviceData DeviceDataPrv__DEFAULT_RTOS_ALLOC;
/* {Default RTOS Adapter} Global variable used for passing a parameter into ISR */
static ExtIntLdd4_TDeviceData * INT_PORTB__DEFAULT_RTOS_ISRPARAM;

/*
** ===================================================================
**     Method      :  ExtIntLdd4_Init (component ExtInt_LDD)
*/
/*!
**     @brief
**         This method initializes the associated peripheral(s) and the
**         component internal variables. The method is called
**         automatically as a part of the application initialization
**         code.
**     @param
**         UserDataPtr     - Pointer to the RTOS device
**                           structure. This pointer will be passed to
**                           all events as parameter.
**     @return
**                         - Pointer to the dynamically allocated private
**                           structure or NULL if there was an error.
*/
/* ===================================================================*/
LDD_TDeviceData* ExtIntLdd4_Init(LDD_TUserData *UserDataPtr)
{
  /* Allocate LDD device structure */
  ExtIntLdd4_TDeviceData *DeviceDataPrv;

  /* {Default RTOS Adapter} Driver memory allocation: Dynamic allocation is simulated by a pointer to the static object */
  DeviceDataPrv = &DeviceDataPrv__DEFAULT_RTOS_ALLOC;
  /* Store the UserData pointer */
  DeviceDataPrv->UserData = UserDataPtr;
  /* Interrupt vector(s) allocation */
  /* {Default RTOS Adapter} Set interrupt vector: IVT is static, ISR parameter is passed by the global variable */
  INT_PORTB__DEFAULT_RTOS_ISRPARAM = DeviceDataPrv;
  /* Initialization of Port Control registers */
  /* PORTB_PCR0: ISF=0,MUX=1 */
  PORTB_PCR0 = (uint32_t)((PORTB_PCR0 & (uint32_t)~(uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_MUX(0x06)
               )) | (uint32_t)(
                PORT_PCR_MUX(0x01)
               ));
  /* PORTB_PCR0: ISF=1,IRQC=0x0A */
  PORTB_PCR0 = (uint32_t)((PORTB_PCR0 & (uint32_t)~(uint32_t)(
                PORT_PCR_IRQC(0x05)
               )) | (uint32_t)(
                PORT_PCR_ISF_MASK |
                PORT_PCR_IRQC(0x0A)
               ));
  /* NVICIP41: PRI41=0x80 */
  NVICIP41 = NVIC_IP_PRI41(0x80);
  /* NVICISER1: SETENA|=0x0200 */
  NVICISER1 |= NVIC_ISER_SETENA(0x0200);
  /* Registration of the device structure */
  PE_LDD_RegisterDeviceStructure(PE_LDD_COMPONENT_ExtIntLdd4_ID,DeviceDataPrv);
  return ((LDD_TDeviceData *)DeviceDataPrv);
}

/*
** ===================================================================
**     Method      :  ExtIntLdd4_Enable (component ExtInt_LDD)
*/
/*!
**     @brief
**         Enable the component - the external events are accepted.
**         This method is available only if HW module allows
**         enable/disable of the interrupt.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
*/
/* ===================================================================*/
void ExtIntLdd4_Enable(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */
  PORT_PDD_ClearPinInterruptFlag(PORTB_BASE_PTR, ExtIntLdd4_PIN_INDEX);
  PORT_PDD_SetPinInterruptConfiguration(PORTB_BASE_PTR,
    ExtIntLdd4_PIN_INDEX, PORT_PDD_INTERRUPT_ON_FALLING);
}

/*
** ===================================================================
**     Method      :  ExtIntLdd4_Disable (component ExtInt_LDD)
*/
/*!
**     @brief
**         Disable the component - the external events are not accepted.
**         This method is available only if HW module allows
**         enable/disable of the interrupt.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
*/
/* ===================================================================*/
void ExtIntLdd4_Disable(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */
  PORT_PDD_SetPinInterruptConfiguration(PORTB_BASE_PTR,
    ExtIntLdd4_PIN_INDEX, PORT_PDD_INTERRUPT_DMA_DISABLED);
}

/*
** ===================================================================
**     Method      :  ExtIntLdd4_Interrupt (component ExtInt_LDD)
**
**     Description :
**         The method services the interrupt of the selected peripheral(s)
**         and eventually invokes event(s) of the component.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void ExtIntLdd4_Interrupt(void)
{
  /* {Default RTOS Adapter} ISR parameter is passed through the global variable */
  ExtIntLdd4_TDeviceDataPtr DeviceDataPrv = INT_PORTB__DEFAULT_RTOS_ISRPARAM;

  /* Check the pin interrupt flag of the shared interrupt */
  if (PORT_PDD_GetPinInterruptFlag(PORTB_BASE_PTR, ExtIntLdd4_PIN_INDEX)) {
    /* Clear the interrupt flag */
    PORT_PDD_ClearPinInterruptFlag(PORTB_BASE_PTR, ExtIntLdd4_PIN_INDEX);
    /* Call OnInterrupt event */
    ExtIntLdd4_OnInterrupt(DeviceDataPrv->UserData);
  }
}

/*
** ===================================================================
**     Method      :  ExtIntLdd4_GetVal (component ExtInt_LDD)
*/
/*!
**     @brief
**         Returns the actual value of the input pin of the component.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     @return
**                         - Returned input value. Possible values:
**                           <false> - logical "0" (Low level) <true> -
**                           logical "1" (High level)
*/
/* ===================================================================*/
bool ExtIntLdd4_GetVal(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */
  if ((GPIO_PDD_GetPortDataInput(PTB_BASE_PTR) & ExtIntLdd4_PIN_MASK) != 0U) {
    return TRUE;
  } else {
    return FALSE;
  }
}

/* END ExtIntLdd4. */

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
