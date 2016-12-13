/*******************************************************************************
* File Name: IR_emitter.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "IR_emitter.h"


/*******************************************************************************
* Function Name: IR_emitter_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void IR_emitter_Write(uint8 value) 
{
    uint8 staticBits = (IR_emitter_DR & (uint8)(~IR_emitter_MASK));
    IR_emitter_DR = staticBits | ((uint8)(value << IR_emitter_SHIFT) & IR_emitter_MASK);
}


/*******************************************************************************
* Function Name: IR_emitter_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  IR_emitter_DM_STRONG     Strong Drive 
*  IR_emitter_DM_OD_HI      Open Drain, Drives High 
*  IR_emitter_DM_OD_LO      Open Drain, Drives Low 
*  IR_emitter_DM_RES_UP     Resistive Pull Up 
*  IR_emitter_DM_RES_DWN    Resistive Pull Down 
*  IR_emitter_DM_RES_UPDWN  Resistive Pull Up/Down 
*  IR_emitter_DM_DIG_HIZ    High Impedance Digital 
*  IR_emitter_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void IR_emitter_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(IR_emitter_0, mode);
}


/*******************************************************************************
* Function Name: IR_emitter_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro IR_emitter_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 IR_emitter_Read(void) 
{
    return (IR_emitter_PS & IR_emitter_MASK) >> IR_emitter_SHIFT;
}


/*******************************************************************************
* Function Name: IR_emitter_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 IR_emitter_ReadDataReg(void) 
{
    return (IR_emitter_DR & IR_emitter_MASK) >> IR_emitter_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(IR_emitter_INTSTAT) 

    /*******************************************************************************
    * Function Name: IR_emitter_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 IR_emitter_ClearInterrupt(void) 
    {
        return (IR_emitter_INTSTAT & IR_emitter_MASK) >> IR_emitter_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
